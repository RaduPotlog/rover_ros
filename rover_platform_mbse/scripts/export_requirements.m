function export_requirements()
%EXPORT_REQUIREMENTS Write the review spreadsheets in export/ from the .slreqx masters.
%   ROVER-A1-<PACKAGE>-SWRS-V-001.xlsx        one SWRS per package (SYS-SR column layout)
%   ROVER-A1-ROVER_ROS_SYS-SR-COMPLIANCE-V-001.xlsx  SYS-SR compliance, findings, model
%                                             tests, verification gaps, open questions
%   ROVER-A1-ROVER_ROS-TRACEABILITY-V-001.xlsx SYS-SR x package matrix, SWR trace list
%   Model test results come from results/model_test_results.json (run_model_tests).

root = mbse.root();
outDir = fullfile(root, 'export');
rep = check_traceability();   % first: it clears all loaded requirement sets when done
slreq.clear();
load_system('RoverPlatformArch');
cleanup = onCleanup(@() cleanupAll());
sysSet = slreq.load(fullfile(root, 'requirements', 'ROVER_ROS_SYS-SR.slreqx'));
comp = mbse.readJson(fullfile('data', 'sys_sr_compliance.json'));
testRes = readTestResults(root);
tags = mbse.testVerifies();

specFiles = dir(fullfile(root, 'data', 'swrs', '*.json'));
specs = cell(1, numel(specFiles));
for k = 1:numel(specFiles)
    specs{k} = jsondecode(fileread(fullfile(specFiles(k).folder, specFiles(k).name)));
end
today = char(datetime('today', 'Format', 'yyyy-MM-dd'));

%% One SWRS per package
hdr = {'No', 'Category', 'Requirement', 'Rationale', 'Verification Method', ...
    'Acceptance Criteria', 'Priority', 'Status', 'Trace / Depends On', ...
    'Evidence (src/rover_ros path:line)', 'Implementation', 'Verified By (rover_ros tests)', ...
    'Model Verification', 'Comments'};
allSwr = {};   % id, package, requirement object
for k = 1:numel(specs)
    s = specs{k};
    rs = slreq.load(fullfile(root, 'requirements', ['SWRS_' s.prefix '.slreqx']));
    reqs = rs.find('Type', 'Requirement');
    rows = cell(numel(reqs), numel(hdr));
    for j = 1:numel(reqs)
        q = reqs(j);
        rows(j, :) = {q.Id, att(q, 'Category'), q.Description, q.Rationale, ...
            att(q, 'VerificationMethod'), att(q, 'AcceptanceCriteria'), att(q, 'Priority'), ...
            att(q, 'Status'), att(q, 'ParentSysSR'), att(q, 'Evidence'), ...
            att(q, 'Implementation'), att(q, 'VerifiedBy'), modelVerification(q.Id, tags, testRes), ...
            strtrim(strjoin({att(q, 'DerivedRationale'), att(q, 'Comments')}, ' '))};
        allSwr(end + 1, :) = {q.Id, s.package, q}; %#ok<AGROW>
    end
    file = fullfile(outDir, [s.document '.xlsx']);
    freshFile(file);
    writecell([hdr; rows], file, 'Sheet', 'Requirements');
    writecell({ ...
        'Document', s.document; 'Title', s.title; 'Package', ['src/rover_ros/' s.package]; ...
        'Version', 'V-001'; 'Date', today; 'Scope', s.scope; ...
        'Master', ['rover_platform_mbse/requirements/SWRS_' s.prefix '.slreqx (Requirements Toolbox)']; ...
        'Parent', 'ROVER-A1-ROVER_ROS_SYS-SR-V-002'; ...
        'Status values', 'As-Built = reverse-engineered from the code; Proposed = needed by a SYS-SR, not (fully) in the code'; ...
        'Implementation values', 'Implemented / Partial / Gap (not implemented) / Deviation (differs from the SYS-SR or the documentation)'}, ...
        file, 'Sheet', 'Document');
    writecell({'Version', 'Date', 'Author', 'Changes'; 'V-001', today, ...
        'Draft for review (prepared with Claude Code)', ...
        'First issue: reverse-engineered from src/rover_ros and traced to SYS-SR V-002.'}, ...
        file, 'Sheet', 'Revision History');
end

%% Compliance workbook
file = fullfile(outDir, 'ROVER-A1-ROVER_ROS_SYS-SR-COMPLIANCE-V-001.xlsx');
freshFile(file);
items = containers.Map({comp.items.id}, num2cell(comp.items));
sysReqs = sysSet.find('Type', 'Requirement');
rows = cell(numel(sysReqs), 10);
for k = 1:numel(sysReqs)
    q = sysReqs(k);
    it = items(q.Id);
    derived = derivedIds(q);
    rows(k, :) = {q.Id, att(q, 'Category'), q.Description, att(q, 'Priority'), att(q, 'Status'), ...
        it.allocation, it.verdict, it.summary, strjoin(derived, ', '), ...
        modelVerification(q.Id, tags, testRes)};
end
writecell([{'No', 'Category', 'Requirement', 'Priority', 'SYS-SR Status', 'Allocation', ...
    'Verdict', 'Assessment (as-built, with evidence)', 'Derived SWRs', 'Model Verification'}; rows], ...
    file, 'Sheet', 'Compliance');
writecell([{'Verdict', 'Meaning'}; {comp.verdicts.verdict}', {comp.verdicts.meaning}'], ...
    file, 'Sheet', 'Verdict Legend');

findings = {};
for k = 1:size(allSwr, 1)
    q = allSwr{k, 3};
    if ~strcmp(att(q, 'Implementation'), 'Implemented')
        findings(end + 1, :) = {q.Id, allSwr{k, 2}, att(q, 'Implementation'), att(q, 'Status'), ...
            att(q, 'ParentSysSR'), q.Description, att(q, 'Evidence'), att(q, 'Comments')}; %#ok<AGROW>
    end
end
writecell([{'SWR', 'Package', 'Implementation', 'Status', 'Parent SYS-SR', 'Requirement', ...
    'Evidence', 'Comments'}; findings], file, 'Sheet', 'Findings');

tr = cell(numel(testRes), 5);
for k = 1:numel(testRes)
    r = testRes(k);
    t = tags(strcmp({tags.name}, r.name));
    verifies = '';
    if ~isempty(t)
        verifies = strjoin(t.ids, ', ');
    end
    tr(k, :) = {r.name, resultText(r), logical(r.knownDeviation), verifies, r.message};
end
writecell([{'Model test', 'Result', 'Known deviation', 'Verifies', 'Message'}; tr], file, 'Sheet', 'Model Tests');

gapRows = cell(numel(rep.verificationGaps), 3);
for k = 1:numel(rep.verificationGaps)
    q = allSwr{strcmp(allSwr(:, 1), rep.verificationGaps{k}), 3};
    gapRows(k, :) = {q.Id, att(q, 'Implementation'), q.Description};
end
writecell([{'SWR (As-Built, verification by Test, no test exists)', 'Implementation', 'Requirement'}; gapRows], ...
    file, 'Sheet', 'Verification Gaps');
oq = comp.open_questions;
writecell([{'Ref', 'Question', 'Answer', 'Answered By'}; ...
    [{oq.ref}', {oq.question}', repmat({''}, numel(oq), 2)]], file, 'Sheet', 'Open Questions');

%% Traceability workbook
file = fullfile(outDir, 'ROVER-A1-ROVER_ROS-TRACEABILITY-V-001.xlsx');
freshFile(file);
pkgs = cellfun(@(s) s.package, specs, 'UniformOutput', false);
M = cell(numel(sysReqs), numel(pkgs));
M(:) = {''};
for k = 1:size(allSwr, 1)
    parents = strtrim(strsplit(att(allSwr{k, 3}, 'ParentSysSR'), ','));
    c = strcmp(pkgs, allSwr{k, 2});
    for p = 1:numel(parents)
        r = find(strcmp({sysReqs.Id}, parents{p}));
        if ~isempty(r)
            M{r, c} = strtrim(strjoin({M{r, c}, allSwr{k, 1}}, ' '));
        end
    end
end
writecell([[{'SYS-SR'}, pkgs]; [{sysReqs.Id}', M]], file, 'Sheet', 'SYS-SR x Package');
list = cell(size(allSwr, 1), 6);
for k = 1:size(allSwr, 1)
    q = allSwr{k, 3};
    t = tags(cellfun(@(ids) any(strcmp(ids, q.Id)), {tags.ids}));
    list(k, :) = {q.Id, allSwr{k, 2}, att(q, 'ParentSysSR'), ...
        ['RoverPlatformArch/.../' allSwr{k, 2}], att(q, 'VerifiedBy'), strjoin({t.name}, '; ')};
end
writecell([{'SWR', 'Package', 'Derived from (SYS-SR)', 'Implemented by (architecture)', ...
    'rover_ros tests', 'Model tests'}; list], file, 'Sheet', 'SWR Trace');
fprintf('export_requirements: %d SWRS workbooks, compliance and traceability workbooks in export/\n', numel(specs));
end

function v = att(q, name)
v = q.getAttribute(name);
if isempty(v)
    v = '';
end
end

function ids = derivedIds(q)
ids = {};
links = q.inLinks();
for k = 1:numel(links)
    if strcmp(links(k).Type, 'Derive')
        src = slreq.structToObj(links(k).source());
        ids{end + 1} = src.Id; %#ok<AGROW>
    end
end
ids = sort(unique(ids));
end

function s = modelVerification(id, tags, res)
s = '';
t = tags(cellfun(@(ids) any(strcmp(ids, id)), {tags.ids}));
parts = cell(1, numel(t));
for k = 1:numel(t)
    r = res(strcmp({res.name}, t(k).name));
    status = 'not run';
    if ~isempty(r)
        status = resultText(r);
    end
    parts{k} = sprintf('%s: %s', t(k).name, status);
end
if ~isempty(parts)
    s = strjoin(parts, '; ');
end
end

function s = resultText(r)
if r.passed
    s = 'PASS';
elseif r.knownDeviation
    s = 'FAIL (known deviation)';
else
    s = 'FAIL';
end
end

function res = readTestResults(root)
f = fullfile(root, 'results', 'model_test_results.json');
res = struct('name', {}, 'passed', {}, 'knownDeviation', {}, 'message', {});
if isfile(f)
    d = jsondecode(fileread(f));
    res = d.results;
end
end

function freshFile(file)
if isfile(file)
    delete(file);
end
end

function cleanupAll()
slreq.clear();
close_system('RoverPlatformArch', 0);
end
