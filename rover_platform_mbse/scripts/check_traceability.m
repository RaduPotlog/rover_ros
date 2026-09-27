function report = check_traceability()
%CHECK_TRACEABILITY Coverage and orphan check over the requirement sets and links.
%   Errors - model integrity; the function throws after printing them:
%     - a SYS-SR whose compliance verdict is not 'Not SW' has no derived SWR
%     - a SWR has no parent SYS-SR and no derived rationale
%     - a SWR has no implementing architecture component
%   Verification gaps - findings about rover_ros, reported, not fatal:
%     - an As-Built SWR verified by Test with neither a rover_ros test (VerifiedBy)
%       nor a model test
%   Planned - a Proposed SWR verified by Test (its test comes with the implementation).

root = mbse.root();
slreq.clear();
load_system('RoverPlatformArch');   % resolves the implement links
cleanup = onCleanup(@() cleanupAll());
sysSet = slreq.load(fullfile(root, 'requirements', 'ROVER_ROS_SYS-SR.slreqx'));
comp = mbse.readJson(fullfile('data', 'sys_sr_compliance.json'));
verdict = containers.Map({comp.items.id}, {comp.items.verdict});
swFiles = dir(fullfile(root, 'requirements', 'SWRS_*.slreqx'));
for k = 1:numel(swFiles)
    slreq.load(fullfile(swFiles(k).folder, swFiles(k).name));
end
modelTests = mbse.testVerifies();
testedIds = [modelTests.ids];

errors = {};
gaps = {};
planned = {};
sysReqs = sysSet.find('Type', 'Requirement');
for k = 1:numel(sysReqs)
    id = sysReqs(k).Id;
    nDerived = countLinks(sysReqs(k).inLinks(), 'Derive');
    if nDerived == 0 && ~strcmp(verdict(id), 'Not SW')
        errors{end + 1} = sprintf('%s: no derived SWR (verdict %s)', id, verdict(id)); %#ok<AGROW>
    end
end

nSwr = 0;
for k = 1:numel(swFiles)
    rs = slreq.find('type', 'ReqSet', 'Name', erase(swFiles(k).name, '.slreqx'));
    reqs = rs.find('Type', 'Requirement');
    for j = 1:numel(reqs)
        q = reqs(j);
        nSwr = nSwr + 1;
        if countLinks(q.outLinks(), 'Derive') == 0 && isempty(q.getAttribute('DerivedRationale'))
            errors{end + 1} = sprintf('%s: no parent SYS-SR and no derived rationale', q.Id); %#ok<AGROW>
        end
        if countLinks(q.inLinks(), 'Implement') == 0
            errors{end + 1} = sprintf('%s: no implementing component', q.Id); %#ok<AGROW>
        end
        isTest = contains(q.getAttribute('VerificationMethod'), 'Test');
        hasTest = ~isempty(q.getAttribute('VerifiedBy')) || any(strcmp(testedIds, q.Id));
        if isTest && ~hasTest
            if strcmp(q.getAttribute('Status'), 'Proposed')
                planned{end + 1} = q.Id; %#ok<AGROW>
            else
                gaps{end + 1} = q.Id; %#ok<AGROW>
            end
        end
    end
end

report = struct('sysSr', numel(sysReqs), 'swr', nSwr, 'errors', {errors}, ...
    'verificationGaps', {gaps}, 'plannedTests', {planned});
fprintf('check_traceability: %d SYS-SR, %d SWR, %d errors\n', numel(sysReqs), nSwr, numel(errors));
cellfun(@(s) fprintf('  ERROR %s\n', s), errors);
fprintf('  verification gaps (As-Built, Test, no test): %d  %s\n', numel(gaps), strjoin(gaps, ' '));
fprintf('  tests to come with Proposed SWRs: %d\n', numel(planned));
if ~isempty(errors)
    error('check_traceability:failed', '%d traceability errors', numel(errors));
end
end

function n = countLinks(links, type)
n = 0;
for k = 1:numel(links)
    n = n + strcmp(links(k).Type, type);
end
end

function cleanupAll()
slreq.clear();
close_system('RoverPlatformArch', 0);
end
