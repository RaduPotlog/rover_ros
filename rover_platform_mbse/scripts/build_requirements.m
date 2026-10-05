function build_requirements(opts)
%BUILD_REQUIREMENTS Build the Requirements Toolbox sets in requirements/.
%   ROVER_ROS_SYS-SR.slreqx  system requirements, updated in place from
%                            system/ROVER-A1-ROVER_ROS_SYS-SR-V-002.xlsx on every run
%                            (the xlsx is the system-level master).
%   SWRS_<PREFIX>.slreqx     one software requirements specification per package,
%                            created from data/swrs/<package>.json. Once created, the
%                            .slreqx is the master and is NOT overwritten unless
%                            build_requirements('Rebuild', true).

% Copyright 2026 Mechatronics Academy
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

arguments
    opts.Rebuild (1, 1) logical = false
end

root = mbse.root();
reqDir = fullfile(root, 'requirements');
slreq.clear();

%% Software requirements specifications
files = dir(fullfile(root, 'data', 'swrs', '*.json'));
swAttrs = {'Category', 'VerificationMethod', 'AcceptanceCriteria', 'Priority', 'Status', ...
    'ParentSysSR', 'DerivedRationale', 'Evidence', 'Implementation', 'VerifiedBy', 'Comments'};
nSets = 0;
nSwr = 0;
kept = {};
for f = 1:numel(files)
    spec = jsondecode(fileread(fullfile(files(f).folder, files(f).name)));
    setFile = fullfile(reqDir, ['SWRS_' spec.prefix '.slreqx']);
    if isfile(setFile) && ~opts.Rebuild
        kept{end + 1} = ['SWRS_' spec.prefix]; %#ok<AGROW>
        continue
    end
    % Delete the set and its link set (SWRS_X~slreqx.slmx, derive links; rebuilt by
    % build_trace_links). Built before any other set is loaded: loading the SYS-SR set
    % would pull these link sets in and block re-creating the set under its name.
    linkFile = strrep(setFile, '.slreqx', '~slreqx.slmx');
    for old = {setFile, linkFile}
        if isfile(old{1})
            delete(old{1});
        end
    end
    ss = slreq.new(setFile);
    ss.Description = sprintf('%s (%s). %s', spec.title, spec.document, spec.scope);
    ensureAttributes(ss, swAttrs);
    reqs = mbse.items(spec.requirements);
    for k = 1:numel(reqs)
        q = reqs{k};
        req = ss.add('Id', q.id);
        req.Summary = sprintf('%s: %s', q.category, shorten(q.text));
        req.Description = q.text;
        req.Rationale = q.rationale;
        req.setAttribute('Category', q.category);
        req.setAttribute('VerificationMethod', q.verification);
        req.setAttribute('AcceptanceCriteria', q.acceptance);
        req.setAttribute('Priority', q.priority);
        req.setAttribute('Status', q.status);
        req.setAttribute('ParentSysSR', joinList(q.parents, ', '));
        req.setAttribute('DerivedRationale', q.derived_rationale);
        req.setAttribute('Evidence', joinList(q.evidence, '; '));
        req.setAttribute('Implementation', q.implementation);
        req.setAttribute('VerifiedBy', joinList(q.verified_by, '; '));
        req.setAttribute('Comments', q.comments);
        nSwr = nSwr + 1;
    end
    ss.save();
    nSets = nSets + 1;
end
slreq.clear();

%% System requirements from the xlsx
xlsx = fullfile(root, 'system', 'ROVER-A1-ROVER_ROS_SYS-SR-V-002.xlsx');
C = readcell(xlsx, 'Sheet', 'Requirements');
hdr = string(C(1, :));
col = @(name) find(startsWith(hdr, name), 1);
sysFile = fullfile(reqDir, 'ROVER_ROS_SYS-SR.slreqx');
if isfile(sysFile)
    rs = slreq.load(sysFile);
else
    rs = slreq.new(sysFile);
end
rs.Description = ['ROVER-A1 rover_ros platform system requirements, imported from ' ...
    'system/ROVER-A1-ROVER_ROS_SYS-SR-V-002.xlsx (the master). Re-run build_requirements after editing the xlsx.'];
sysAttrs = {'Category', 'VerificationMethod', 'AcceptanceCriteria', 'Priority', 'Status', ...
    'TraceDependsOn', 'OriginalText', 'Comments'};
sysCols = {'Category', 'Verification Method', 'Acceptance Criteria', 'Priority', 'Status', ...
    'Trace', 'Original Text', 'Comments'};
ensureAttributes(rs, sysAttrs);
nSys = 0;
for r = 2:size(C, 1)
    id = cellText(C{r, col('No')});
    if ~startsWith(id, 'SYS-SR-')
        continue
    end
    req = rs.find('Type', 'Requirement', 'Id', id);
    if isempty(req)
        req = rs.add('Id', id);
    end
    text = cellText(C{r, col('Requirement')});
    req.Summary = sprintf('%s: %s', cellText(C{r, col('Category')}), shorten(text));
    req.Description = text;
    req.Rationale = cellText(C{r, col('Rationale')});
    for a = 1:numel(sysAttrs)
        req.setAttribute(sysAttrs{a}, cellText(C{r, col(sysCols{a})}));
    end
    nSys = nSys + 1;
end
rs.save();

fprintf('build_requirements: %d SYS-SR; %d SWRS sets (%d SWR) built', nSys, nSets, nSwr);
if ~isempty(kept)
    fprintf('; %d kept as master: %s', numel(kept), strjoin(kept, ', '));
end
fprintf('\n');
slreq.clear();
end

function ensureAttributes(rs, names)
existing = rs.CustomAttributeNames;
for k = 1:numel(names)
    if ~any(strcmp(existing, names{k}))
        rs.addAttribute(names{k}, 'Edit');
    end
end
end

function s = cellText(v)
if isa(v, 'missing')
    s = '';
elseif isnumeric(v)
    s = num2str(v);
else
    s = char(string(v));
end
end

function s = joinList(x, sep)
% jsondecode gives [] for an empty list, a char for one string, a cell for several.
if isempty(x)
    s = '';
else
    s = strjoin(cellstr(x), sep);
end
end

function s = shorten(text)
% Summary is a label. Quotes are dropped: systemcomposer.sysml.exportFromMLProject
% (R2026a) uses it as a quoted SysML name without escaping them.
s = strrep(strrep(text, '''', ''), '"', '');
if strlength(s) > 90
    s = [extractBefore(s, 88) '...'];
end
end
