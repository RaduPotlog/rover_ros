function proj = setup_project()
%SETUP_PROJECT Open (or create) the RoverPlatform MATLAB project.
%   The project puts scripts/, architecture/, behaviour/, requirements/ and tests/
%   on the path, so the profile, the dictionary and the requirement sets resolve
%   no matter which folder MATLAB starts in. Idempotent.

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

root = mbse.root();
prjFile = fullfile(root, 'RoverPlatform.prj');
try
    proj = currentProject();
    if strcmpi(proj.RootFolder, root)
        return
    end
    close(proj);
catch
    % no project open
end
if isfile(prjFile)
    proj = openProject(prjFile);
else
    proj = matlab.project.createProject(Folder = root, Name = 'RoverPlatform');
end

folders = {'scripts', 'architecture', 'behaviour', 'requirements', 'tests', 'data', 'system', 'export'};
for k = 1:numel(folders)
    f = fullfile(root, folders{k});
    if ~isfolder(f)
        mkdir(f);
    end
    if isempty(proj.findFile(f))
        proj.addFolderIncludingChildFiles(f);
    end
end
onPath = {proj.ProjectPath.File};
for k = 1:5
    f = fullfile(root, folders{k});
    if ~any(strcmp(onPath, f))
        proj.addPath(f);
    end
end
end
