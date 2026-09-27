function proj = setup_project()
%SETUP_PROJECT Open (or create) the RoverPlatform MATLAB project.
%   The project puts scripts/, architecture/, behaviour/, requirements/ and tests/
%   on the path, so the profile, the dictionary and the requirement sets resolve
%   no matter which folder MATLAB starts in. Idempotent.

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
