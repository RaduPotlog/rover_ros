function export_sysml()
%EXPORT_SYSML Export the MATLAB project to SysML v2 text (R2026a+):
%   export/rover_platform_export.sysml. The export is lossy (units become Real, no
%   requirements or behaviour) and is for review and diffs only. Validate it with
%   /sysml-validate from WSL.
setup_project();
out = fullfile(mbse.root(), 'export', 'rover_platform_export.sysml');
if isfile(out)
    delete(out);
end
systemcomposer.sysml.exportFromMLProject(fullfile(mbse.root(), 'RoverPlatform.prj'), out);
mbse.closeAll();
fprintf('export_sysml: %s\n', out);
end
