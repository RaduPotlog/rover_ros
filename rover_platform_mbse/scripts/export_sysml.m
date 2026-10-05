function export_sysml()
%EXPORT_SYSML Export the MATLAB project to SysML v2 text (R2026a+):
%   export/rover_platform_export.sysml. The export is lossy (units become Real, no
%   requirements or behaviour) and is for review and diffs only. Validate it with
%   /sysml-validate from WSL.

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

setup_project();
out = fullfile(mbse.root(), 'export', 'rover_platform_export.sysml');
if isfile(out)
    delete(out);
end
systemcomposer.sysml.exportFromMLProject(fullfile(mbse.root(), 'RoverPlatform.prj'), out);
mbse.closeAll();
fprintf('export_sysml: %s\n', out);
end
