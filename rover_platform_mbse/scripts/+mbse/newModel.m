function file = newModel(name, stepSize)
%NEWMODEL Create an empty fixed-step discrete Simulink model in behaviour/,
%   deleting an older copy. Outputs are logged to a Dataset (out.yout).

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

file = fullfile(mbse.root(), 'behaviour', [name '.slx']);
if bdIsLoaded(name)
    close_system(name, 0);
end
if isfile(file)
    delete(file);
end
new_system(name);
set_param(name, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(stepSize), 'StopTime', '10', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on', 'LoadExternalInput', 'on', ...
    'SaveTime', 'on', 'SaveState', 'off');
end
