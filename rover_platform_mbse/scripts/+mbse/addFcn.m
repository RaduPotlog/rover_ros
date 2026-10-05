function blk = addFcn(path, code, paramNames, position)
%ADDFCN Add a MATLAB Function block with the given code. Arguments listed in
%   paramNames become non-tunable Parameter data resolved from the model
%   workspace (the P struct built from data/platform_parameters.json).

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

add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', position);
blk = path;
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', path);
chart.Script = code;
for k = 1:numel(paramNames)
    d = chart.find('-isa', 'Stateflow.Data', 'Name', paramNames{k});
    d.Scope = 'Parameter';
    d.Tunable = false;
end
end
