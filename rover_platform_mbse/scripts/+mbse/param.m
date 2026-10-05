function [value, unit, source] = param(name)
%PARAM Value of a named parameter from data/platform_parameters.json.
%   All model values come from this file, which cites the rover_ros source line.

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

persistent table
if isempty(table)
    p = mbse.readJson(fullfile('data', 'platform_parameters.json'));
    table = containers.Map();
    for k = 1:numel(p.parameters)
        e = p.parameters(k);
        table(e.name) = e;
    end
end
if ~isKey(table, name)
    error('mbse:param', 'Unknown parameter "%s".', name);
end
e = table(name);
value = e.value;
unit = e.unit;
source = e.source;
end
