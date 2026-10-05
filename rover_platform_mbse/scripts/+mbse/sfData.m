function sfData(ch, names, scope)
%SFDATA Add double-typed chart data. scope: 'Input', 'Output' or 'Parameter'.

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

for k = 1:numel(names)
    d = Stateflow.Data(ch);
    d.Name = names{k};
    d.Scope = scope;
    if ~strcmp(scope, 'Parameter')
        d.DataType = 'double';
    end
end
end
