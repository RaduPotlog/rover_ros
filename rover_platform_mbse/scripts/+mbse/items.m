function c = items(x)
%ITEMS Return a jsondecode array as a cell array of structs, whether jsondecode
%   produced a struct array (uniform fields) or a cell array (mixed fields).

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

if iscell(x)
    c = x(:)';
else
    c = num2cell(x(:)');
end
end
