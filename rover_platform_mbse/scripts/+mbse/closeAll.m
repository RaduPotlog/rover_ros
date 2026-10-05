function closeAll()
%CLOSEALL Close every model, dictionary, profile, allocation set and requirement
%   set without saving. A "save changes?" dialog would block the MCP session.

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

bdclose('all');
Simulink.data.dictionary.closeAll('-discard');
systemcomposer.allocation.AllocationSet.closeAll();
systemcomposer.profile.Profile.closeAll();
slreq.clear();
end
