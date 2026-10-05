function t = sfDefault(parent, dst)
%SFDEFAULT Default transition into dst, drawn from above its top edge.

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

t = Stateflow.Transition(parent);
t.Destination = dst;
t.DestinationOClock = 0;
p = dst.Position;
x = p(1) + p(3) / 2;
t.SourceEndpoint = [x, p(2) - 25];
t.MidPoint = [x, p(2) - 12];
end
