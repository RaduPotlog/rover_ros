function t = sfTrans(parent, src, dst, label, order, srcClock, dstClock)
%SFTRANS Add a transition src -> dst with a guard label and execution order.
%   srcClock/dstClock: attachment points (0-12, o'clock) to keep arrows readable.

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
t.Source = src;
t.Destination = dst;
if nargin > 5
    t.SourceOClock = srcClock;
    t.DestinationOClock = dstClock;
end
t.LabelString = label;
t.ExecutionOrder = order;
end
