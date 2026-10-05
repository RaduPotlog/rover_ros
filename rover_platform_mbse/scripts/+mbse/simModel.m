function y = simModel(model, t, U, overrides)
%SIMMODEL Simulate a behaviour model with the input matrix U (one column per root
%   Inport, in port order) sampled at times t. overrides: struct of model-workspace
%   variables to replace for this run (e.g. a modified P). Returns a struct with one
%   field per logged output (column vector, same length as the sim time vector) and
%   the time vector in y.t.

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

if nargin < 4
    overrides = struct();
end
in = Simulink.SimulationInput(model);
in = in.setExternalInput([t(:) U]);
in = in.setModelParameter('StopTime', num2str(t(end)));
f = fieldnames(overrides);
for k = 1:numel(f)
    in = in.setVariable(f{k}, overrides.(f{k}), 'Workspace', model);
end
out = sim(in);
ds = out.yout;
names = ds.getElementNames();
y = struct();
for k = 1:numel(names)
    v = ds.get(names{k}).Values;
    y.(names{k}) = double(v.Data(:));
    y.t = v.Time(:);
end
end
