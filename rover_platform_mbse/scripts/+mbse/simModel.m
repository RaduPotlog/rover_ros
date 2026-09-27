function y = simModel(model, t, U, overrides)
%SIMMODEL Simulate a behaviour model with the input matrix U (one column per root
%   Inport, in port order) sampled at times t. overrides: struct of model-workspace
%   variables to replace for this run (e.g. a modified P). Returns a struct with one
%   field per logged output (column vector, same length as the sim time vector) and
%   the time vector in y.t.
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
