function blk = addFcn(path, code, paramNames, position)
%ADDFCN Add a MATLAB Function block with the given code. Arguments listed in
%   paramNames become non-tunable Parameter data resolved from the model
%   workspace (the P struct built from data/platform_parameters.json).
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
