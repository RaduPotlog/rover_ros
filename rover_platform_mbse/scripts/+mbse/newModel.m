function file = newModel(name, stepSize)
%NEWMODEL Create an empty fixed-step discrete Simulink model in behaviour/,
%   deleting an older copy. Outputs are logged to a Dataset (out.yout).
file = fullfile(mbse.root(), 'behaviour', [name '.slx']);
if bdIsLoaded(name)
    close_system(name, 0);
end
if isfile(file)
    delete(file);
end
new_system(name);
set_param(name, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(stepSize), 'StopTime', '10', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on', 'LoadExternalInput', 'on', ...
    'SaveTime', 'on', 'SaveState', 'off');
end
