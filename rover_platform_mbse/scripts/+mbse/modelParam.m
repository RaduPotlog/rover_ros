function v = modelParam(model, name)
%MODELPARAM Read a variable from a behaviour model's workspace.
load_system(model);
v = getVariable(get_param(model, 'ModelWorkspace'), name);
end
