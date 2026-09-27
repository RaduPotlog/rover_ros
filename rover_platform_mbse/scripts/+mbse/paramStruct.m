function P = paramStruct(names)
%PARAMSTRUCT Struct of the named platform parameters (values in the JSON units).
P = struct();
for k = 1:numel(names)
    P.(names{k}) = mbse.param(names{k});
end
end
