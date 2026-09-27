function sfData(ch, names, scope)
%SFDATA Add double-typed chart data. scope: 'Input', 'Output' or 'Parameter'.
for k = 1:numel(names)
    d = Stateflow.Data(ch);
    d.Name = names{k};
    d.Scope = scope;
    if ~strcmp(scope, 'Parameter')
        d.DataType = 'double';
    end
end
end
