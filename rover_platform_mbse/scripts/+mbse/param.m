function [value, unit, source] = param(name)
%PARAM Value of a named parameter from data/platform_parameters.json.
%   All model values come from this file, which cites the rover_ros source line.
persistent table
if isempty(table)
    p = mbse.readJson(fullfile('data', 'platform_parameters.json'));
    table = containers.Map();
    for k = 1:numel(p.parameters)
        e = p.parameters(k);
        table(e.name) = e;
    end
end
if ~isKey(table, name)
    error('mbse:param', 'Unknown parameter "%s".', name);
end
e = table(name);
value = e.value;
unit = e.unit;
source = e.source;
end
