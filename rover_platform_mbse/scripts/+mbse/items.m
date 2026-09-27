function c = items(x)
%ITEMS Return a jsondecode array as a cell array of structs, whether jsondecode
%   produced a struct array (uniform fields) or a cell array (mixed fields).
if iscell(x)
    c = x(:)';
else
    c = num2cell(x(:)');
end
end
