function n = sanitize(s)
%SANITIZE Turn a topic or port label into a valid System Composer name.
n = regexprep(char(s), '[^A-Za-z0-9_]', '_');
n = regexprep(n, '_+', '_');
n = regexprep(n, '^_|_$', '');
end
