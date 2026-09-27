function v = testVerifies()
%TESTVERIFIES Parse the "% Verifies:" tags of the model tests.
%   Returns a struct array: file, name ('tClass/testMethod'), fnLine, tagLine, ids,
%   knownDeviation (true when the method sits in a TestTags = {'KnownDeviation'} block).
v = struct('file', {}, 'name', {}, 'fnLine', {}, 'tagLine', {}, 'ids', {}, 'knownDeviation', {});
tests = dir(fullfile(mbse.root(), 'tests', 't*.m'));
for k = 1:numel(tests)
    file = fullfile(tests(k).folder, tests(k).name);
    [~, cls] = fileparts(file);
    lines = splitlines(fileread(file));
    known = false;
    fnLine = 0;
    fnName = '';
    for i = 1:numel(lines)
        if ~isempty(regexp(lines{i}, '^\s*methods\s*\(', 'once'))
            known = contains(lines{i}, 'KnownDeviation');
        end
        fn = regexp(lines{i}, '^\s*function\s+(test\w+)', 'tokens', 'once');
        if ~isempty(fn)
            fnLine = i;
            fnName = fn{1};
        end
        tok = regexp(lines{i}, '^\s*%\s*Verifies:\s*(.*)$', 'tokens', 'once');
        if ~isempty(tok)
            v(end + 1) = struct('file', file, 'name', [cls '/' fnName], 'fnLine', fnLine, ...
                'tagLine', i, 'ids', {strtrim(strsplit(tok{1}, ','))}, 'knownDeviation', known); %#ok<AGROW>
        end
    end
end
end
