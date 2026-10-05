function v = testVerifies()
%TESTVERIFIES Parse the "% Verifies:" tags of the model tests.
%   Returns a struct array: file, name ('tClass/testMethod'), fnLine, tagLine, ids,
%   knownDeviation (true when the method sits in a TestTags = {'KnownDeviation'} block).

% Copyright 2026 Mechatronics Academy
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

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
