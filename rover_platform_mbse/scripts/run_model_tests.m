function results = run_model_tests()
%RUN_MODEL_TESTS Run tests/t*.m and write results/model_test_results.json.
%   Tests tagged KnownDeviation are expected to fail: they measure a requirement the
%   as-built design misses. The run errors only when an untagged test fails or a
%   KnownDeviation test starts passing (then the deviation is fixed: remove the tag
%   and update the compliance data).

root = mbse.root();
r = runtests(fullfile(root, 'tests'));
tags = mbse.testVerifies();
known = containers.Map({tags.name}, num2cell([tags.knownDeviation]));
results = struct('name', {}, 'passed', {}, 'knownDeviation', {}, 'message', {});
for k = 1:numel(r)
    msg = '';
    if r(k).Failed
        d = r(k).Details.DiagnosticRecord;
        if ~isempty(d)
            msg = firstLine(d(1).TestDiagnosticResults, d(1).Report);
        end
    end
    kd = isKey(known, r(k).Name) && known(r(k).Name);
    results(end + 1) = struct('name', r(k).Name, 'passed', r(k).Passed, ...
        'knownDeviation', kd, 'message', msg); %#ok<AGROW>
end

outDir = fullfile(root, 'results');
if ~isfolder(outDir)
    mkdir(outDir);
end
out = struct('date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')), ...
    'matlab', version, 'results', results);
fid = fopen(fullfile(outDir, 'model_test_results.json'), 'w');
fprintf(fid, '%s', jsonencode(out, PrettyPrint = true));
fclose(fid);

passed = [results.passed];
kd = [results.knownDeviation];
fprintf('run_model_tests: %d passed, %d known deviations confirmed, %d unexpected failures, %d deviations now passing\n', ...
    sum(passed & ~kd), sum(~passed & kd), sum(~passed & ~kd), sum(passed & kd));
bad = {results((~passed & ~kd) | (passed & kd)).name};
if ~isempty(bad)
    error('run_model_tests:unexpected', 'Unexpected results: %s', strjoin(bad, ', '));
end
end

function s = firstLine(testDiag, report)
% Prefer the test's own diagnostic text (e.g. "worst-case ... = 240 ms").
s = '';
if ~isempty(testDiag)
    s = strtrim(char(testDiag(1).DiagnosticText));
end
if isempty(s)
    lines = strtrim(splitlines(string(report)));
    lines = lines(lines ~= "" & ~startsWith(lines, ["=", "-"]));
    s = char(strjoin(lines(1:min(end, 6)), ' '));
end
end
