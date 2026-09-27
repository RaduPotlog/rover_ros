function s = readJson(relPath)
%READJSON Decode a JSON file given relative to the project root.
s = jsondecode(fileread(fullfile(mbse.root(), relPath)));
end
