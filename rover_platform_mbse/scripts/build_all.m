function build_all(opts)
%BUILD_ALL Rebuild the whole rover_platform_mbse project, in order:
%   project -> architecture -> behaviour models -> requirements -> trace links ->
%   model tests -> traceability check + exports -> SysML v2 export.
%   build_all('RebuildRequirements', true) also recreates the SWRS .slreqx sets from
%   data/swrs/*.json, discarding edits made in the Requirements Editor.
%   Afterwards, from WSL: uv run --no-project --with openpyxl python scripts/format_exports.py

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

arguments
    opts.RebuildRequirements (1, 1) logical = false
end
setup_project();
mbse.closeAll();
build_architecture();
build_command_arbitration();
build_estop_latch();
build_mode_manager();
build_skid_steer();
build_requirements('Rebuild', opts.RebuildRequirements);
build_trace_links();
run_model_tests();
export_requirements();
export_sysml();
mbse.closeAll();
fprintf('build_all: done\n');
end
