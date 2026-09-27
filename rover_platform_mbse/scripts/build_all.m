function build_all(opts)
%BUILD_ALL Rebuild the whole rover_platform_mbse project, in order:
%   project -> architecture -> behaviour models -> requirements -> trace links ->
%   model tests -> traceability check + exports -> SysML v2 export.
%   build_all('RebuildRequirements', true) also recreates the SWRS .slreqx sets from
%   data/swrs/*.json, discarding edits made in the Requirements Editor.
%   Afterwards, from WSL: uv run --no-project --with openpyxl python scripts/format_exports.py
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
