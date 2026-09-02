#!/usr/bin/env bash
# Fails if any file under include/rover_hardware_interface/domain/ or src/domain/ includes a
# ROS/hardware_interface/vendor-SDK header. Domain code (pure value types, ports, and business
# rules) must have zero such dependencies - see .claude/rules/clean_architecture.md. Infrastructure
# adapters (rover_driver/phidget_driver/, rover_sensors/, rover_system/, ...) are exempt: they are
# expected to depend on these.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <package_source_root>" >&2
    exit 2
fi

PKG_ROOT="$1"
DOMAIN_INCLUDE_DIR="${PKG_ROOT}/include/rover_hardware_interface/domain"
DOMAIN_SRC_DIR="${PKG_ROOT}/src/domain"

# Matches an #include of any forbidden header, whether angle-bracketed or quoted.
FORBIDDEN_PATTERN='#[[:space:]]*include[[:space:]]*[<"](rclcpp|rclcpp_lifecycle|hardware_interface|diagnostic_updater|realtime_tools|libphidget22|phidget22|phidgets_api|[A-Za-z0-9_]+_msgs/msg/)'

violations=$(grep -rEn "$FORBIDDEN_PATTERN" "$DOMAIN_INCLUDE_DIR" "$DOMAIN_SRC_DIR" 2>/dev/null || true)

if [[ -n "$violations" ]]; then
    echo "Domain-layer purity check FAILED - forbidden includes found under domain/:"
    echo "$violations"
    echo
    echo "Domain code must not depend on ROS, hardware_interface, diagnostic_updater," \
         "realtime_tools, ROS message packages, or the Phidget vendor SDK. Move this file" \
         "out of domain/ if it genuinely needs one of these, or remove the dependency."
    exit 1
fi

echo "Domain-layer purity check passed."
exit 0
