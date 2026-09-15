#!/bin/bash

# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Shows or configures the Teltonika RUTX11 GNSS NMEA forwarding used by rover_gps.
#
#   ./rutx11_gps_nmea_forwarding.sh show    # read-only: GPS uci config + current position
#   ./rutx11_gps_nmea_forwarding.sh apply   # UDP forwarding of GGA/RMC/VTG to the rover
#
# The password is never stored: export RUTX11_PASSWORD for a non-interactive run (it is passed
# to ssh through a temporary SSH_ASKPASS helper, not on the command line), or leave it unset to
# type it at the ssh prompt. Checked against RutOS RUTX_R_00.07.02.
#
# Environment (defaults):
#   RUTX11_HOST=192.168.1.1  RUTX11_USER=root  ROVER_HOST=192.168.1.201
#   NMEA_PORT=10110          NMEA_INTERVAL_S=1

set -euo pipefail

MODE="${1:-show}"
RUTX11_HOST="${RUTX11_HOST:-192.168.1.1}"
RUTX11_USER="${RUTX11_USER:-root}"
ROVER_HOST="${ROVER_HOST:-192.168.1.201}"
NMEA_PORT="${NMEA_PORT:-10110}"
NMEA_INTERVAL_S="${NMEA_INTERVAL_S:-1}"

# Sentences nmea_navsat_driver uses: GGA (position, fix quality, HDOP), RMC and VTG (velocity).
NMEA_SENTENCES="GPGGA GPRMC GPVTG"

usage() {
  echo "Usage: $0 show|apply" >&2
  exit 2
}

[[ "$MODE" == "show" || "$MODE" == "apply" ]] || usage
[[ "$NMEA_PORT" =~ ^[0-9]+$ && "$NMEA_INTERVAL_S" =~ ^[0-9]+$ ]] || {
  echo "NMEA_PORT and NMEA_INTERVAL_S must be integers." >&2
  exit 2
}
[[ "$ROVER_HOST" =~ ^[A-Za-z0-9.:-]+$ ]] || {
  echo "ROVER_HOST contains invalid characters." >&2
  exit 2
}

run_remote() {
  local ssh_opts=(-o ConnectTimeout=10 -o PubkeyAuthentication=no)

  if [[ -n "${RUTX11_PASSWORD:-}" ]]; then
    local askpass
    askpass="$(mktemp)"
    trap 'rm -f "${askpass:-}"; trap - RETURN' RETURN
    chmod 700 "$askpass"
    printf '#!/bin/sh\nprintf "%%s\\n" "$RUTX11_PASSWORD"\n' > "$askpass"
    SSH_ASKPASS="$askpass" SSH_ASKPASS_REQUIRE=force DISPLAY="${DISPLAY:-none}" \
      RUTX11_PASSWORD="$RUTX11_PASSWORD" \
      ssh "${ssh_opts[@]}" -o NumberOfPasswordPrompts=1 "${RUTX11_USER}@${RUTX11_HOST}" "$1"
  else
    ssh "${ssh_opts[@]}" "${RUTX11_USER}@${RUTX11_HOST}" "$1"
  fi
}

SHOW_CMD='
cat /etc/version 2>/dev/null
echo "--- gps uci (gpsd, forwarding, forwarded sentences) ---"
uci show gps | grep -E "^gps\.(gpsd|nmea_forwarding)\.|^gps\.(GPGGA|GPRMC|GPVTG)\."
echo "--- current position ---"
ubus call gpsd position 2>/dev/null || echo "gpsd position unavailable"
'

APPLY_CMD="
set -e
uci -q get gps.gpsd >/dev/null || { echo 'gps.gpsd section missing (unsupported RutOS version?)' >&2; exit 1; }
uci -q get gps.nmea_forwarding >/dev/null || { echo 'gps.nmea_forwarding section missing' >&2; exit 1; }
uci set gps.gpsd.enabled='1'
uci set gps.nmea_forwarding.enabled='1'
uci set gps.nmea_forwarding.proto='udp'
uci set gps.nmea_forwarding.hostname='${ROVER_HOST}'
uci set gps.nmea_forwarding.port='${NMEA_PORT}'
for sentence in ${NMEA_SENTENCES}; do
  uci -q get gps.\$sentence >/dev/null || { echo \"gps.\$sentence rule missing\" >&2; exit 1; }
  uci set gps.\$sentence.forwarding_enabled='1'
  uci set gps.\$sentence.forwarding_interval='${NMEA_INTERVAL_S}'
done
uci commit gps
/etc/init.d/gpsd restart
echo 'Applied.'
"

if [[ "$MODE" == "apply" ]]; then
  echo "Configuring ${RUTX11_HOST}: UDP NMEA (${NMEA_SENTENCES}) every ${NMEA_INTERVAL_S}s -> ${ROVER_HOST}:${NMEA_PORT}"
  run_remote "$APPLY_CMD"
fi

run_remote "$SHOW_CMD"
