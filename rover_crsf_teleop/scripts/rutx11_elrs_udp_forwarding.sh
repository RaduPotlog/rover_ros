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

# Shows or configures the Teltonika RUTX11 side of the ELRS receiver -> UDP path used by
# rover_crsf_teleop.
#
#   ./rutx11_elrs_udp_forwarding.sh show    # read-only: USB serial device, serial + firewall config
#   ./rutx11_elrs_udp_forwarding.sh apply   # firewall: drop AP clients' datagrams to the RC port
#
# The serial forwarding itself is NOT applied by this script: Teltonika does not document the uci
# schema behind Services -> Serial Utilities, and guessing option names on the router that carries
# the rover's RC link is worse than a WebUI step. Configure it once in the WebUI (below), then run
# `show` and fold the uci it prints into this script's APPLY_CMD.
#
#   Services -> Serial Utilities -> add, type "Over IP":
#     Device: USB RS232 interface      Baud rate: 460800   Data bits: 8   Parity: None
#     Stop bits: 1                     Flow control: None
#     Protocol: UDP    Mode: Client    Destination address: ${ROVER_HOST} : ${RC_PORT}
#     Advanced: Raw mode on, Serial timeout 1-2 ms (lowest that still fills datagrams sensibly),
#               Inactivity timeout 0
#
# Before that, check `show` lists a /dev/ttyUSB* for the adapter. If it does not, RutOS has no
# driver for the adapter's chip; FTDI or CP210x adapters are the safest choice.
#
# Why the firewall rule: CRSF is unauthenticated and the RC input is twist_mux's highest-priority
# velocity source. rover_crsf_udp_receiver already drops every datagram whose source is not the
# router (its source_ip parameter), but the AP network (WWAN, 192.168.77.0/24) shares the router's
# `lan` zone, so its clients are routed to the rover unhindered; this rule drops them at the
# router as a second layer. WireGuard peers are already rejected by `VPN-reject-everything-else`
# (see rover_docker/README.md). Neither layer stops a host that forges 192.168.1.1 as its source
# address (the rule matches on source address too); only a separate network segment for the
# rover computer and the router would close that.
#
# The password is never stored: export RUTX11_PASSWORD for a non-interactive run (it is passed
# to ssh through a temporary SSH_ASKPASS helper, not on the command line), or leave it unset to
# type it at the ssh prompt. Written against RutOS 7.25.2.
#
# Environment (defaults):
#   RUTX11_HOST=192.168.1.1  RUTX11_USER=root  ROVER_HOST=192.168.1.201
#   RC_PORT=10111            AP_SUBNET=192.168.77.0/24

set -euo pipefail

MODE="${1:-show}"
RUTX11_HOST="${RUTX11_HOST:-10.8.0.5}"
RUTX11_USER="${RUTX11_USER:-root}"
ROVER_HOST="${ROVER_HOST:-192.168.1.201}"
RC_PORT="${RC_PORT:-10111}"
AP_SUBNET="${AP_SUBNET:-192.168.77.0/24}"

# Named uci section, so `apply` is idempotent.
RULE="elrs_rc_udp_from_ap"

usage() {
  echo "Usage: $0 show|apply" >&2
  exit 2
}

[[ "$MODE" == "show" || "$MODE" == "apply" ]] || usage
[[ "$RC_PORT" =~ ^[0-9]+$ ]] || {
  echo "RC_PORT must be an integer." >&2
  exit 2
}
[[ "$ROVER_HOST" =~ ^[A-Za-z0-9.:-]+$ && "$AP_SUBNET" =~ ^[0-9./]+$ ]] || {
  echo "ROVER_HOST or AP_SUBNET contains invalid characters." >&2
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

SHOW_CMD="
cat /etc/version 2>/dev/null
echo '--- USB serial devices ---'
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo 'none: no driver for the adapter, or nothing plugged in'
dmesg | grep -iE 'usb|tty|ftdi|cp210|ch34|pl2303' | tail -n 15
echo '--- serial-related uci configs ---'
for config in \$(ls /etc/config); do
  case \"\$config\" in
    *serial*|*rs232*|*rs485*|*overip*|*ser2net*) uci show \"\$config\" ;;
  esac
done
echo '--- RC firewall rule ---'
uci -q show firewall.${RULE} || echo 'firewall.${RULE} not set'
"

APPLY_CMD="
set -e
uci set firewall.${RULE}=rule
uci set firewall.${RULE}.name='ELRS-RC-UDP-drop-from-AP'
uci set firewall.${RULE}.src='lan'
uci set firewall.${RULE}.src_ip='${AP_SUBNET}'
uci set firewall.${RULE}.dest='lan'
uci set firewall.${RULE}.dest_ip='${ROVER_HOST}'
uci set firewall.${RULE}.dest_port='${RC_PORT}'
uci set firewall.${RULE}.proto='udp'
uci set firewall.${RULE}.target='DROP'
uci commit firewall
/etc/init.d/firewall reload
echo 'Applied.'
"

if [[ "$MODE" == "apply" ]]; then
  echo "Configuring ${RUTX11_HOST}: drop UDP ${AP_SUBNET} -> ${ROVER_HOST}:${RC_PORT}"
  run_remote "$APPLY_CMD"
fi

run_remote "$SHOW_CMD"
