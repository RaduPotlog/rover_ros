# CRSF LINK_STATISTICS frame (type 0x14) as reported by the receiver.
std_msgs/Header header

# Uplink = transmitter -> receiver: the direction that carries the sticks. This is the half that
# gates teleop, via uplink_link_quality.
uint8 uplink_rssi_ant1        # dBm * -1
uint8 uplink_rssi_ant2        # dBm * -1
uint8 uplink_link_quality     # packet success rate / link quality, 0-100 %
int8  uplink_snr              # dB, signed
uint8 active_antenna          # diversity: 0 = antenna 1, 1 = antenna 2
uint8 rf_mode                 # 0 = 4 Hz, 1 = 50 Hz, 2 = 150 Hz
uint8 uplink_tx_power         # 0=0mW 1=10mW 2=25mW 3=100mW 4=500mW 5=1000mW 6=2000mW

# Downlink = receiver -> transmitter: telemetry only, does not gate teleop.
uint8 downlink_rssi           # dBm * -1
uint8 downlink_link_quality   # packet success rate / link quality, 0-100 %
int8  downlink_snr            # dB, signed
