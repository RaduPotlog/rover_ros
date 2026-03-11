# This message defines the battery charging process status

# Charger types
uint8 UNKNOWN = 0
uint8 WIRED = 1
uint8 WIRELESS = 2

std_msgs/Header header
bool charging               # True if battery is being charged
float32 current             # Power supply total current (A)
float32 current_battery     # Power supply current (A) for battery 1
uint8 charger_type          # Determines the type of charger connection
