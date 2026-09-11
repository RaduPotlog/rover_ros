# This message defines the animation playing on one priority layer of an LED segment

# Priority layers (lower value is drawn on top)
uint8 ERROR = 0
uint8 ALERT = 1
uint8 INFO = 2
uint8 STATE = 3

uint8 priority              # Priority layer this entry describes
bool active                 # True if an animation is currently playing on the layer
uint16 id                   # Animation ID (valid only if active)
string name                 # Animation name (valid only if active)
string param                # Parameter the animation was started with
bool repeating              # True if the animation restarts when it ends
float32 progress            # Progress of the animation in range [0.0, 1.0]
uint8 queued                # Animations waiting behind this one (ALERT layer only)
