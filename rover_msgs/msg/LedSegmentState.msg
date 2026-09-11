# This message defines the state of every priority layer of one LED segment

string name                 # Segment name, as in the animations configuration
uint16 channel              # LED panel channel the segment belongs to
LedLayerState[] layers      # One entry per priority layer, ordered ERROR to STATE
