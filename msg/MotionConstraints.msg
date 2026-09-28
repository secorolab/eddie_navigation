# How the base has to move through a zone; a new constraint is a new field, handled in the server.

uint8 HEADING_ANY = 0      # keep the heading the base had when the pass started
uint8 HEADING_FORWARD = 1  # face the way through
uint8 heading_mode

float64 max_speed            # [m/s] through the zone
float64 align_tolerance_xy   # [m] lateral error before the pass starts
float64 align_tolerance_yaw  # [rad] heading error before the pass starts
float64 stop_distance        # [m] look-ahead of the footprint check; a lethal cell stops the base
