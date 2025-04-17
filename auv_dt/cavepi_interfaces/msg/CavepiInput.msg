# This represents a control message to the CavePI AUV/ROV.

Header header
float64 surge # -1.0 to 1.0, full thrust reverse to 0 to full thrust forward.
float64 heave # -1.0 to 1.0, full thrust down to 0 to full thrust up.
float64 roll  # -1.0 to 1.0, full thrust roll left to 0 to full thrust roll right.
float64 yaw   # -1.0 to 1.0, full thrust yaw left to 0 to full thrust yaw right.