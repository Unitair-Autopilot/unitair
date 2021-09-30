# State of a gamepad-style "joystick" (like an XBox controller).
##################################################

time stamp  # [s]
bool error  # whether or not there is a problem with the connection

# Right-hand analog inputs
float64 right_stick_north  # -1 to 1
float64 right_stick_east   # -1 to 1
float64 right_trigger      #  0 to 1

# Left-hand analog inputs
float64 left_stick_north  # -1 to 1
float64 left_stick_east   # -1 to 1
float64 left_trigger      #  0 to 1

# "RB" and "LB"
bool right_bumper
bool left_bumper

# Direction-pad
bool dpad_north
bool dpad_south
bool dpad_east
bool dpad_west

# "Y", "A", "B", and "X"
bool button_north
bool button_south
bool button_east
bool button_west

# "Start" and "Select" as **toggles**
bool switch_east
bool switch_west
