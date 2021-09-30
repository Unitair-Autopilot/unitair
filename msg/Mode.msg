# Logic state of the Manager.
##################################################

# Enumeration for possible modes
uint32 UNDEFINED=0  # indicates flawed logic
uint32 SETUP=1      # waiting for subsystems to get set-up
uint32 STANDBY=2    # waiting to begin a mission (no output)
uint32 TRACKING=3   # executing a path-tracking mission

# Time when mode was determined
time stamp  # [s]

# Mode value and verbose description
uint32 mode
string details

# Whether or not a human pilot is overriding commands
bool autonomous
