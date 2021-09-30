# Any node can publish Health messages to the topic "health".
# The Manager subscribes to "health", aggregates the messages into a dictionary, and acts accordingly / publishes insight.
# Complete node failure (e.g. segfault) is handled by the ROS Launch system ("required" vs "respawn", etc).
##################################################

# Enumeration for request types:
uint32 PASS=0
uint32 WARN=1
uint32 WAIT=2
uint32 FAIL=3

# Who 'dis and when
string node_name
time stamp  # [s]

# What to do (request), regarding what (label)
uint32[] requests
string[] labels  # must have the same size as `requests`
