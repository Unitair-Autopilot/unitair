# A simple "catch-all" message for associated name:value data.
# This should only be used for non-critical information with poorly defined structure.
##################################################

time stamp  # [s]

string[] names
float64[] values  # must have the same size as `names`
