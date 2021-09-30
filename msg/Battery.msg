# State of the battery power-source.
##################################################

time stamp  # [s]

float64 voltage  # raw value, [V]
int32   percent  # assumed remaining life as `int(100*(voltage-min)/(max-min))`
