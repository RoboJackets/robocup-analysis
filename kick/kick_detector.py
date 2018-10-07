# Take last 3 ball applied to each kalman filter
# Check direction change
#       speed change
#       distance to closest robot
# If so, get kicking robot
# Get list of balls since kick time
# Create kick event
# check velocity jump between 1->2 and 2->3
# Trigger on angle + vel jump
# Get list of all merged balls we have updated since kick time
# Kick time is middle ball time

# Get kicking bot based on direction facing, min ball vel, infront, distance (was near, now away), increasing distance

# Collect all new balls along that are detected for each of these filters
# Try to fit various things to it