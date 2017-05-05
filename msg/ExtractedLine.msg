# The m and c coefficients define the extracted line: y = tan(m)*x + c.  
#
float32 m
float32 c
#
# The following points are used for display / debugging purposes only.
# These points come from the position of the first and last laser scan points
# used to estimate the line.  They do not define the line and may not have even
# been used in estimating the line (e.g. they may have exceeded the
# maximum_range parameter).  However, they are useful in visualizing what range
# of scan points were used to estimate the line.
geometry_msgs/Point firstScanPoint
geometry_msgs/Point lastScanPoint
