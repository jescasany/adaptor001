define kinect ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  #color "black"
  color_rgba [ 0 0 1 0.3 ]
  size [ 0.06 0.15 0.03 ]
  pose [0.0 0.0 0.5 90]
)

# this bumper array VERY crudely approximates the Roomba's bumpers
#  bumper( bcount 2  
#	  blength 0.33
#        bpose[0] [0.12  0.12  45]
#       bpose[1] [0.12 -0.12 -45] 
#        )

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
)
