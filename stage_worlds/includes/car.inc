
# Desc: Device definitions for generic robotic car
# Author: Jack O'Quin
# $Id: car.inc 9053 2009-11-17 15:16:52Z joq $

#float32 length = 3.25                    # overall length
#float32 width = 1.397                    # overall width
#float32 height = 1.778                    # overall height (TBD)
#float32 halflength = 1.625                # length / 2
#float32 halfwidth = 0.6985                # width / 2
#float32 halfheight = 0.889               # height / 2
#float32 wheelbase = 2.59     	# wheelbase
# a generic car, similar to ART's Marvin
define car position
(
  # actual size in meters
  # (need height so it shows up on laser scans)
  size [3.25 1.397 1.778 ]
  # only used if drive is set to "car"
  wheelbase 2.59
    # [ xmin xmax ymin ymax zmin zmax amin amax ]				
    velocity_bounds [-6.7056 6.7056 -0 0 -0 0 -34 34 ]					
    acceleration_bounds [-1 1 -1 1 -1 1 -90 90]

  # the car rotates about the rear axle, behind the center of area
  # it seems front of the car is neg - y-offset should be pos
  origin [1.3 0.0 0 0]

  # car-like steering
  drive "car"

  # draw a nose on the robot so we can see which way it points
  gui_nose 1
  gui_outline 1

  # estimated mass in KG
  mass 700.0 

  # visible to lasers
  laser_return 1

  block
  (
    points 8
    point[0] [  0.20  0.05 ]
    point[1] [  0.15  0.15 ]
    point[2] [ -0.15  0.15 ]
    point[3] [ -0.20  0.05 ]
    point[4] [ -0.20 -0.05 ]
    point[5] [ -0.15 -0.15 ]
    point[6] [  0.15 -0.15 ]
    point[7] [  0.20 -0.05 ]
    z [ 0 2 ]
  )
) 
