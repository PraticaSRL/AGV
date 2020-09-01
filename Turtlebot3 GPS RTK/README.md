The Turtlebot3-based prototype has been equipped with a Real Time Kinematics GPS System produced by Ublox. The differential drive vehicle will receive the location data from USB and use them to navigate in the outdoor environment. Once the RTK GPS system is started and the survey is complete (that is the system is working in maximum precision), the ublox_gps package is used to read data from USB (coming from RTK ROVER) and launched from the rover.launch file, which publish the coordinates of the rover in the NED format in the /ROVER/navrelposned topic. Then the gps_ublox node, implemented through the gps_ublox.py script, will read the localization data from the topic /ROVER/navrelposned (in the NED format), convert them in the PoseWithCovarianceStamped format and publish them in the topic /pose_gps. The robot_localization package will allow the fusion of multiple sources of localization data by mean of Extended Kalman Filters, through which localization data coming from odometry, IMU and GPS RTK system are used to produce an optimal position estimation.

agv.launch provides:
1) initialize and start the navigation system;
2) start the nodes for reading GPS RTK data from usb and convert them in the PoseWithCovarianceStamped format;
3) start the rviz visualization tool and the necessary markers