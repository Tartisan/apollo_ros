  #                x axis  ^
  #                        | longitude_dist
  #                        |
  #                        |
  #                        |
  #          lateral_dist  |
  #          y axis        |
  #        <----------------
  #        ooooooooooooo   //radar front surface
std_msgs/Header header
bool clusterortrack  # 0 = track, 1 = cluster
int32 obstacle_id    # obstacle Id
# longitude distance to the radar; (+) = forward; unit = m
float64 longitude_dist
# lateral distance to the radar; (+) = left; unit = m
float64 lateral_dist
# longitude velocity to the radar; (+) = forward; unit = m/s
float64 longitude_vel
# lateral velocity to the radar; (+) = left; unit = m/s
float64 lateral_vel
# obstacle Radar Cross-Section; unit = dBsm
float64 rcs
# 0 = moving, 1 = stationary, 2 = oncoming, 3 = stationary candidate
# 4 = unknown, 5 = crossing stationary, 6 = crossing moving, 7 = stopped
int32 dynprop
# longitude distance standard deviation to the radar; (+) = forward; unit = m
float64 longitude_dist_rms
# lateral distance standard deviation to the radar; (+) = left; unit = m
float64 lateral_dist_rms
# longitude velocity standard deviation to the radar; (+) = forward; unit = m/s
float64 longitude_vel_rms
# lateral velocity standard deviation to the radar; (+) = left; unit = m/s
float64 lateral_vel_rms
# obstacle probability of existence
float64 probexist
# The following is only valid for the track object message
# 0 = deleted, 1 = new, 2 = measured, 3 = predicted, 4 = deleted for, 5 = new
# from merge
int32 meas_state
# longitude acceleration to the radar; (+) = forward; unit = m/s2
float64 longitude_accel
# lateral acceleration to the radar; (+) = left; unit = m/s2
float64 lateral_accel
# orientation angle to the radar; (+) = counterclockwise; unit = degree
float64 oritation_angle
# longitude acceleration standard deviation to the radar; (+) = forward; unit
# = m/s2
float64 longitude_accel_rms
# lateral acceleration standard deviation to the radar; (+) = left; unit = m/s2
float64 lateral_accel_rms
# orientation angle standard deviation to the radar; (+) = counterclockwise;
# unit = degree
float64 oritation_angle_rms
float64 length  # obstacle length; unit = m
float64 width   # obstacle width; unit = m
# 0: point; 1: car; 2: truck; 3: pedestrian; 4: motorcycle; 5: bicycle; 6: wide;
# 7: unknown
int32 obstacle_class