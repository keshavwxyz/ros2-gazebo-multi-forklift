# Status of the reverse-motion safety with the rear LiDAR (forklift_control/rear_safety.py).
# The node publishes it at each change and at 5 Hz.

std_msgs/Header header        # frame_id = the base_footprint of the robot
string state                  # CLEAR | WARN | SLOW | STOP | FAULT
float32 nearest_obstacle      # m behind the rear face (inf = no obstacle in the corridor, nan = no scan)
float32 commanded_linear_x    # last command into the filter, m/s
float32 allowed_linear_x      # command that the filter sent to the drive controller, m/s
string reason                 # empty if CLEAR
uint32 interventions          # commands that the filter decreased or blocked since the start
