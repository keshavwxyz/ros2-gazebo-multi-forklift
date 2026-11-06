# Heartbeat of one forklift for the fleet coordinator (forklift_fleet/fleet_agent.py), 2 Hz.
# It uses only the estimates of the robot: AMCL pose, mission status, fork load sensing,
# rear-LiDAR safety. If the heartbeat of a robot stops, the coordinator marks it as not available.

std_msgs/Header header
string robot_id               # for example, "forklift_1" (= its ROS namespace)
string state                  # IDLE | TASK_ASSIGNED | NAVIGATING_TO_PICKUP | PICKING |
                              # CARRYING_LOAD | NAVIGATING_TO_DESTINATION | PLACING |
                              # RETURNING | PAUSED | FAULT
string mission_state          # raw state of the mission manager (for example, ALIGN_FORKS)
string current_box_id
string current_destination_id # decoded from the QR code of the box after the identification
bool localized
float64 x                     # map frame, from AMCL (never the ground truth of the simulator)
float64 y
float64 yaw
float32 speed                 # m/s, from the EKF
bool carrying                 # lift load sensing
string rear_state             # SafetyStatus.state
float32 rear_nearest          # m
string[] boxes_delivered      # placements that this robot verified since the start
string[] boxes_picked
string failure_reason
bool mission_running          # a RunMission goal runs on this robot
bool nav_active               # a NavigateToPose goal runs (mission or coordinator)
geometry_msgs/Point[] path_ahead   # Nav2 plan, next 6 m or less, 0.25 m between points (map frame)
