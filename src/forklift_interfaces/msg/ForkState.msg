# State of the fork mechanism from the lift controller node.

std_msgs/Header header
float64 height                # m, fork top surface above the lowest joint position (lift joint position)
float64 target_height         # m, last commanded height
float64 min_height            # m, lower limit of the joint
float64 max_height            # m, upper limit of the joint
bool moving
bool at_target
# Load estimate from the effort of the lift actuator (equivalent to hydraulic pressure).
bool load_detected
float64 estimated_load_kg
bool fault                    # actuator timeout / tracking error / old state
string fault_reason
