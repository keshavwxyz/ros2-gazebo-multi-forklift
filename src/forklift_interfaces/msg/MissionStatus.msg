# Mission record from the task manager.

std_msgs/Header header
string state                  # name of the current state of the state machine, for example "NAVIGATE_TO_PICKUP"
string current_box_id
string current_destination_id
string[] boxes_detected
string[] boxes_picked
string[] boxes_delivered
string[] boxes_failed
uint32 boxes_total
bool paused
bool mission_complete
string failure_reason         # empty if the last transition was not a failure
