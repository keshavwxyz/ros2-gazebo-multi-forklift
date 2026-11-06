# Snapshot of the coordinator: the shared mission state. Latched, at 1 Hz and at each change.
std_msgs/Header header
string state                  # IDLE | RUNNING | MISSION_COMPLETE | MISSION_FAILED | CANCELED
TaskState[] tasks
RobotStatus[] robots
string[] available_robots
Reservation[] reservations
uint32 conflicts_detected
uint32 conflicts_resolved
string failure_reason
