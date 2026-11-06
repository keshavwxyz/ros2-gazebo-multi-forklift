# Diagnostic record of one conflict and of its solution.
std_msgs/Header header
string conflict_id
string type                   # TASK_CLAIM | ZONE | PATH | HEAD_ON | REAR_BLOCKED | DEADLOCK
string[] robots
string[] tasks                # the current task ids of the robots
string resource               # zone, place or location of the conflict
string priority_robot
string yielding_robot
string resolution             # for example, "forklift_2 waits; forklift_1 passes"
string status                 # ACTIVE | RESOLVED | ESCALATED
float32 wait_time             # s of wait time until now / in total
bool recovery_succeeded
