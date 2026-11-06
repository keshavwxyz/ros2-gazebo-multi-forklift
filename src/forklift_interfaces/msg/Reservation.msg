# Exclusive use of a warehouse resource (zone, aisle, intersection) for a limited time.
string resource_id
string owner
float64 stamp                 # s, ROS time of the grant
float64 expires               # s, ROS time; the coordinator removes expired reservations
string direction              # the movement that the robot plans through it, for information only
