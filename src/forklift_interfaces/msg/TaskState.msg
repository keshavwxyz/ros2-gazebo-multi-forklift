# One box-delivery task in the authoritative registry of the coordinator.
string task_id                # stable, for example "task_BOX_A"
string box_id
string pickup_id
string destination_id         # planned (configuration); the robot delivers to the destination of its QR code
string qr_destination_id      # destination that the robot decoded ("" until the identification)
string state                  # AVAILABLE | RESERVED | ASSIGNED | PICKUP_IN_PROGRESS |
                              # PICKED_UP | DELIVERY_IN_PROGRESS | DELIVERED |
                              # FAILED | HELD (the load state is not known, an operator must examine it)
string owner                  # robot that does the task ("" if there is no robot)
string reserved_for           # robot for which the task is planned ("" if there is no robot)
string assignment_id          # unique for each assignment attempt
uint32 attempts
float64 last_update           # s, ROS time
string last_confirmed         # last physically verified state, for example "picked (30.0 kg)"
string failure_reason
