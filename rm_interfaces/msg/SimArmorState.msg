# One armor plate's ground truth, in the `map` frame (benchmark v2).
#
# `armor_id` is the same integer as the `/tf` frame name `armor_<N>`, so GT and
# TF can be cross-checked frame by frame by eye or by script. `robot_id` is the
# ownership v1 did not have: `ArmorId` is a construction-order counter, so it
# can never be used to infer which vehicle a plate belongs to.

std_msgs/Header header
uint64 sim_time_ns
uint32 armor_id             # identical to the /tf frame suffix in `armor_<N>`
uint8  robot_id             # owning robot (see SimRobotState.robot_id)
uint8  slot                 # stable index of the plate within its robot
float32[3] pos              # plate centre, map frame, meters
float32[3] normal           # plate normal, pointing out of the vehicle
float32 plate_w             # plate width (m)
float32 plate_h             # plate height (m)
