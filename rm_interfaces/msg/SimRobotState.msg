# One robot's ground truth, in the `map` frame (benchmark v2).
#
# `header.stamp` is the same wall clock `/tf` stamps, sampled in the same
# `Update` stage, so a GT message and the TF frame of "the same world state"
# carry exactly the same stamp. `sim_time_ns` is the fixed-step scenario clock
# and is what scoring must use for anything rate-like.

std_msgs/Header header
uint64 sim_time_ns          # fixed-step accumulated scenario time; 0 = scenario start
uint8  robot_id             # 0 = ego (controlled), 1..n = targets
uint8  team                 # 0 = red, 1 = blue
uint8  robot_type           # 0 = infantry3, 1 = infantry4, 2 = hero, 3 = sentry, 4 = engineer
bool   alive
float32[3] pos              # map frame, meters
float32[3] vel              # map frame, m/s
float32 yaw                 # ROS convention (same rebasing as /tf)
float32 yaw_rate            # rad/s
float32[3] gimbal_rpy       # gimbal attitude (rad)
uint8  armor_count
