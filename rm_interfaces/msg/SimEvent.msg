# One fire or hit event (benchmark v2).
#
# `shot_seq` is globally increasing and is the primary key pairing a `fire`
# (kind=0) with its `hit_armor` (kind=1) or `hit_env` (kind=2). Attribution is
# therefore per-shot and needs no image alignment: the event carries its own
# `sim_time_ns`, which is the scenario clock the scorer uses for lead and ToF
# (`header.stamp` stays the wall clock of the frame that emitted it).
#
# NOTE ON THE CONTRACT: `IMPLEMENTATION_PLAN.md` §3.2 listed this message
# without `sim_time_ns`, but §3.4.4 ("/sim/events/* 自带 sim_time_ns 与
# shot_seq") and `collect.py`'s `write_event` (`int(m.sim_time_ns)`) both require
# it. Added as `uint64` next to `header`, matching SimRobotState/SimArmorState.

std_msgs/Header header
uint64 sim_time_ns          # fixed-step scenario clock; 0 = scenario start
uint8  kind                 # 0 = fire, 1 = hit_armor, 2 = hit_env
uint32 shot_seq             # global counter; pairs fire <-> hit
uint8  shooter_id
uint8  target_id            # kind=1 only; 255 = invalid. NOT 0: robot 0 is the ego
uint32 armor_id             # kind=1 only; 0xFFFFFFFF = invalid
float32[3] world_point      # fire: muzzle position. hit: the projectile's
                            # Transform in the frame that emitted the event --
                            # NOT the contact point (by then the projectile has
                            # been resolved for up to one physics step, and may
                            # already have bounced off the plate)
float64[3] contact_point    # kind=1 only: the true contact point on the plate
                            # (world/map frame, metres), obtained from a
                            # between-steps shape cast of the projectile sphere.
                            # kind=0/2 and "cannot be recovered": all NaN
                            # (never 0 -- 0 is a valid coordinate)
float32[3] velocity         # fire: muzzle velocity; hit: impact velocity
float32 flight_time_s       # filled in for kind=1/2; app-clock difference
                            # between the fire and the resolve (the event itself
                            # carries `sim_time_ns` for the scenario clock)
