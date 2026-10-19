# Scenario execution status (benchmark v2).

std_msgs/Header header
string scenario_id
uint32 seed
float32 t_scenario          # seconds
uint8  phase                # 0 = idle, 1 = running, 2 = done
float32 max_track_deviation_m   # actual vs nominal trajectory (clamping)
float32 max_along_track_m       # same measurement along the commanded direction;
                                # start-up lag and coast are expected physics, so this
                                # never gates validity - it is reported so "moved slower
                                # than commanded" stays visible. Zero for static/spin legs.
bool   spawn_applied        # every requested spawn applied (pose + chassis heading);
                            # a scenario without spawns is trivially true
string spawn_error          # non-empty: why a requested spawn was not applied
