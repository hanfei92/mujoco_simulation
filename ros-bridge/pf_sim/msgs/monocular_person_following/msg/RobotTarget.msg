std_msgs/Header header

bool target_visible
bool target_valid
string target_source
string tracker_state
string robot_state
float32 target_lost_duration

# Navigation target for the chassis, expressed in map.
geometry_msgs/Point navigation_point_map
float32 navigation_yaw_map
bool navigation_valid

# Arm target, expressed in g_base. On loss this carries the last valid point.
geometry_msgs/Point person_g_base
float32 person_yaw_g_base
bool person_g_base_valid
