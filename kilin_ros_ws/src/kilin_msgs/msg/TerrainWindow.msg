# Robot-centred, bounded elevation window expressed in the planning map frame.
std_msgs/Header header
geometry_msgs/Pose origin
float32 resolution_m
uint32 width
uint32 height
float32[] elevation_m
uint8[] valid
