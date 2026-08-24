# Fixed order: front-left, front-right, rear-left, rear-right.
Headers header
# True when base_to_output_rotation was estimated rather than assumed.
bool orientation_valid
# Quaternion rotating vectors from base_link into header.frame_id.
geometry_msgs/Quaternion base_to_output_rotation
# StairTerrainStamped.header.seq used for this estimate; -1 means no terrain.
int32 terrain_seq
geometry_msgs/Point com
# Nominal wheel-bottom points from robot geometry; independent of contact sensors.
geometry_msgs/Point[4] contact_points
# Sensor fields are validation-only and are not used by the geometry monitor.
bool[4] sensor_valid
bool[4] in_contact
bool[4] supported
float64[4] contact_force
