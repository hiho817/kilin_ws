# Fixed order: front-left, front-right, rear-left, rear-right.
Headers header
geometry_msgs/Point com
# Nominal wheel-bottom points from robot geometry; independent of contact sensors.
geometry_msgs/Point[4] contact_points
# Sensor fields are validation-only and are not used by the geometry monitor.
bool[4] sensor_valid
bool[4] in_contact
bool[4] supported
float64[4] contact_force
