# Generic object description for environment/perception
std_msgs/Header header
uint32 id
uint8 type
geometry_msgs/Pose pose
geometry_msgs/Vector3 size  # For GEOM_BOX (fallback; width(x), depth(y), height(z))
geometry_msgs/Twist twist

# Geometry shape (to visualize true footprint, e.g., buildings)
uint8 GEOM_BOX=0
uint8 GEOM_POLYGON=1
uint8 geom_type
geometry_msgs/Point32[] footprint  # Map-frame polygon (z ignored); closed by convention (first==last not required)
float32 height  # For GEOM_POLYGON, extruded height (meters)

# Type labels (keep minimal and course-focused)
uint8 LABEL_UNKNOWN=0
uint8 LABEL_VEHICLE=1
uint8 LABEL_PEDESTRIAN=2
uint8 LABEL_CYCLIST=3
uint8 LABEL_TRAFFIC_LIGHT=10
uint8 LABEL_TRAFFIC_SIGN=11
uint8 LABEL_GUARDRAIL=12
uint8 LABEL_BUILDING=13


