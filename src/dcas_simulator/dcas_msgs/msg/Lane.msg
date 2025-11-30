std_msgs/Header header
uint32 id
# For simplicity use 2D points in map frame
geometry_msgs/Point32[] lane_lines

# Lane type definitions
uint8 TYPE_UNKNOWN=0
uint8 TYPE_WHITE_SOLID=1
uint8 TYPE_WHITE_DASHED=2
uint8 TYPE_YELLOW_SOLID=3
uint8 TYPE_YELLOW_DASHED=4
uint8 TYPE_DOUBLE_YELLOW_SOLID=5
uint8 lane_type


