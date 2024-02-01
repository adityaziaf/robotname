int8 METHOD_P2P = 0
int8 METHOD_LINEAR = 1
int8 METHOD_CURVE  = 2

std_msgs/Header header
geometry_msgs/Pose2D[] pose
float64 speed #m/s
int8 method