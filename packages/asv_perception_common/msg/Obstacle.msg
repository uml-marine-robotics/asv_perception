# Represents a 3D obstacle

# polygon describing the obstacle, with first and last points connected
geometry_msgs/Polygon shape

# classification label
string label

# classification label probability
float32 probability

# centroid and orientation
geometry_msgs/Pose pose

# velocity
geometry_msgs/Accel velocity