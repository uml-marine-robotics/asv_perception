# Represents a 3D obstacle

# header timestamp should be acquisition time of source data
Header header

# unique obstacle identifier
uint32 id

# initial observation time
time observed_initial

# classification label
string label

# classification label probability
float32 label_probability

# area [m^2]
float32 area

# centroid and orientation
geometry_msgs/PoseWithCovariance pose

# estimated rectangular dimensions [m]
geometry_msgs/Vector3 dimensions

# estimated velocity [m/s]
geometry_msgs/TwistWithCovariance velocity

# 2d convex hull, points are relative to pose.position
geometry_msgs/Polygon hull2d