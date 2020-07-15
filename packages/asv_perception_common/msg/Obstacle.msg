# Represents a 3D obstacle

# header timestamp should be acquisition time of source data
Header header

# unique obstacle identifier
string id

# initial observation time
time observed_initial

# classification label
string label

# classification label probability
float32 label_probability

# centroid and orientation
geometry_msgs/PoseWithCovariance pose

# estimated rectangular dimensions
geometry_msgs/Vector3 dimensions

# estimated velocity [m/s]
geometry_msgs/TwistWithCovariance velocity

# point representation of the object
sensor_msgs/PointCloud2 points