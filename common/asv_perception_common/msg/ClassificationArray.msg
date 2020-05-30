# This message represents an array of classifications of regions within an image
# header timestamp should be acquisition time of source image
Header header

# height of the original image from which the classifications were derived
uint32 image_height

# width of the original image from which the classifications were derived
uint32 image_width          

# The array of classifications
Classification[] classifications