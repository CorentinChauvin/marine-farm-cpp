#
# Alga message
#

# Position of the top left hand corner of the alga
geometry_msgs/Point32 position

# Orientation
geometry_msgs/Quaternion orientation

# Dimensions along the different axes
geometry_msgs/Point32 dimensions

# Disease heatmap
mf_common/Float32Array[] disease_heatmap
