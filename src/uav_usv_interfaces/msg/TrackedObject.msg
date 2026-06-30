# Source bitmask. Multiple bits indicate a fused track.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_LIDAR=1
uint8 SOURCE_CAMERA=2
uint8 SOURCE_AIS=4
uint8 SOURCE_FUSED=8

# Coarse semantic classification shared by all perception sources.
uint8 CLASS_UNKNOWN=0
uint8 CLASS_VESSEL=1
uint8 CLASS_BUOY=2
uint8 CLASS_DEBRIS=3
uint8 CLASS_LANDMARK=4

unique_identifier_msgs/UUID uuid
string track_id
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_update
uint8 source_mask
uint8 classification
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/Vector3 dimensions
float32 confidence
uint32 mmsi
