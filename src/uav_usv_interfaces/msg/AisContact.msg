# Raw or simulated AIS contact before multi-sensor fusion.
uint8 NAV_STATUS_UNDER_WAY=0
uint8 NAV_STATUS_AT_ANCHOR=1
uint8 NAV_STATUS_NOT_UNDER_COMMAND=2
uint8 NAV_STATUS_RESTRICTED_MANEUVERABILITY=3
uint8 NAV_STATUS_MOORED=5
uint8 NAV_STATUS_UNKNOWN=15

std_msgs/Header header
uint32 mmsi
string vessel_name
geographic_msgs/GeoPoint position
float32 sog_mps
# Course and heading are clockwise from true north, in radians.
float32 cog_rad
float32 heading_rad
uint8 navigation_status
geometry_msgs/Vector3 dimensions
float32 position_accuracy_m
