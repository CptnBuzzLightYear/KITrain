# ObserverInfo.msg

geometry_msgs/Point position    # Object position relative to LiDAR
float64 distance                # Object distance to LiDAR
string object_class             # Classification of the object, e.g., "vehicle", "pedestrian", "signal"
bool signal_detected            # Whether a signal is detected
string signal_setting           # Signal setting: "red", "white", or ""
