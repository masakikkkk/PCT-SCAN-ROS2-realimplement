# One coherent geometry/version/health envelope; no pairing of separate topics.
# Header stamp is the latest successfully incorporated input cloud's stamp.
std_msgs/Header header
# Changes on node restart or explicit map reset.
string epoch
# Increases only when the exported stable surface geometry changes.
uint64 version
bool healthy
string reason
# Full stable surface snapshot when healthy, empty when unhealthy.
sensor_msgs/PointCloud2 cloud
