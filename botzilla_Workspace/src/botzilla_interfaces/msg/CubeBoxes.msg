# Cube detections on one camera frame, as pixel boxes with no depth. Produced by
# yolo_node in boxes mode on the leader's GPU for the collector's camera; turned into
# detected_cube on the collector by remote_detection_node, which has the depth.
std_msgs/Header header        # stamp = the source image's stamp (collector clock)
uint32 width                  # source image size, pixels
uint32 height
float32[] cx                  # box centres, pixels
float32[] cy
float32[] confidence
float32 inference_s           # model time on the leader, for latency accounting
