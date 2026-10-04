# Leader -> collector: collect the cube estimated at `target` (map frame) and deliver it
# to the collector's HOME. Published latched; the collector acts on each new task_id once.
uint32 task_id                # 0 = no task
geometry_msgs/Point target    # map frame; z unused
uint32 sightings              # leader detections merged into this estimate
