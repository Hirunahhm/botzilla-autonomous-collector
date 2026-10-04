# Leader -> collector: collect the cube estimated at `target` (map frame) and deliver it
# to the collector's HOME. Published latched; the collector acts on each new task_id once.
uint32 task_id                # unique per assignment; 0 = no task
uint32 cube_id                # the leader's registry id (one cube can be re-assigned)
geometry_msgs/Point target    # map frame; z unused
uint32 sightings              # leader detections merged into this estimate
