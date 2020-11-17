uint8 DETECTED = 1  # Current result is obtained by detector
uint8 TRACKED = 2  # Current result is obtained by opencv tracker
uint8 LOST = 3      # Failed to obtain result, this is last saved

int32 id            # track unique id (in group of one object)
uint8 status        # status described above
#extended_object_detection/ImagePoint[] image_point_history
#geometry_msgs/Transform[] transform_history
