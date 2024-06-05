extended_object_detection/ImagePoint[] points
geometry_msgs/Vector3[] translates # z == 1 is still the position is unknown, only direction
string[] labels
float64[] scores
ImagePoint[] connections # x - id of first point, y - id of second
