int32 type_id                                       # id of object type, according to ObjectBase
string type_name                                    # name of object type, according to ObjectBase

float64 score                                       # confidence coefficient of current result
extended_object_detection/Rect rect                 # object frames on image and 3d coordinates
geometry_msgs/Transform transform                   # transform to object center in meters, if z=1 distance is unknow, only direction
extended_object_detection/SimpleObject[] objects    # set of simple objects form current complex
