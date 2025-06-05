
class ArucoMarker:
    def __init__(self, t=None, **kwargs):
        if t:
            # time
            self.time_sec = t.header.stamp.sec
            self.time_usec = t.header.stamp.nanosec

            # frame ids of obj, and what it's relative to (fixed)
            self.fixed_frame_id = t.header.frame_id
            self.obj_frame_id = t.child_frame_id
            
            # coords
            self.coord_x = t.transform.translation.x
            self.coord_y = t.transform.translation.y
            self.coord_z = t.transform.translation.z

            # quaternion rot
            self.rot_x = t.transform.rotation.x
            self.rot_y = t.transform.rotation.y
            self.rot_z = t.transform.rotation.z
            self.rot_w = t.transform.rotation.w

            # t is type TransformStamped, so....
                # std_msgs/Header header
                #     builtin_interfaces/Time stamp
                #     string frame_id
                # string child_frame_id
                # geometry_msgs/Transform transform
                #     geometry_msgs/Vector3 translation
                #         float64 x
                #         float64 y
                #         float64 z
                #     geometry_msgs/Quaternion rotation
                #         float64 x
                #         float64 y
                #         float64 z
                #         float64 w
        else:
            self.__dict__.update(kwargs)

    
# for yaml load
def arucomarker_constructor(loader, node):
    values = loader.construct_mapping(node)
    return ArucoMarker(**values)

# for yaml dump
def arucomarker_representer(dumper, data):
    return dumper.represent_mapping('!ArucoMarker', data.__dict__)