class ArucoMarker:
    def __init__(self, t):

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

    def to_dict(self):
        return {
            'time_sec': self.time_sec,
            'time_usec': self.time_usec,

            'fixed_frame_id': self.fixed_frame_id,
            'obj_frame_id': self.obj_frame_id,

            'coord_x': self.coord_x,
            'coord_y': self.coord_y,
            'coord_z': self.coord_z,

            'rot_x': self.rot_x,
            'rot_y': self.rot_y,
            'rot_z': self.rot_z,
            'rot_w': self.rot_w
        }