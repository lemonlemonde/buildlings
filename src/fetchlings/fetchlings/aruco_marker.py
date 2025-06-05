class ArucoMarker:
    def __init__(self, time, id):
        print("sup")

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