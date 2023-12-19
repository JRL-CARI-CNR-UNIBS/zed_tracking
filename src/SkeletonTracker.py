from dataclasses import dataclass
import pyzed.sl as sl
import rospy
from utils import *


@dataclass
class SkeletonTracker:
    # Class variable definition
    camera: sl.Camera
    init_params: sl.InitParameters
    body_params: sl.BodyTrackingParameters
    positional_tracking_param: sl.PositionalTrackingParameters
    bodies: sl.Bodies
    body_runtime_params: sl.BodyTrackingRuntimeParameters
    skeleton_pub: rospy.Publisher

    def __init__(self):
        # Create a Camera object
        self.camera = sl.Camera()
        
        # Create a InitParameters object and set configuration parameters
        self.set_init_params()

        # Create a BodyTrackingParameters object and set configuration parameters
        self.set_body_params()

        # Create a Bodies object to retrieve detected people
        self.bodies = sl.Bodies()

        # Create a BodyTrackingRuntimeParameters object and set configuration parameters
        self.set_body_runtime_params()
    

    def set_init_params(self):
        self.init_params = sl.InitParameters()

        self.init_params.camera_resolution = read_param('camera_resolution',
                                                        default_value=sl.RESOLUTION.HD1080)
        self.init_params.depth_mode = read_param('depth_mode',
                                                 default_value=sl.DEPTH_MODE.PERFORMANCE)
        self.init_params.coordinate_units = read_param('unit',
                                                       default_value=sl.UNIT.METER)
        self.init_params.sdk_verbose = read_param('sdk_verbose',
                                                  default_value=1)
        
        err = self.camera.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr("Camera Open : "+repr(err)+". Exit program.")
            exit()


    def set_body_params(self):
        self.body_params = sl.BodyTrackingParameters()

        self.body_params.detection_model = read_param('detection_model',
                                                      default_value=sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST) # optimize the runtime or the accuracy
        self.body_params.enable_tracking = read_param('enable_tracking',
                                                      default_value=True)    
        self.body_params.image_sync = read_param('image_sync',
                                                 default_value=True)
        self.body_params.enable_segmentation = read_param('enable_segmentation',
                                                          default_value=False)
        self.body_params.enable_body_fitting = read_param('enable_body_fitting',
                                                          default_value=True) # optimize the person joints position, requires more computations
        
        if self.body_params.enable_tracking:
            self.positional_tracking_param = sl.PositionalTrackingParameters()
            self.positional_tracking_param.set_as_static = True # enable if the camera is static for better performance
            self.positional_tracking_param.set_floor_as_origin = True # enable if the camera is static and the floor is the origin of the world frame
            self.camera.enable_positional_tracking(self.positional_tracking_param) # enable positional tracking

        rospy.loginfo("Body tracking: Loading Module...")
        err = self.camera.enable_body_tracking(self.body_params)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr("Enable Body Tracking : "+repr(err)+". Exit program.")
            self.camera.close()
            exit()


    def set_body_runtime_params(self):
        self.body_runtime_params = sl.BodyTrackingRuntimeParameters()
        
        # For outdoor scene or long range, the confidence should be lowered to avoid missing detections (~20-30)
        # For indoor scene or closer range, a higher confidence limits the risk of false positives and increase the precision (~50+)
        self.body_runtime_params.detection_confidence_threshold = read_param('detection_confidence_threshold',
                                                                            default_value=40)


    def close_camera(self):
        self.camera.disable_body_tracking()
        self.camera.close()
