from dataclasses import dataclass, field
import pyzed.sl as sl
from utils import *

logger = create_logger('SkeletonTracker', level=logging.INFO)

@dataclass
class SkeletonTracker:
    camera: sl.Camera \
        = field(default=sl.Camera())
    init_params: sl.InitParameters \
        = field(default=sl.InitParameters())
    body_params: sl.BodyTrackingParameters \
        = field(default=sl.BodyTrackingParameters())
    positional_tracking_param: sl.PositionalTrackingParameters \
        = field(default=sl.PositionalTrackingParameters())
    bodies: sl.Bodies \
        = field(default=sl.Bodies())
    body_runtime_params: sl.BodyTrackingRuntimeParameters \
        = field(default=sl.BodyTrackingRuntimeParameters())
    

    def load_parameters(self, camera_params, tracking_params):
        # Camera parameters
        if camera_params['general']['grab_resolution'] in ['HD2K', 'HD1080', 'HD720', 'VGA', 'AUTO']:
            self.init_params.camera_resolution = sl.RESOLUTION[camera_params['general']['grab_resolution']]
        else:
            self.init_params.camera_resolution = sl.RESOLUTION.AUTO
            logger.warning("Camera Open: No resolution specified. Defaulting to AUTO.")

        self.init_params.camera_fps = camera_params['general']['grab_frame_rate']
        self.init_params.coordinate_units = sl.UNIT[camera_params['general']['unit']]
        self.init_params.sdk_verbose = camera_params['general']['sdk_verbose']
        self.init_params.depth_mode = sl.DEPTH_MODE[camera_params['depth']['depth_mode']]

        # Body tracking parameters
        # (https://www.stereolabs.com/docs/body-tracking/using-body-tracking)
        self.body_params.detection_model = sl.BODY_TRACKING_MODEL[tracking_params['body_params']['detection_model']]
        self.body_params.enable_body_fitting = tracking_params['body_params']['enable_body_fitting']
        self.body_params.body_format = sl.BODY_FORMAT[tracking_params['body_params']['body_format']]
        self.body_params.enable_tracking = tracking_params['body_params']['enable_tracking']
        self.body_params.image_sync = tracking_params['body_params']['image_sync']
        self.body_params.enable_segmentation = tracking_params['body_params']['enable_segmentation']

        if self.body_params.enable_tracking:
            self.positional_tracking_param.set_as_static = tracking_params['body_params']['set_as_static']
            self.positional_tracking_param.set_floor_as_origin = tracking_params['body_params']['set_floor_as_origin']

        # Body runtime parameters
        # For outdoor scene or long range, the confidence should be lowered to avoid missing detections (~20-30)
        # For indoor scene or closer range, a higher confidence limits the risk of false positives and increase the precision (~50+)
        self.body_runtime_params.detection_confidence_threshold \
            = tracking_params['body_runtime_params']['detection_confidence_threshold']


    def open_camera(self):
        logger.info("Camera Open: Loading Module...")
        if not self.camera.is_opened():
            err = self.camera.open(self.init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                logger.error("Camera Open : "+repr(err)+". Exit program.")
                exit()
        else:
            logger.warning("Camera Open: Already opened")


    def load_body_tracking(self):
        if self.body_params.enable_tracking:
            self.camera.enable_positional_tracking(self.positional_tracking_param)

        logger.info("Body tracking: Loading Module...")
        err = self.camera.enable_body_tracking(self.body_params)
        if err != sl.ERROR_CODE.SUCCESS:
            logger.error("Enable Body Tracking : "+repr(err)+". Exit program.")
            self.camera.close()
            exit()


    def close_camera(self):
        self.camera.disable_body_tracking()
        self.camera.close()
