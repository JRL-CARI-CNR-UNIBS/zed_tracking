########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
   This sample shows how to detect a human bodies and draw their 
   modelised skeleton in an OpenGL window
"""
import cv2
import sys
import pyzed.sl as sl
import time
import ogl_viewer.viewer as gl
import numpy as np
import json
import argparse

def parse_args(init, opt):
    if len(opt.input_svo_file)>0 and opt.input_svo_file.endswith(".svo"):
        init.set_from_svo_file(opt.input_svo_file)
        print("[Sample] Using SVO File input: {0}".format(opt.input_svo_file))
    elif len(opt.ip_address)>0 :
        ip_str = opt.ip_address
        if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
            init.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
            print("[Sample] Using Stream input, IP : ",ip_str)
        elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
            init.set_from_stream(ip_str)
            print("[Sample] Using Stream input, IP : ",ip_str)
        else :
            print("Unvalid IP format. Using live stream")
    
    if ("HD2K" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD2K
        print("[Sample] Using Camera in resolution HD2K")
    elif ("HD1200" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1200
        print("[Sample] Using Camera in resolution HD1200")
    elif ("HD1080" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1080
        print("[Sample] Using Camera in resolution HD1080")
    elif ("HD720" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD720
        print("[Sample] Using Camera in resolution HD720")
    elif ("SVGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.SVGA
        print("[Sample] Using Camera in resolution SVGA")
    elif ("VGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.VGA
        print("[Sample] Using Camera in resolution VGA")
    elif len(opt.resolution)>0:
        print("[Sample] No valid resolution entered. Using default")
    else :
        print("[Sample] Using default resolution")
    
    if opt.display:
        print("[Sample] OpenGL Viewer is enabled")


def progress_bar(iter, tot_iter, bar_length=50):
    percent_done = svo_position / nb_frames * 100
    done_length = int(bar_length * percent_done / 100)
    bar = '=' * done_length + '-' * (bar_length - done_length)
    sys.stdout.write('[%s] %i%s  frame: %i / %i\r' % (bar, percent_done, '%', iter, tot_iter))
    sys.stdout.flush()

def addIntoOutput(out, identifier, tab):
    out[identifier] = []
    for element in tab:
        out[identifier].append(element)
    return out

def serializeBodyData(body_data):
    """Serialize BodyData into a JSON like structure"""
    out = {}
    out["id"] = body_data.id
    out["unique_object_id"] = str(body_data.unique_object_id)
    out["tracking_state"] = str(body_data.tracking_state)
    out["action_state"] = str(body_data.action_state)
    addIntoOutput(out, "position", body_data.position)
    addIntoOutput(out, "velocity", body_data.velocity)
    addIntoOutput(out, "bounding_box_2d", body_data.bounding_box_2d)
    out["confidence"] = body_data.confidence
    addIntoOutput(out, "bounding_box", body_data.bounding_box)
    addIntoOutput(out, "dimensions", body_data.dimensions)
    addIntoOutput(out, "keypoint_2d", body_data.keypoint_2d)
    addIntoOutput(out, "keypoint", body_data.keypoint)
    addIntoOutput(out, "keypoint_cov", body_data.keypoints_covariance)
    addIntoOutput(out, "head_bounding_box_2d", body_data.head_bounding_box_2d)
    addIntoOutput(out, "head_bounding_box", body_data.head_bounding_box)
    addIntoOutput(out, "head_position", body_data.head_position)
    addIntoOutput(out, "keypoint_confidence", body_data.keypoint_confidence)
    addIntoOutput(out, "local_position_per_joint", body_data.local_position_per_joint)
    addIntoOutput(out, "local_orientation_per_joint", body_data.local_orientation_per_joint)
    addIntoOutput(out, "global_root_orientation", body_data.global_root_orientation)
    return out

def serializeBodies(bodies):
    """Serialize Bodies objects into a JSON like structure"""
    out = {}
    out["is_new"] = bodies.is_new
    out["is_tracked"] = bodies.is_tracked
    out["timestamp"] = bodies.timestamp.data_ns
    out["body_list"] = []
    for sk in bodies.body_list:
        out["body_list"].append(serializeBodyData(sk))
    return out

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


if __name__ == "__main__":
    # CLI arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, help='Path to an .svo file, if you want to replay it',default = '')
    parser.add_argument('--ip_address', type=str, help='IP Adress, in format a.b.c.d:port or a.b.c.d, if you have a streaming setup', default = '')
    parser.add_argument('--resolution', type=str, help='Resolution, can be either HD2K, HD1200, HD1080, HD720, SVGA or VGA', default = '')
    parser.add_argument('--display', type=str, help='Visualize keypoints while processing, can be either True of False', default = False)
    opt = parser.parse_args()

    # common parameters
    init_params = sl.InitParameters()
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA

    parse_args(init_params, opt)

    zed = sl.Camera()
    error_code = zed.open(init_params)
    if(error_code != sl.ERROR_CODE.SUCCESS):
        print("Can't open camera: ", error_code)

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    error_code = zed.enable_positional_tracking(positional_tracking_parameters)
    if(error_code != sl.ERROR_CODE.SUCCESS):
        print("Can't enable positionnal tracking: ", error_code)

    body_tracking_parameters = sl.BodyTrackingParameters()
    body_tracking_parameters.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
    body_tracking_parameters.body_format = sl.BODY_FORMAT.BODY_38
    body_tracking_parameters.enable_body_fitting = True
    body_tracking_parameters.enable_tracking = True

    error_code = zed.enable_body_tracking(body_tracking_parameters)
    if(error_code != sl.ERROR_CODE.SUCCESS):
        print("Can't enable positionnal tracking: ", error_code)

    # Get ZED camera information
    camera_info = zed.get_camera_information()

    if opt.display:
        image = sl.Mat()
        viewer = gl.GLViewer()
        viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, \
                    body_tracking_parameters.enable_tracking, \
                    body_tracking_parameters.body_format)

    # Create ZED objects filled in the main loop
    bodies = sl.Bodies()
    # single_bodies = [sl.Bodies]

    # Start SVO conversion to AVI/SEQUENCE
    nb_frames = zed.get_svo_number_of_frames()
    sys.stdout.write("Extracting data from SVO to JSON... Use Ctrl-C to interrupt conversion.\n")
    sys.stdout.write("Number of frames to analyze: " + str(nb_frames) + ".\n")
    
    skeleton_file_data = {}
    while True:
        err = zed.grab()
        if err == sl.ERROR_CODE.SUCCESS:
            svo_position = zed.get_svo_position()
            zed.retrieve_bodies(bodies)
            skeleton_file_data[str(svo_position)] = serializeBodies(bodies)
            if opt.display:
                viewer.update_view(image, bodies)

        # Display progress
        progress_bar(svo_position, nb_frames, 30)

        if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            progress_bar(nb_frames, nb_frames, 30)
            sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
            break

        if svo_position == 500:
            break

    # Save data into JSON file:
    filename = opt.input_svo_file.split("/")[-1].split(".")[0] + "_temp.json"
    path = "../data_raw/"
    file_sk = open(path + filename, 'w')
    sys.stdout.write("Saving to JSON file...\n")
    file_sk.write(json.dumps(skeleton_file_data, cls=NumpyEncoder, indent=2))
    sys.stdout.write("Saving to JSON file... COMPLETED.\n")
    file_sk.close()

    if opt.display:
        viewer.exit()