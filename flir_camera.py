#!/usr/bin/env python3

import os
import PySpin
import sys
import time

import ipaddress
import struct

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from std_srvs.srv import Empty, EmptyResponse
from mouss_ros.srv import GetDropConfig, GetDropConfigResponse

print("hello")

drop_config_dict = {}

camera_1_name = left_camera_name = "camera_left"
camera_1_ip = left_camera_ip = "192.168.0.2"

camera_2_name = right_camera_name = "camera_right"
camera_2_ip = right_camera_ip = "192.168.0.3"

camera_ips = []
camera_pubs = []
cam_vars = []
cam_names = []

trigger_counter = 0
trigger_topic = "/trigger_counter"
# trigger_pub = None
save_path = ''

default_camera_params = {
    "BinningHorizontal" : 2,
    "BinningVertical"   : 2,
    "Height"            : 1456,
    "Width"             : 1936,
    "ExposureTimeAbs"   : 15000
}


flir_attr_datatype_dict = {
    "BinningHorizontal" : "int",
    "BinningVertical"   : "int",
    "Height"            : "int",
    "Width"             : "int",
    "ExposureAuto"      : "enum",
    "ExposureTime"      : "float",
    "GainAuto"          : "enum",
    "Gain"              : "float",
    "AutoExposureExposureTimeUpperLimit": "float",
    "AutoExposureGainUpperLimit"        : "float"
}

def get_camera_attribute(camera_nodemap, attribute_name, attribute_type):

    if attribute_type == "enum":
        attribute_obj = PySpin.CEnumerationPtr(camera_nodemap.GetNode(attribute_name))
        return attribute_obj.GetIntValue()

    elif attribute_type == 'int':
        attribute_obj = PySpin.CIntegerPtr(camera_nodemap.GetNode(attribute_name))
        return attribute_obj.GetValue()


print("2")
def set_camera_attribute(camera_nodemap, attribute_name, attribute_type, attribute_value):

    if attribute_type == "enum":
        attribute_obj = PySpin.CEnumerationPtr(camera_nodemap.GetNode(attribute_name))
        attribute_value_obj = attribute_obj.GetEntryByName(attribute_value)
        attribute_obj.SetIntValue(attribute_value_obj.GetValue())

    elif attribute_type == 'int':
        attribute_obj = PySpin.CIntegerPtr(camera_nodemap.GetNode(attribute_name))
        attribute_obj.SetValue(int(attribute_value))

    elif attribute_type == 'bool':
        attribute_obj = PySpin.CBooleanPtr(camera_nodemap.GetNode(attribute_name))
        attribute_obj.SetValue(attribute_value)

    elif attribute_type == 'float':
        attribute_obj = PySpin.CFloatPtr(camera_nodemap.GetNode(attribute_name))
        attribute_obj.SetValue(float(attribute_value))

def get_camera_config_from_drop_config():
    # camera_keys = ["BinningHorizontal", "BinningVertical", "Height", "Width", "ExposureTimeAbs"]
    camera_keys = ["BinningHorizontal", "BinningVertical", "Height", "Width",
                   "ExposureAuto","ExposureTime", "AutoExposureExposureTimeUpperLimit",
                    "GainAuto", "Gain"]
    camera_config = { camera_key: drop_config_dict[camera_key] for camera_key in camera_keys }
    camera_config["AutoExposureGainUpperLimit"] = 47.0
    return camera_config

def cameras_ready_cb(empty_msg):
    return EmptyResponse()
print("3")

def main():
    print("hello")
    global cam_vars, cam_names

    # create an instance
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    num_cameras = cam_list.GetSize()

    print('Number of FLIR cameras detected: %d' % num_cameras)
    print('camera IPs = %s' % camera_ips)
    print(camera_ips)

    cam_vars = []  # Create an empty list to store camera objects

    for i, cam in enumerate(cam_list):
        # Retrieve TL device nodemap
        nodemap = cam.GetTLDeviceNodeMap()

        tli = PySpin.TransportLayerInterface(nodemap)

        cam_ip_decoded = str(ipaddress.IPv4Address(struct.pack(">I", tli.GevDeviceIPAddress.GetValue())))

        if cam_ip_decoded in camera_ips:
            cam.Init()
            print("Camera {} Initialized".format(cam_ip_decoded))

            cam_nodemap = cam.GetNodeMap()

            # ... Rest of your camera initialization code ...

            cam.BeginAcquisition()
            print("Camera {} Beginning Acquisition".format(cam_ip_decoded))
            cam_vars.append(cam)  # Store the camera object in the list

    try:
        # Do your image acquisition and processing here

        # When you're done with the cameras, release them
        for cam in cam_vars:
            cam.EndAcquisition()
            cam.DeInit()  # Deinitialize the camera object

    except Exception as e:
        print("Exception:", str(e))
    finally:
        # Release the camera list and system
        del cam
        del cam_vars  # Ensure that the camera objects are out of scope
        del cam_list
        #cam_list.Clear()
        system.ReleaseInstance()


# def main():
#     print("hello")
#     global cam_vars, cam_names

#     # create an instance
#     system = PySpin.System.GetInstance()
#     cam_list = system.GetCameras()
#     num_cameras = cam_list.GetSize()

#     print('Number of FLIR cameras detected: %d' % num_cameras)
#     print('camera IPs = %s' % camera_ips)

#     cam = None

#     for i, cam in enumerate(cam_list):
#         # Retrieve TL device nodemap
#         nodemap = cam.GetTLDeviceNodeMap()

#         tli = PySpin.TransportLayerInterface(nodemap)

#         cam_ip_decoded = str(ipaddress.IPv4Address(struct.pack(">I", tli.GevDeviceIPAddress.GetValue())))
#         # print(cam_ip_decoded)
        

#         if cam_ip_decoded in camera_ips:
#             cam.Init()
#             print("Camera {} Initalized".format(cam_ip_decoded))

#             cam_nodemap = cam.GetNodeMap()

#             if cam_ip_decoded == camera_1_ip:
#                 ### Set Primary (Hardware Trigger)
#                 set_camera_attribute(cam_nodemap, 'LineSelector', "enum", "Line1")
#                 set_camera_attribute(cam_nodemap, 'LineMode', "enum", 'Output')
#                 set_camera_attribute(cam_nodemap, 'LineSelector', "enum", "Line2")
#                 set_camera_attribute(cam_nodemap, 'V3_3Enable', "bool", True)
#                 cam_names.append("Left")
#             if cam_ip_decoded == camera_2_ip:
#                 ### Set Secondary (Hardware Trigger)
#                 set_camera_attribute(cam_nodemap, 'TriggerSource', "enum", 'Line2')
#                 set_camera_attribute(cam_nodemap, 'TriggerOverlap', "enum", "ReadOut")
#                 set_camera_attribute(cam_nodemap, 'TriggerMode', "enum", "On")
#                 cam_names.append("Right")

#             node_acquisition_mode = PySpin.CEnumerationPtr(cam_nodemap.GetNode('AcquisitionMode'))
#             node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
#             acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
#             node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
#             # print(type(acquisition_mode_continuous))

#             # binning_horizontal = PySpin.CIntegerPtr(cam_nodemap.GetNode('BinningHorizontal'))
#             # binning_horizontal.SetValue(2)

#             # print(binning_horizontal.GetValue())

#             # binning_vertical = PySpin.CIntegerPtr(cam_nodemap.GetNode('BinningVertical'))
#             # binning_vertical.SetValue(2)
#             # set_camera_attribute(cam_nodemap, 'BinningHorizontal', "int", 2)

#             ## TODO set camera attributes retreived from service request
#             # for attribute, value in drop_config_dict.items():
#             for attribute, value in get_camera_config_from_drop_config().items():
#                 try:
#                     set_camera_attribute(cam_nodemap, attribute,
#                                          flir_attr_datatype_dict[attribute], value)
#                     #rospy.loginfo("Set Attribute " + str(attribute) + " to value " + str(value))
#                 except Exception as e:
#                     print("exception")
#                     #rospy.logwarn("Unable To Set Attribute " + str(attribute) + " to value " + str(value))
#                     #rospy.logwarn(e)



#             cam.BeginAcquisition()
#             print("Camera {} Beginning Acquisition".format(cam_ip_decoded))
#             cam_vars.append(cam)
    
#     #rospy.Service("cameras_ready", Empty, cameras_ready_cb)
#     #rospy.spin()
#     print("4")
#     del cam

#     # release the instance 
#    #system.ClearCamlist() # questionable 
#     system.ReleaseInstance()



def trigger_counter_cb(data):
    global save_path, trigger_counter, cam_vars

    trigger_counter = data.data
    # print(trigger_counter)
    # print(cam_vars)

    cam_image_results = []

    for cam in cam_vars:
        cam_image_results.append(cam.GetNextImage(1000))

    for i, cam_image_result in enumerate(cam_image_results):
        if cam_image_result.IsIncomplete():
            print("{} Camera grabbed incomplete Image number {} with status {}".format(cam_names[i], trigger_counter, cam_image_result.GetImageStatus()))
        else:
            cam_image_converted = cam_image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
            cam_filename = os.path.join(save_path, cam_names[i] + "/" + str(trigger_counter).zfill(6) + ".jpg")
            # cam1_filename = "/tmp/Left_{}.jpg".format(i)
            cam_image_converted.Save(cam_filename)

    for cam_image_result in cam_image_results:
        cam_image_result.Release()

    cam_image_results = []


if __name__ == '__main__':
    # global save_path
    if len(sys.argv) > 1:
        if (sys.argv[1] != "left" and sys.argv[1] != "right" and sys.argv[1] != "both"):
            print("Error: invalid arguement")
            print('Valid arguments: "left", "right", or "both"')

            print("Exiting...")
            sys.exit()

    node_name = "camera_node"

    if len(sys.argv) > 1 and ( sys.argv[1] == "left" or sys.argv[1] == "both"):
        camera_ips.append(camera_1_ip)
        #camera_pubs.append(rospy.Publisher(camera_1_name + "/image_raw", Image, queue_size=1))
        node_name = "camera_left_node"
    if len(sys.argv) > 1 and ( sys.argv[1] == "right" or sys.argv[1] == "both"):
        camera_ips.append(camera_2_ip)
        #camera_pubs.append(rospy.Publisher(camera_2_name + "/image_raw", Image, queue_size=1))
        node_name = "camera_right_node"
    if len(sys.argv) > 1 and ( sys.argv[1] == "both"):
        node_name = "camera_node"


    #rospy.init_node(node_name)
    #save_path = rospy.get_param("/data_save_path")

    # if sys.argv[1] == "pub" or sys.argv[1] == "test":


    #trigger_sub = rospy.Subscriber(trigger_topic, Int32, trigger_counter_cb)

    #rospy.wait_for_service('get_drop_config')
    #sp = rospy.ServiceProxy('get_drop_config', GetDropConfig)
    #resp1 = sp()

    # for attr in resp1.drop_config.__slots__:
    #     drop_config_dict[attr] = getattr(resp1.drop_config, attr)

    # drop_config_dict["ExposureTime"] = drop_config_dict.pop("ExposureTimeAbs")
    # drop_config_dict["AutoExposureExposureTimeUpperLimit"] = drop_config_dict.pop("ExposureAutoMax")

    # camera_config = get_camera_config_from_drop_config(resp1.drop_config.__slots__)
    #
    # for attr in resp1.drop_config.__slots__:
    #     drop_config_dict[attr] = getattr(resp1.drop_config, attr)


    main()
