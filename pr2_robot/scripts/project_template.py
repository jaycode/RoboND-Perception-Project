#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

# Update this constant with index of the world to load.
TEST_SCENE_NUM = 2
MODEL_FILE = 'model_3_1000.sav'
REQUEST_YAML_FILE = 'requests_2.yaml'
SEND_REQUESTS = False

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, object_name, arm_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = int(test_scene_num.data)
    yaml_dict["arm_name"]  = str(arm_name.data)
    yaml_dict["object_name"] = str(object_name.data)
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    yaml_dict["pick_pose"]['position']['x']= float(yaml_dict["pick_pose"]['position']['x'])
    yaml_dict["pick_pose"]['position']['y']= float(yaml_dict["pick_pose"]['position']['y'])
    yaml_dict["pick_pose"]['position']['z']= float(yaml_dict["pick_pose"]['position']['z'])
    yaml_dict["pick_pose"]['orientation']['x']= float(yaml_dict["pick_pose"]['orientation']['x'])
    yaml_dict["pick_pose"]['orientation']['y']= float(yaml_dict["pick_pose"]['orientation']['y'])
    yaml_dict["pick_pose"]['orientation']['z']= float(yaml_dict["pick_pose"]['orientation']['z'])
    yaml_dict["place_pose"]['position']['x']= float(yaml_dict["place_pose"]['position']['x'])
    yaml_dict["place_pose"]['position']['y']= float(yaml_dict["place_pose"]['position']['y'])
    yaml_dict["place_pose"]['position']['z']= float(yaml_dict["place_pose"]['position']['z'])
    yaml_dict["place_pose"]['orientation']['x']= float(yaml_dict["place_pose"]['orientation']['x'])
    yaml_dict["place_pose"]['orientation']['y']= float(yaml_dict["place_pose"]['orientation']['y'])
    yaml_dict["place_pose"]['orientation']['z']= float(yaml_dict["place_pose"]['orientation']['z'])
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    print("data_dict:")
    print(data_dict)
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    global previous_detected_objects, same_detection_times

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE =  0.004
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()

    # Extract outliers
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()


    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)

    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(50000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo("Detected {} objects: {}".format(
        len(detected_objects_labels),
        detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()

    if previous_detected_objects == detected_objects_labels:
        print("same")
        same_detection_times += 1
    else:
        same_detection_times = 0

    if same_detection_times >= 0:
        print("move")
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass

    previous_detected_objects = detected_objects_labels

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    responses = []
    group_arm = {'green': 'right', 'red': 'left'}
    box_positions = {'green': [], 'red': []}
    yaml_request_list = []
    test_scene_num = Int32()
    test_scene_num.data = TEST_SCENE_NUM

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    box_params = rospy.get_param('/dropbox')
    print(box_params)

    # TODO: Parse parameters into individual variables
    for box_param in box_params:
        box_positions[box_param['group']] = box_param['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for obj_to_pick in object_list_param:
        centroids = [] # list of tuples (x, y, z)

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for obj_detected in object_list:
            if obj_detected.label == obj_to_pick['name']:
                points_arr = ros_to_pcl(obj_detected.cloud).to_array()
                centroids = np.mean(points_arr, axis=0)[:3]
                centroids[0] = np.asscalar(centroids[0])
                centroids[1] = np.asscalar(centroids[1])
                centroids[2] = np.asscalar(centroids[2])

        # If we found an object to pick...
        if len(centroids) > 0:
            print("working on {}".format(obj_to_pick['name']))
            # TODO: Create 'place_pose' for the object
            pick_pose = Pose()
            pick_pose.position.x = centroids[0]
            pick_pose.position.y = centroids[1]
            pick_pose.position.z = centroids[2]

            place_pose = Pose()
            place_pose.position.x = box_positions[obj_to_pick['group']][0]
            place_pose.position.y = box_positions[obj_to_pick['group']][1]
            place_pose.position.z = box_positions[obj_to_pick['group']][2]

            object_name = String()
            object_name.data = obj_to_pick['name']

            # TODO: Assign the arm to be used for pick_place
            which_arm = String()
            which_arm.data = group_arm[obj_to_pick['group']]

            # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_request = make_yaml_dict(test_scene_num,
                                          object_name,
                                          which_arm,
                                          pick_pose,
                                          place_pose)
            yaml_request_list.append(yaml_request)

            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            if SEND_REQUESTS:
                try:
                    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                    # TODO: Insert your message variables to be sent as a service request
                    resp = pick_place_routine(test_scene_num,
                                              object_name,
                                              which_arm,
                                              pick_pose,
                                              place_pose)

                    print ("Response: ",resp.success)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml(REQUEST_YAML_FILE, yaml_request_list)

if __name__ == '__main__':

    previous_detected_objects = []
    same_detection_times = 0

    # TODO: ROS node initialization
    rospy.init_node('pick_place', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points",
                               pc2.PointCloud2, pcl_callback,
                               queue_size=1)

    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray,
                                           queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", pc2.PointCloud2,
                                      queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open(MODEL_FILE, 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

