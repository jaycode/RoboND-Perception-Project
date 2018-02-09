#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from pcl_helper import *
from pr2_robot.training_helper import spawn_model
from pr2_robot.training_helper import delete_model
from pr2_robot.training_helper import initial_setup
from pr2_robot.training_helper import capture_sample
from pr2_robot.features import compute_color_histograms
from pr2_robot.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

# Update this constant with index of the world to load.
TEST_SCENE_NUM = 1
TRAINING_SET_FILE = 'training_set_1.sav'

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
       'biscuits',
       'soap',
       'soap2']

    if TEST_SCENE_NUM == 2:
        models = [\
           'biscuits',
           'soap',
           'book',
           'soap2',
           'glue']
    elif TEST_SCENE_NUM == 3:
        models = [\
           'sticky_notes',
           'snacks',
           'biscuits',
           'eraser',
           'soap2',
           'soap',
           'glue']

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    for model_name in models:
        spawn_model(model_name)

        for i in range(50):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, using_hsv=True)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

        delete_model()


    pickle.dump(labeled_features, open(TRAINING_SET_FILE, 'wb'))

