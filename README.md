# Perception Pick and Place

## How to run this project

### Step 0: Decide Which World to Operate On

Update the following settings:
- `pr2_robot/slaunch/pick_place_project.launch`: `world_name` parameter and `pick_list_*.yaml`
- `pr2_robot/scripts/capture_features.py`: `TEST_SCENE_NUM` and `TRAINING_SET_FILE`
- `pr2_robot/scripts/train_svm.py`: `TRAINING_SET_FILE` and `MODEL_FILE`
- `pr2_robot/scripts/project_template.py`: `TEST_SCENE_NUM`, `MODEL_FILE`, and `REQUEST_YAML_FILE`

Note: Prior to running anything, make sure to run `$ source catkin_ws/devel/setup.bash`.

### Step 1: Training the Object Recognizer

There are two parts here, creating training data and training the classifier.

#### Prepare Training Data

Launch the environment

```
$ roslaunch sensor_stick training.launch
```

Spawn training data

```
$ rosrun sensor_stick capture_features.py
```

The above steps should generate `catkin_ws/training_set.sav` file.

#### Train

```
$ rosrun sensor_stick train_svm.py
```

At the end of this step, `catkin_ws/model.sav` file should be created, which can be used for inference.

### Step 2: Infer Objects

Launch the environment

```
$ roslaunch pr2_robot pick_place_project.launch
```

And then, in another terminal:

```
$ rosrun pr2_robot project_template.py
```

To see if the objects point cloud was collected properly, look at Rviz's PointCloud2 object's `\pcl_cluster` topic.


## Write-up

### Filtering with RANSAC Plane Fitting + Segmentation

See the "Debugging" section above for how to run the segmentation process.

One addition from the exercise was 