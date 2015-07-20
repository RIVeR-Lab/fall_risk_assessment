### Activity Analysis and TUG Test

#### Folder Organization
+ tug_test: ROS package that provides ros nodes to learn a model, predict sitting/standing positions, visualize the output video, and log tranforms between joints
+ feature_extraction: extracting features for tug test learning
+ kinect_client: getting data streams from Kinect V2 via Windows
+ openni_tracker: skeleton tracker for openni 1 (kinect 360)
+ skeleton_tracker: skeleton_tracker for openni 2 (asus xtion)

####tug_test
tug_test package defines several rosnodes to allow performing the tug test remotely. Each of these nodes are described below.
+ learn.py: This script reads training data from data/sit_vs_others.csv file and create a learned model that can be used for predicting. The learnt model is stored in model folder.
+ predict.py: This script subscribes to joint transforms and predicts if a person a standing or sitting. Use this file when using OpenNI version < 2
+ perdict2.py: Same as predict.py for use with skeleton_tracker based off of OpenNI2.
+ visualize: This node displays a video of the person taking the tug_test with a timer for the test.
+ tf_logger: this node outputs tf transforms for all the joints mentioned in the config file conf/frames.cfg
