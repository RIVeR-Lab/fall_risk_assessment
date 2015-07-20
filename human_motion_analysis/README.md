### Activity Analysis and TUG Test

#### Package & Node Organization

+ tug_test: ros pkgs related to tug test
  + visualize: visualizing robot's view, prediction status and timer
  + tf_logger:  extracting transforms between joints
  + predict: sit & stand prediction code
  + learn:  training code for sit & stand prediction


+ feature_extraction: extracting features for tug test learning
+ kinect_client: getting data streams from Kinect V2 via Windows
+ openni_tracker: skeleton tracker for openni 1 (kinect 360)
+ skeleton_tracker: skeleton_tracker for openni 2 (asus xtion)
+ tug_bringup: launch files for tug test pkgs
