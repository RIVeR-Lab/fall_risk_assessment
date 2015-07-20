#!/usr/bin/env python

# ros libs
import rospy
import rospkg
import tf
from std_msgs.msg import Int8

# scikit libs
import numpy as np
from sklearn import svm
from sklearn import cross_validation
from sklearn.externals import joblib

import const


reference_frame = '/right_foot'
targetFrames = ('/head', '/right_hip', '/torso', '/left_foot')
test_features = np.zeros((3, 7))
features = np.zeros(21)

STANDING = {'no': 1, 'value': "Standing"}
SITTING = {'no': 2, 'value': "Sitting"}
WALKING = {'no': 3, 'value': "Walking"}
NONE = {'no': 0, 'value': "Unkwon activity!"}



rospack = rospkg.RosPack()
node_path = rospack.get_path('tug_test')
model_path = node_path + '/model/model.pkl'
clf_svm = joblib.load(model_path)
previousPose = 3
count = 0

def predict(features):

    pred = clf_svm.predict(features)
    if pred == 1:
        # rospy.loginfo(SITTING['value'])
        # activity_state = "sitting"
        activity = SITTING
    elif pred == 0:
        # rospy.loginfo(STANDING['value'])
        # activity_state = "standing"
        activity = STANDING
    else:
        rospy.loginfo( "else")
        activity = NONE
    return activity


if __name__ == '__main__':
    rospy.init_node('activity_predict')
    listener = tf.TransformListener()
    counter = 0



    pub = rospy.Publisher('activity_state', Int8, queue_size=10)
    while not rospy.is_shutdown():
        try:

            (trans, rot) = listener.lookupTransform(reference_frame, targetFrames[0], rospy.Time(0))
            test_features[0] = trans + rot
            (trans, rot) = listener.lookupTransform(reference_frame, targetFrames[1], rospy.Time(0))
            test_features[1] = trans + rot
            (trans, rot) = listener.lookupTransform(reference_frame, targetFrames[2], rospy.Time(0))
            test_features[2] = trans + rot

            features = np.hstack((test_features[0], test_features[1], test_features[2]))
            currentPose = predict(features)

            if previousPose == currentPose['no']:
                count += 1
                if count > 3:
                    pub.publish(currentPose['no'])
                    rospy.loginfo(currentPose['value'] +" is pusblished!")

            previousPose = currentPose['no']




            #pub.publish(activity_state)
            #rospy.loginfo(activity_state)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("exception")

	#rospy.spin()
