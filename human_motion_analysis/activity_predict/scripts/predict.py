#!/usr/bin/env python

# ros libs
import rospy
import rospkg
import tf
from std_msgs.msg import String

# scikit libs
import numpy as np
from sklearn import svm
from sklearn import cross_validation
from sklearn.externals import joblib


reference_frame = '/right_foot_1'
targetFrames = ('/head_1', '/right_hip_1', '/torso_1', '/left_foot_1')
test_features = np.zeros((3, 7))
features = np.zeros(21)


rospack = rospkg.RosPack()
node_path = rospack.get_path('activity_model')
model_path = node_path + '/model/model.pkl'
clf_svm = joblib.load(model_path)

def predict(features):

    pred = clf_svm.predict(features)
    if pred == 1:
        rospy.loginfo( "sitting")
        activity_state = "sitting"

    elif pred == 0:
        rospy.loginfo( "standing")
        activity_state = "standing"
    else:
        rospy.loginfo( "else")
    return activity_state


if __name__ == '__main__':
    rospy.init_node('activity_predict')
    listener = tf.TransformListener()
    counter = 0

    pub = rospy.Publisher('activity_state', String, queue_size=10)
    while not rospy.is_shutdown():
        try:

            (trans, rot) = listener.lookupTransform(reference_frame, targetFrames[0], rospy.Time(0))
            test_features[0] = trans + rot
            (trans, rot) = listener.lookupTransform(reference_frame, targetFrames[1], rospy.Time(0))
            test_features[1] = trans + rot
            (trans, rot) = listener.lookupTransform(reference_frame, targetFrames[2], rospy.Time(0))
            test_features[2] = trans + rot

            features = np.hstack((test_features[0], test_features[1], test_features[2]))
            activity_state = predict(features)

            pub.publish(activity_state)
            rospy.loginfo(activity_state)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("exception")

	#rospy.spin()
