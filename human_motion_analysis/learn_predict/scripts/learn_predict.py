#!/usr/bin/env python
import rospy
import rospkg
import tf
from std_msgs.msg import String

# scikit libs
from sklearn import svm
from sklearn import cross_validation
import numpy as np

reference_frame = '/right_foot_1'
targetFrames = ('/head_1','/right_hip_1','/torso_1','/left_foot_1')
test_features = np.zeros((3,7))
features = np.zeros(21)

clf_svm = svm.SVC()

rospack = rospkg.RosPack()

def predict(features):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print features
    pred = clf_svm.predict(features)
    if pred == 1:
         print "sitting"
    
    elif pred == 0:
         print "standing" 
    else:
         print "don't understand"


def learn(): # built SVM model
    node_path = rospack.get_path('learn_predict')
    data_path = node_path + '/data/sit_vs_others.csv'
    f = open(data_path)
    f.readline()
    db_raw = np.genfromtxt(f, delimiter=",")
    # print "raw_data_shape:" , np.shape(db_raw)
    train_features = db_raw[:,1:22] # DatasetPerson1 & DatasetPerson2
    # print "features_shape:" , np.shape(train_features)
    train_labels = db_raw[:,22] # DatasetPerson1 & DatasetPerson2
    # print "labels_shape:" , np.shape(train_labels)

	
    clf_svm.fit(train_features, train_labels)
    #scores_accuracy = cross_validation.cross_val_score(clf_svm, train_features, train_labels, cv=10, scoring='accuracy')
    #print("Accuracy(SVM): %0.2f (+/- %0.2f)" % (scores_accuracy.mean(), scores_accuracy.std() * 2))


def listener():
	
    rospy.init_node('learn_predict')
    listener = tf.TransformListener()
    
    # rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
	    try:
		(trans,rot) = listener.lookupTransform(reference_frame, targetFrames[0], rospy.Time(0))
		test_features[0] = trans + rot 
		#print test_features[0]
	    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
	    try:
		(trans,rot) = listener.lookupTransform(reference_frame, targetFrames[1], rospy.Time(0))
		test_features[1] = trans + rot 
		#print test_features[1]
	    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
	    try:
		(trans,rot) = listener.lookupTransform(reference_frame, targetFrames[2], rospy.Time(0))
		test_features[2] = trans + rot 
		#print test_features[2]
	    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
	     
	    features = np.hstack((test_features[0], test_features[1], test_features[2]))
	    #print features
	    return features

    # rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # return test_features

if __name__ == '__main__':
    learn()
    test_features = listener()
    while not rospy.is_shutdown():
    	predict(test_features)