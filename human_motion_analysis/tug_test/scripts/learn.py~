#!/usr/bin/env python
import rospy
import rospkg
import tf
from std_msgs.msg import String

# scikit libs
import numpy as np
from sklearn import svm
from sklearn import cross_validation
from sklearn.externals import joblib

rospack = rospkg.RosPack()
node_path = rospack.get_path('activity_model')
clf_svm = svm.SVC()


def learn():

    data_path = node_path + '/data/sit_vs_others.csv'
    f = open(data_path)
    f.readline()
    db_raw = np.genfromtxt(f, delimiter=",")
    # print "raw_data_shape:" , np.shape(db_raw)
    train_features = db_raw[:,1:22] # DatasetPerson1 & DatasetPerson2
    # print "features_shape:" , np.shape(train_features)
    train_labels = db_raw[:,22] # DatasetPerson1 & DatasetPerson2
    # print "labels_shape:" , np.shape(train_labels)



    scores_accuracy = cross_validation.cross_val_score(clf_svm, train_features, train_labels, cv=10, scoring='accuracy')
    rospy.loginfo("Accuracy(SVM): %0.2f (+/- %0.2f)" % (scores_accuracy.mean(), scores_accuracy.std() * 2))
    clf_svm.fit(train_features, train_labels)

    # saving learned model for future usage
    model_path = node_path + '/model/model.pkl'
    joblib.dump(clf_svm, model_path)


if __name__ == '__main__':
    rospy.init_node('activity_model')
    try:
        learn()
        rospy.loginfo("Learning is completed!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Learning is not completed!")
        pass
