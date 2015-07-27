import numpy as np
import scipy.signal as sig
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
import csv

# load data
def loadData():

    # data path for linux
    # data_path = "/home/riverlab-tarik/fallrisk_ws/src/fall_risk_assessment/human_motion_analysis/tug_test/data/vin_ep1.csv"
    # data path for mac
    data_path = "/Users/tkelestemur/github/fall_risk_assessment/human_motion_analysis/tug_test/data/vin_ep1.csv"
    f = open(data_path, 'rb')
    data = csv.reader(f)

    target = []
    feetD_x = []
    feetD_y = []
    feetD_z = []
    ref_frames =  ["head_1 ", "left_elbow_1 ", "left_foot_1 ", "left_hand_1 ", "left_hip_1 ", "left_knee_1 ", "left_shoulder_1 ", "neck_1 ", "right_elbow_1 ", "right_foot_1 ", "right_hand_1 ", "right_hip_1 ", "right_knee_1 ", "right_shoulder_1 ", "torso_1 " ]
    ref = ref_frames.index("left_foot_1 ")
    for row in data:
        if row[1] == "reference_frame":
            target_frames = row

        if row[1] == ref_frames[ref]:
            temp_x = row[target_frames.index("right_foot_1_X")]
            temp_y = row[target_frames.index("right_foot_1_Y")]
            temp_z = row[target_frames.index("right_foot_1_Z")]

            feetD_x.append(float(temp_x))
            feetD_y.append(float(temp_y))
            feetD_z.append(float(temp_z))
            # print feetD_x, type(feetD_x)
            # print feetD_x
    return np.asarray(feetD_x), np.asarray(feetD_y), np.asarray(feetD_z)

# preprocess data
def processPrintData(feetD_x, feetD_y, feetD_z):

    feetD_x_abs = np.absolute(feetD_x)
    feetD_y_abs = np.absolute(feetD_y)
    feetD_z_abs = np.absolute(feetD_z)

    feetD_z_csp = sig.cspline1d(feetD_z, 11)
    # feetD_z_qsp = sig.cubic(feetD_z)
    # feetD_z_med = sig.medfilt(feetD_z, kernel_size=13)
    # feetD_z_wiener = sig.wiener(feetD_z, mysize=23, noise=0.2)
    # feetD_z_savgol = sig.savgol_filter(feetD_z, 5, 2)

    feetD_z_csp_abs = np.absolute(feetD_z_csp)
    peaks = sig.find_peaks_cwt(feetD_z_csp_abs, np.arange(0.2,10))

    peaksData = np.zeros(len(peaks))
    for i in range(len(peaks)):
        peaksData[i] = feetD_z_csp_abs[peaks[i]]

    print "average of step distance: ", np.average(peaksData)
    print "# of steps: ", np.shape(peaks)
    print "peak locations: ", peaks
    # print sig.argrelmin(feet_distance_z)
    # print sig.argrelextrema(feet_distance_z. np.less)

    # plt.plot(feetD_x)
    # plt.plot(feetD_y)
    plt.subplot(3, 2, 1)
    plt.plot(feetD_z,'g')
    plt.ylabel('raw data')
    plt.subplot(3, 2, 2)
    plt.plot(feetD_z_csp, 'm')
    plt.ylabel('cubic spline')
    plt.subplot(3, 2, 3)
    plt.plot(feetD_z_csp_abs)
    plt.ylabel('cubic spline abs')
    # plt.subplot(3, 2, 4)
    # plt.plot(feetD_z_wiener)
    # plt.ylabel('wiener')
    # plt.subplot(3, 2, 5)
    # plt.plot(feetD_z_savgol)
    # plt.ylabel('savgol')
    plt.show()
    #
    # plt.plot(elbow_distance)
    # plt.show()


if __name__ == "__main__":
    feetD_x, feetD_y, feetD_z = loadData()
    processPrintData(feetD_x, feetD_y, feetD_z)
