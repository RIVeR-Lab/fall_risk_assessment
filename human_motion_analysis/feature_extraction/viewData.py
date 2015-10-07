import numpy as np
import scipy.signal as sig
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
import csv
from sys import argv, exit

# load data
def loadData(reference_frame, target_frame):

    # data path for linux
    data_path = "/home/apoorv/catkin_ws/src/fall_risk_assessment/human_motion_analysis/tug_test/data/data.csv"
    # data path for mac
    # data_path = "/Users/tkelestemur/github/fall_risk_assessment/human_motion_analysis/tug_test/data/vin_ep1.csv"
    f = open(data_path, 'rb')
    data = csv.reader(f)

    dataPos_x = []
    dataPos_y = []
    dataPos_z = []

    dataAng_x = []
    dataAng_y = []
    dataAng_z = []
    dataAng_w = []

    ref_frames =  ["head_1 ", "left_elbow_1 ", "left_foot_1 ", "left_hand_1 ", "left_hip_1 ", "left_knee_1 ", "left_shoulder_1 ", "neck_1 ", "right_elbow_1 ", "right_foot_1 ", "right_hand_1 ", "right_hip_1 ", "right_knee_1 ", "right_shoulder_1 ", "torso_1 " ]
    reference_frame = reference_frame + "_1 "
    # ref = ref_frames.index("left_foot_1 ")
    ref = ref_frames.index(reference_frame)

    target_frame_X = target_frame + "_1_X"
    target_frame_Y = target_frame + "_1_Y"
    target_frame_Z = target_frame + "_1_Z"

    target_frame_AngX = target_frame + "_1_AngX"
    target_frame_AngY = target_frame + "_1_AngY"
    target_frame_AngZ = target_frame + "_1_AngZ"
    # target_frame_AngW = target_frame + "_1_AngW"


    for row in data:
        if row[1] == "reference_frame":
            target_frames = row

        if row[1] == ref_frames[ref]:

            temp_x = row[target_frames.index(target_frame_X)]
            temp_y = row[target_frames.index(target_frame_Y)]
            temp_z = row[target_frames.index(target_frame_Z)]

            dataPos_x.append(float(temp_x))
            dataPos_y.append(float(temp_y))
            dataPos_z.append(float(temp_z))

            temp_AngX = row[target_frames.index(target_frame_AngX)]
            temp_AngY = row[target_frames.index(target_frame_AngY)]
            temp_AngZ = row[target_frames.index(target_frame_AngZ)]
            # temp_AngW = row[target_frames.index(target_frame_AngW)]

            dataAng_x.append(float(temp_AngX))
            dataAng_y.append(float(temp_AngY))
            dataAng_z.append(float(temp_AngZ))
            # dataAng_w.append(float(temp_AngW))



    return np.asarray(dataPos_x), np.asarray(dataPos_y), np.asarray(dataPos_z), np.asarray(dataAng_x), np.asarray(dataAng_y), np.asarray(dataAng_z)

# preprocess data
def processPrintData(dataPos_x, dataPos_y, dataPos_z, dataAng_x, dataAng_y, dataAng_z):

    dataPos_x_abs = np.absolute(dataPos_x)
    dataPos_y_abs = np.absolute(dataPos_y)
    dataPos_z_abs = np.absolute(dataPos_z)

    dataPos_z_csp = sig.cspline1d(dataPos_z, 11)
    dataAng_z_csp = sig.cspline1d(dataAng_z, 11)

    # dataPos_z_qsp = sig.cubic(dataPos_z)
    # dataPos_z_med = sig.medfilt(dataPos_z, kernel_size=13)
    # dataPos_z_wiener = sig.wiener(dataPos_z, mysize=23, noise=0.2)
    # dataPos_z_savgol = sig.savgol_filter(dataPos_z, 5, 2)

    dataPos_z_csp_abs = np.absolute(dataPos_z_csp)
    dataAng_z_csp_abs = np.absolute(dataAng_z_csp)
    peaks = sig.find_peaks_cwt(dataPos_z_csp_abs, np.arange(0.2,10))

    peaksData = np.zeros(len(peaks))
    for i in range(len(peaks)):
        peaksData[i] = dataPos_z_csp_abs[peaks[i]]

    print "average of step distance: ", np.average(peaksData)
    print "# of steps: ", np.shape(peaks)
    print "peak locations: ", peaks
    # print sig.argrelmin(feet_distance_z)
    # print sig.argrelextrema(feet_distance_z. np.less)

    # plt.plot(dataPos_x)
    # plt.plot(dataPos_y)
    plt.subplot(3, 2, 1)
    plt.plot(dataPos_z,'g')
    plt.ylabel('raw / positions(z)')
    plt.subplot(3, 2, 3)
    plt.plot(dataPos_z_csp, 'm')
    plt.ylabel('cubic spline / position(z)')
    plt.subplot(3, 2, 5)
    plt.plot(dataPos_z_csp_abs)
    plt.ylabel('cubic spline abs/ position(z)')
    plt.subplot(3, 2, 2)
    plt.plot(dataAng_z)
    plt.ylabel('raw / rotation(z)')
    plt.subplot(3, 2, 4)
    plt.plot(dataAng_z_csp)
    plt.ylabel('cubic spline / rotation(z)')
    plt.subplot(3, 2, 6)
    plt.plot(dataAng_z_csp_abs)
    plt.ylabel('cubic spline abs/ rotation(z)')
    plt.show()
    #
    # plt.plot(elbow_distance)
    # plt.show()


if __name__ == "__main__":
    try:
        script, reference_frame, target_frame = argv
    except Exception as e:
        print "wrong line arguments"
        print "usage : python viewData.py reference_frame target_frame"
        exit()

    # target_frame = "neck"
    # reference_frame = "right"

    dataPos_x, dataPos_y, dataPos_z, dataAng_x, dataAng_y, dataAng_z= loadData(reference_frame, target_frame)
    processPrintData(dataPos_x, dataPos_y, dataPos_z, dataAng_x, dataAng_y, dataAng_z)
