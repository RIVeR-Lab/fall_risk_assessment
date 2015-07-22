import numpy as np
import scipy.signal as sig
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt



# load data
# f = open("fullTest1.csv")
f = open("/media/riverlab-tarik/Backup/fallRiskAssesment_rosbags/full_test/tf_logs/vinayak/vinayak_ep1.csv")
f.readline()
f.readline()
f.readline()
db_raw = np.genfromtxt(f, delimiter=",")

# preprocess data

index = db_raw[:,1]

feetD_x = db_raw[:,7]
feetD_y = db_raw[:,8]
feetD_z = db_raw[:,9]

feetD_x_abs = np.absolute(db_raw[:,7])
feetD_y_abs = np.absolute(db_raw[:,8])
feetD_z_abs = np.absolute(db_raw[:,9])

feetD_z_csp = sig.cspline1d(feetD_z, 10)
# feetD_z_qsp = sig.cubic(feetD_z)
# feetD_z_med = sig.medfilt(feetD_z, kernel_size=13)
# feetD_z_wiener = sig.wiener(feetD_z, mysize=23, noise=0.2)
# feetD_z_savgol = sig.savgol_filter(feetD_z, 5, 2)


peaks = sig.find_peaks_cwt(feetD_z_abs, np.arange(0.2,10))

peaksData = np.zeros(len(peaks))
for i in range(len(peaks)):
    peaksData[i] = feetD_z_abs[peaks[i]]

print "average of step distance: ", np.average(peaksData)
print "# of steps: ", np.shape(peaks)
print "peak locations: ", peaks
# print sig.argrelmin(feet_distance_z)
# print sig.argrelextrema(feet_distance_z. np.less)



# plt.plot(feetD_x)
# plt.plot(feetD_y)
plt.subplot(3, 2, 1)
plt.plot(feetD_z_abs,'g')
plt.ylabel('raw data')
plt.subplot(3, 2, 2)
plt.plot(feetD_z_csp, 'm')
plt.ylabel('cubic spline')
plt.subplot(3, 2, 3)
plt.plot(feetD_z_med)
plt.ylabel('median')
plt.subplot(3, 2, 4)
plt.plot(feetD_z_wiener)
plt.ylabel('wiener')
# plt.subplot(3, 2, 5)
# plt.plot(feetD_z_savgol)
# plt.ylabel('savgol')
plt.show()

# plt.plot(elbow_distance)
# plt.show()
