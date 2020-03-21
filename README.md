# GSE-Tracking
Online complementary materials of the paper titled 

**ROPHS: Determine Real-Time Status of a Multi-Carriage Logistics Train at Airport**

By Shixiong Wang, Chongshou Li, and Andrew Lim

From Department of Industrial Systems Engineering and Management, National University of Singapore, Singapore117576. 

Project Website: <https://alim.algorithmexchange.com/caas/>



***MATLAB Version: 2016A or later***

[1] The folder "References" contains reference literature (where mathematical details are given) for understanding the source code

[2] The folder "Code - Trajectory" contains the code to generate Figure 1 in the paper
- The file "Trajectory.m" contains source code to parse GPGGA sentence obtained from RTK positioning board;
- The file "RTK-Raw-Data.txt" is the raw data collected at 21:16 on Jan 15, 2019 at National University of Singapore. This data file is used by "Trajectory.m".

[3] The folder "Code - Uncertainty Region" contains the code to generate Figure 2 in the paper
- The file "main.m" is the entrance of the code. It calls "R.m" and "drawTrain.m". This file simulate the uncertainty region of the corner point on the last trailing vehicle;
- The file "drawTrain.m" is used to plot the diagram of the train;
- The file "R.m" defeins a temporary function (i.e., the "**Q**" Eq. (6)).

[4] The folder "Code - Tracking Algorithm" contains the code to generate Figure 3 in the paper
- The file "RawData.mat" are collected data (a part of trajectory of APM depolyed on the leading vehicle) when doing filed test at National University of Singapore on Feb 14, 2020. It includes three variable:
  + "X_Real": Real status of the leading vehicle, i.e., true value of "X^t" in Eq. (2) and (3). This variable is 4-dimensional. The first entry is the true position in x-axis; the second is the true velocity in x-axis; the third is the true position in y-axis; the fourth is the true velocity in y-axis; 
  + "Y_Mear": Noised measurements of the positions of the leading vehicle, i.e., "Y^t" in Eq. (2) and (3). This variable is 2-dimensional. The first entry is the measured position in x-axis, while the second is the measured position in y-axis;
  + "Ts"    : Sampling period in seconds. In our experiment, we use Ts = 0.1s, meeaning the update frequency of GPGGA sentence is 10 per second (i.e., we obtain 10 observations per second of the real-time position of the APM).
- The file "main.m" is the entrance of the code. It calls other ".m" files. In this file, the geometry based positioing algorithm (Eq. (1)) and the IMM-KF tracking algorithm (Eqs. (2) and (3)) are implemented. The mathmatical details of IMM-KF could be found in "References/IMM.pdf" or in the paper;
- The file "Kalman.m" defines the canonical Kalman Filtering for IMM-KF;
- The file "Model_Mix.m" and "Model_P_Update.m" defines model mixture operation and model probability update operation, respectively. The mathmatical details could be found in "References/IMM.pdf".
