This folder contains the Matlab implementation of the RANSAC based on QuEst algorithm.

Example.m: Is a demo file that shows how RANSAC based on QuEst can be used to estimate the relative pose between keyframes in the KITTI image dataset (http://www.cvlibs.net/datasets/kitti/eval_odometry.php).


Copyright (C) 2013-2018, under the GNU lesser General Public License, by Kaveh Fathian.


-------------------------------------------------------------
Main algorithms:

QuEst_RANSAC_Ver1_2: QuEst algorithm with RANSAC incorporated.

QuEst_Ver1_1: The implementation of QuEst algorithm that recoveres the rotation, translation, and depths.

Malab scripts are written and tested in Matlab version 2017b. The code preambles further explain the use of each script.


-------------------------------------------------------------
Additional files:


QuEst_5Pt_Ver5_2: Part of QuEst implementation that recoveres the rotation (as explained in the paper "QuEst: A Quaternion-Based Approach for Camera Motion Estimation from Minimal Feature Points").

QuEst_5Pt_Ver7_8: This is a slightly faster implementation of Ver5_2.

QuEst_5Pt_Ver7_8_spd: Speeded up version. This file can be used for automatic Mex/C++ code generation.

FindTransDepth_Ver1_0: Part of QuEst implementation that simultaniously recoveres the translation and depths.

CoefsVer3_1_1: This file generates the coefficients of the degree four polynomial equations from the matched feature points. 

Q2R: Transforms a quaternion to a rotation matrix.

R2Q: Transforms a rotation matrix to quaternion.
