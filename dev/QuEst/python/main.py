# import numpy as np
# import matplotlib
import pandas as pd

''' private libraries '''
from quest import *
from dlm import *
from dmm import *
from utils import *

''' NBUG libraries '''
from nbug import *
from pdb import set_trace as st
matlab_coefs_outPath = '../matlab/quest_5p_ransac/out/KITTI/keypoints/'

''' general config '''
_show         = True
_save         = True
_prt          = True
# _START        = 0
# _END          = 150
_ALGORITHMS   = ['QuEst_v0708','QuEst']
_BENCHTYPE    = 'KITTI'
_BENCHNUM     = 3
skipFrame     = 0 # num of frames that are skiped between two key frames
ranThresh     = 1e-6 # RANSAC outlier threshold
surfThresh    = 200 # SURF feature point detection threshold
maxPts        = 30 # max num of feature points to use for pose est (lower value increases speed)
minPts        = 5 # min num of feature points required (6 to est a unique pose from RANSAC)

def main():
  global fignum; fignum = int(0)

  dlog = dlm(False)
  dset = dmm(_BENCHTYPE,
             _BENCHNUM
            #  start=_START,
            #  end=_END,
            )

  numImag = len(dset.fnames) # total number of images
  keyFrames = [i for i in range(skipFrame+1, numImag, skipFrame+1)]

  # numKeyFrames = len(keyFrames) # num of key frames
  # numMethods   = len(_ALGORITHMS) # num of algorithms used in the comparison
  # rotErr      = NaN(numKeyFrames, numMethods) # rotation error for each method
  # tranErr     = NaN(numKeyFrames, numMethods) # translation error for each method
  # Q           = cell(numKeyFrames, numMethods) # recovered quaternions
  # T           = cell(numKeyFrames, numMethods) # recovered translations

  fdetector = cv.ORB_create()
  fmatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

  # initialize with the first image feature points
  Im_p, kp_p, des_p = GetFeaturePoints(fdetector, 0, dset, surfThresh)

  ''' recover Pose using RANSAC and compare with ground truth '''
  for i in range(1, len(keyFrames)):
    print('\ ------->>>>>: '+str(i)+'/'+str(len(keyFrames)))

    # match features
    Im_n, kp_n, des_n = GetFeaturePoints(fdetector, i, dset, surfThresh)
    # imageKeys = cv.drawKeypoints(Im_p,kp_p, None, (255,0,0), 4)
    # plt.imshow(imageKeys); plt.show(); cv.waitKey(0); st(); plt.close()

    matches = fmatcher.match(des_p, des_n)
    matches = sorted(matches, key = lambda x:x.distance)
    print(lhead+'found '+str(len(matches))+' matched correspondences...'+stail)
    matches = matches[:maxPts]
    # imageKeys = cv.drawMatches(Im_p,kp_p,Im_n,kp_n,matches,None,flags=4)
    # plt.imshow(imageKeys); plt.show(); cv.waitKey(0); st(); plt.close()
    # nprint('Im_n.shape', Im_n.shape)

    # get ground truth
    qr, tr = RelativeGroundTruth(i, dset)
    # nprint('qr', qr)
    # nprint('tr', tr)

    # In case there are not enough matched points move to the next iteration
    # (This should be checked after 'RelativeGroundTruth')
    if len(matches) < minPts: # skip frame
      Im_p = Im_n; kp_p = kp_n
      print(lhead+'not enough matched feature points. Frame skipped!'+stail)
      continue
    # recover pose and find error by comparing with the ground truth
    for alg in _ALGORITHMS:
      if alg == 'QuEst_RANSAC_v0102':
        M, inliers = QRANSAC0102(matches, ranThresh)
        Qs = M.Q
        tOut = M.t
      elif alg == 'QuEst_v0708':
        # matches = prep_matches(dset, matches, kp_p, kp_n, minPts)
        # tOut, q = Q0708(m=matches.m1, n=matches.m2)
        kp1, kp2 = load_matlab_kps(i, matlab_coefs_outPath)
        Qs = QuEst(m=kp1, n=kp2)
        st()

        q, q_idx = get_closestQuat(qr, Qs)
        tOut = get_Txyz(matches.m1, matches.m2, q)
      else:
        eprint(str('algorithm is not supported: '+alg))

      # find the closet quaternion and translation to the ground truth
      t = FindClosetTrans(tr, [tOut,-tOut]);
      t = -tOut
      st()
      # calcualte the estimate error
      Q_err = get_QuatError(qr, q)
      T_err = get_TransError(tr, t)

      dlog.log_data(i, alg, 'q', Qs)
      dlog.log_data(i, alg, 'qr', qr)
      dlog.log_data(i, alg, 't', t)
      dlog.log_data(i, alg, 'tr', tr)
      dlog.log_data(i, alg, 'Q_err', Q_err)
      dlog.log_data(i, alg, 'T_err', T_err)

    # end of for alg

    # display images with matched feature points
    plt.imshow(Im_p); plt.show()
    Im_match = cv.drawMatches(Im_n, kp_n, Im_p, kp_p, matches, None,\
      flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(Im_match); plt.show()
    fignum+=1;
    write_image(fignum, Im_match, dmm.outDir)


    # store the current image for the next iteration
    Im_p = Im_n
    kp_p = kp_n
  print('end of quest data iterator ----->>')

  ''' end processing '''


  # print results
  # Remove NaN entries (corresponding to skipped frames)
  # nanIdx = find( sum(isnan(rotErr),2) );
  # rotErr(nanIdx,:) = [];
  # tranErr(nanIdx,:) = [];
  # numKeyFrames = size(rotErr,1);

  # statistics of error for rotation and translation estimates
  # rotErrM = mean(rotErr,1);            % Mean
  # rotErrS = std(rotErr,1);             % Standard deviation
  # rotErrMd = median(rotErr,1);         % Median
  # rotErrQ1 = quantile(rotErr,0.25, 1); % 25% quartile
  # rotErrQ3 = quantile(rotErr,0.75, 1); % 75% quartile
  # tranErrM = mean(tranErr,1);
  # tranErrS = std(tranErr,1);
  # tranErrMd = median(tranErr,1);
  # tranErrQ1 = quantile(tranErr,0.25, 1);
  # tranErrQ3 = quantile(tranErr,0.75, 1);

  # Table of results
  # RowNames = ['Rot err mean', 'Rot err std', 'Rot err median',
  #             'Rot err Q_1', 'Rot err Q_3',
              # # 'Tran err mean', 'Tran err std', 'Tran err median',
  #             # 'Tran err Q_1', 'Tran err Q_3']
  # # data = [rotErrM, rotErrS, rotErrMd,
  #         # rotErrQ1, rotErrQ3,
          # # tranErrM, tranErrS, tranErrMd,
  #         # tranErrQ1, tranErrQ3]
  # # table(data(:,1),'RowNames',RowNames, 'VariableNames', algorithms)


  return # end of main


if __name__ == "__main__":

  main()
  print('---------   End   ---------')
  exit()

# EOF
