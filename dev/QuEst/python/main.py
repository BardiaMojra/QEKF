# import numpy as np
# import matplotlib
import pandas as pd

''' private libraries '''
from dmm import *
from utils import *

''' NBUG libraries '''
from pdb import set_trace as st
from nbug import *

''' general config '''
NBUG          = True
_show         = True
_save         = True
_prt          = True
# _START        = 0
# _END          = 150
_ALGORITHMS    = ['QuEst']
_BENCHTYPE    = 'KITTI'
_BENCHNUM     = 3

skipFrame     = 0 # num of frames that are skiped between two key frames
ranThresh     = 1e-6 # RANSAC outlier threshold
surfThresh    = 200 # SURF feature point detection threshold

# Number of feature points
maxPts        = 30 # max num of feature points to use for pose est (lower value increases speed)
minPts        = 6 # min num of feature points required (6 to est a unique pose from RANSAC)

def main():
  global fignum; fignum = int(0)

  # init dataset object
  dset = dmm(_BENCHTYPE,
             _BENCHNUM
            #  start=_START,
            #  end=_END,
            )

  numImag = len(dset.fnames) # total number of images
  i = 1+skipFrame
  keyFrames = list()
  while i < numImag:
    keyFrames.append(i)
    i = i+1+skipFrame

  nprint('keyFrames', keyFrames)
  st()

  numKeyFrames = len(keyFrames) # num of key frames
  numMethods   = len(_ALGORITHMS) # num of algorithms used in the comparison
  # rotErr      = NaN(numKeyFrames, numMethods) # rotation error for each method
  # tranErr     = NaN(numKeyFrames, numMethods) # translation error for each method
  # Q           = cell(numKeyFrames, numMethods) # recovered quaternions
  # T           = cell(numKeyFrames, numMethods) # recovered translations

  fdetector = cv.ORB_create()
  fmatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

  # initialize with the first image feature points
  Im_p, kp_p, des_p = GetFeaturePoints(fdetector, 0, dset, surfThresh)

  ''' recover Pose using RANSAC and compare with ground truth '''

  st()
  cntr = 0 # keyframe counter
  for i in len(keyFrames):
    cntr =+ 1

    # match features
    Im_n, kp_n, des_n = GetFeaturePoints(fdetector, i, dset, surfThresh)
    matches = fmatcher.match(des_p, des_n)
    matches = sorted(matches, key = lambda x:x.distance)
    matches = matches[:maxPts]
    matches = MatchFeaturePoints(Im_p, kp_p, Im_n, kp_n, maxPts, dset)

    # get ground truth
    relPose, posp = RelativeGroundTruth(i, posp, dset)

    # In case there are not enough matched points move to the next iteration
    # (This should be checked after 'RelativeGroundTruth')
    if matches.numPts < minPts:
      # use current image for the next iteration
      Im_p = Im_n
      kp_p = kp_n
      print(lhead+'not enough matched feature points. Frame skipped!'+stail)
      continue

    eprint(str('algorithm is not supported: '+alg))
    # recover pose and find error by comparing with the ground truth
    for alg in _ALGORITHMS:
      if alg == 'QuEst':
        M, inliers = QuEst_RANSAC_v01_2(matches.m1, matches.m2, ranThresh)
        q = M.Q
        tOut = M.t
      else:
        eprint(str('algorithm is not supported: '+alg))

      # find the closet quaternion and translation to the ground truth
      # [q, matchIdx] = FindClosetQVer2_2(relPose.qr, q);
      # t = FindClosetTrans(relPose.tr, [tOut,-tOut]);
      t = -tOut

      # calcualte the estimate error

        % Calculate the estimation error
        rotErr[cntr,alg]  = get_QuatError(relPose.qr, q);
        tranErr[cntr,alg] = get_TransError(relPose.tr, t);

    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Display images with matched feature points
    imshow(Im_p);
    hold on;
    p1 = matches.p1;
    p2 = matches.p2;
    numPts = size(p1,2);
    plot(p1(:,1), p1(:,2), 'g+');
    plot(p2(:,1), p2(:,2), 'yo');
    for j = 1 : numPts
        if any(j == inliers)
            plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'g'); % Detected inlier
        else
            plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'r'); % Detected outlier
        end
    end
    drawnow;
    hold off;

    % Store the current image for the next iteration
    Ip = In;
    ppoints = npoints;
    % Print iteration number
    if mod(cntr,10) == 0
        display(['Iteration ' num2str(cntr) ' of ' num2str(numKeyFrames)]);
    end

end



%% Display results

% Remove NaN entries (corresponding to skipped frames)
nanIdx = find( sum(isnan(rotErr),2) );
rotErr(nanIdx,:) = [];
tranErr(nanIdx,:) = [];
numKeyFrames = size(rotErr,1);

% Statistics of error for rotation and translation estimates
rotErrM = mean(rotErr,1);            % Mean
rotErrS = std(rotErr,1);             % Standard deviation
rotErrMd = median(rotErr,1);         % Median
rotErrQ1 = quantile(rotErr,0.25, 1); % 25% quartile
rotErrQ3 = quantile(rotErr,0.75, 1); % 75% quartile
tranErrM = mean(tranErr,1);
tranErrS = std(tranErr,1);
tranErrMd = median(tranErr,1);
tranErrQ1 = quantile(tranErr,0.25, 1);
tranErrQ3 = quantile(tranErr,0.75, 1);

% Table of results
RowNames = {'Rot err mean'; 'Rot err std'; 'Rot err median'; 'Rot err Q_1'; 'Rot err Q_3';...
    'Tran err mean'; 'Tran err std'; 'Tran err median'; 'Tran err Q_1'; 'Tran err Q_3'};
data = [rotErrM; rotErrS; rotErrMd; rotErrQ1; rotErrQ3; ...
    tranErrM; tranErrS; tranErrMd; tranErrQ1; tranErrQ3;];
table(data(:,1),'RowNames',RowNames, 'VariableNames', algorithms)



%%

warning('on','all');



  # init QEKF object
  # quest = QuEst()

  # x_Txyz = dset.t1  # init state vectors
  # x_Qxyz = dset.q1  # init state vectors
  # for i in range(dset.start, dset.end):
  #   # print('    \\--->>> new state ------->>>>>:  ', i)
  #   u_Wrpy = dset.u_Wrpy_np[i].reshape(-1,1)
  #   z_TVQxyz = dset.z_TVQxyzw_np[i,:-1].reshape(-1,1)
  #   z_TVQxyzw = dset.z_TVQxyzw_np[i].reshape(-1,1) # only for data logging
  #   # nsprint('x_TVQxyz', x_TVQxyz)
  #   # nsprint('u_Wrpy', u_Wrpy)
  #   # nsprint('z_TVQxyz', z_TVQxyz)
  #   # st()
  #   x_TVQxyz = qekf.predict(x_TVQxyz, u_Wrpy)
  #   # nsprint('x_TVQxyz', x_TVQxyz)
  #   x_TVQxyz = qekf.update(x_TVQxyz, z_TVQxyz)
  #   # st()
  #   ''' log z state vector '''
  #   qekf.log.log_z_state(z_TVQxyzw, i)
  print('end of qekf data iterator ----->>')


  ''' post processing '''

  return # end of main


if __name__ == "__main__":

  main()
  print('---------   End   ---------')
  exit()

# EOF
