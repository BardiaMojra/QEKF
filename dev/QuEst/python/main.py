''' main - QuEst '''
# import numpy as np
# import matplotlib
# import pandas as pd

''' private libraries '''
from quest import *
from dlm import *
from dmm import *
from utils import *

''' NBUG libraries '''
from nbug import *
from pdb import set_trace as st


''' #todo
# implement ransac
# explore other datasets with kp in frame the whole time
or
# implement the chaining feature
# explore Mask-RCNN for excluding cars and people
'''

''' general config '''
NBUG                = True
_show               = False
_save               = True
_prt                = True

LOAD_MATLAB_MATCHES = True
LOCK_RANSAC         = True

# _START            = 0
# _END              = 150

_ALGORITHMS         = ['QuEst_RANSAC_v0102']
# _ALGORITHMS         = ['QuEst_v0708']
_BENCHTYPE          = 'KITTI'
_BENCHNUM           = 3
skipFrame           = 0 # skipped between two key frames
ORB_THRESHOLD       = 200 # SURF feature point detection threshold
QUEST_MAX_MKP       = 50 # max matched keypoints for pose est
QUEST_NUM_CORRESPS  = 5 # min num correspondences for pose est
RANSAC_MAX_ITER     = 100
RANSAC_THRESHOLD    = 1.0e-2



def main():
  global fignum; fignum = int(0)

  dlog = dlm()
  dset = dmm(_BENCHTYPE,
             _BENCHNUM
            #  start=_START,
            #  end=_END,
            )
  END_FRAME =len(dset.fnames)
  # numImag = len(dset.fnames) # total number of images
  # keyFrames = [i for i in range(skipFrame+1, numImag, skipFrame+1)]

  fdetector = cv.ORB_create()
  fmatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

  # initialize with the first image feature points
  Im_p,kp_p,des_p = GetFeaturePoints(fdetector,0,dset,ORB_THRESHOLD,_show=_show)

  ''' recover Pose using RANSAC and compare with ground truth '''
  for i in range(2,END_FRAME):
    print('\n\n --------===========-------->>>>>: '+str(i)+'/'+str(END_FRAME))

    Im_n,kp_n,des_n = GetFeaturePoints(fdetector,i,dset,ORB_THRESHOLD,_show=_show)
    matches = match_features(fmatcher,des_p,des_n,QUEST_MAX_MKP,_show,Im_p,kp_p,Im_n,kp_n)
    qr, tr = RelativeGroundTruth(i,dset)

    # In case there are not enough matched points move to the next iteration
    # (This should be checked after 'RelativeGroundTruth')
    if len(matches) < QUEST_NUM_CORRESPS: # skip frame
      Im_p = Im_n; kp_p = kp_n
      print(lhead+'not enough matched feature points. Frame skipped!'+stail)
      continue
    # recover pose and find error by comparing with the ground truth
    for alg in _ALGORITHMS:
      if alg == 'QuEst_RANSAC_v0102':
        if NBUG and LOAD_MATLAB_MATCHES:
          matches, dat = load_matlab_matches(i,dset.K)
        else:
          matches, dat = prep_matches(dset,matches,kp_p,kp_n,len(matches))

        rquest = RQUEST(dat,
                        QuEst, get_Txyz,
                        RANSAC_MAX_ITER,
                        RANSAC_THRESHOLD,
                        QUEST_NUM_CORRESPS,
                        nbug=NBUG,
                        return_all=True,
                        lock=LOCK_RANSAC)

        mod,m_idxs,q,tOut,qs = rquest.get_best_fit()

      elif alg == 'QuEst_v0708':
        matches, dat = prep_matches(dset, matches, kp_p, kp_n, QUEST_NUM_CORRESPS)
        qs = QuEst(dat)
        # dat = load_matlab_kps(i)
        # qs = QuEst(m=m1, n=m2)
        q, q_idx = get_closestQuat(qr,qs)
        tOut, dep1, dep2, res = get_Txyz(dat,q)
      else:
        eprint(str('algorithm is not supported: '+alg))
      # find the closet quaternion and translation to the ground truth
      #todo investigate this
      # t = FindClosetTrans(tr, [tOut,-tOut]);
      t = -tOut
      # calcualte the estimate error
      Q_err = phi03_dist(qr, q)
      T_err = get_TransError(tr, t)
      # st()
      dlog.log_state(i,q,qs,qr,t,tr,Q_err,T_err,alg)
    # end of for alg ----

    # store the current image for the next iteration
    #todo: test this, keep reference at t=0, instead of frame by frame
    Im_p = Im_n
    kp_p = kp_n
  print('end of quest data iterator ----->>')

  # dlog.prt_log()
  dlog.prt_stats()
  st()
  return # end of main


if __name__ == "__main__":
  main()
  print('---------   End   ---------')
  exit()

# EOF
