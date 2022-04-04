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
matlab_coefs_outPath = '../matlab/quest_5p_ransac/out/KITTI/keypoints/'


''' #todo
# implement ransac
# explore other datasets with kp in frame the whole time
or
# implement the chaining feature
# explore Mask-RCNN for excluding cars and people
'''

''' general config '''
NBUG          = True
_show         = False
_save         = True
_prt          = True
# _START        = 0
# _END          = 150
_ALGORITHMS   = ['QuEst_RANSAC_v0102']
# _ALGORITHMS   = ['QuEst_v0708']
_BENCHTYPE    = 'KITTI'
_BENCHNUM     = 3
skipFrame     = 0 # num of frames that are skiped between two key frames
ORB_THRESHOLD    = 200 # SURF feature point detection threshold
QUEST_MAX_CORRESPS        = 50 # max num of feature points to use for pose est (lower value increases speed)
QUEST_MIN_CORRESPS        = 5 # min num of feature points required (6 to est a unique pose from RANSAC)
RANSAC_MIN_INLIERS   = 5 #todo fix this
RANSAC_MAX_ITER       = 50 #todo fix this
RANSAC_THRESHOLD    = 1.0000e-06

def main():
  global fignum; fignum = int(0)

  dlog = dlm()
  dset = dmm(_BENCHTYPE,
             _BENCHNUM
            #  start=_START,
            #  end=_END,
            )

  numImag = len(dset.fnames) # total number of images
  keyFrames = [i for i in range(skipFrame+1, numImag, skipFrame+1)]

  fdetector = cv.ORB_create()
  fmatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

  # initialize with the first image feature points
  Im_p, kp_p, des_p = GetFeaturePoints(fdetector, 0, dset, ORB_THRESHOLD)

  ''' recover Pose using RANSAC and compare with ground truth '''
  for i in range(1, len(keyFrames)):
    print('\n\n\ -----------===========---------->>>>>: '+str(i)+'/'+str(len(keyFrames)))

    # match features
    Im_n, kp_n, des_n = GetFeaturePoints(fdetector, i, dset, ORB_THRESHOLD)
    if _show:
      imageKeys = cv.drawKeypoints(Im_p,kp_p, None, (255,0,0), 4)
      plt.imshow(imageKeys); plt.show(); cv.waitKey(0)

    matches = fmatcher.match(des_p, des_n)
    matches = sorted(matches, key = lambda x:x.distance)
    print(lhead+'found '+str(len(matches))+' matched correspondences...'+stail)
    matches = matches[:QUEST_MAX_CORRESPS]
    # imageKeys = cv.drawMatches(Im_p,kp_p,Im_n,kp_n,matches,None,flags=4)
    # plt.imshow(imageKeys); plt.show(); cv.waitKey(0); st(); plt.close()
    # nprint('Im_n.shape', Im_n.shape)

    # get ground truth
    qr, tr = RelativeGroundTruth(i, dset)
    # nprint('qr', qr)
    # nprint('tr', tr)

    # In case there are not enough matched points move to the next iteration
    # (This should be checked after 'RelativeGroundTruth')
    if len(matches) < QUEST_MIN_CORRESPS: # skip frame
      Im_p = Im_n; kp_p = kp_n
      print(lhead+'not enough matched feature points. Frame skipped!'+stail)
      continue
    # recover pose and find error by comparing with the ground truth
    for alg in _ALGORITHMS:
      if alg == 'QuEst_RANSAC_v0102':
        matches, m1, m2 = prep_matches(dset, matches, kp_p, kp_n, len(matches))

        rquest = RQUEST(m1, m2,
                        QuEst, get_Txyz,
                        RANSAC_MAX_ITER,
                        RANSAC_THRESHOLD,
                        RANSAC_MIN_INLIERS,
                        nbug=NBUG,
                        return_all=True)
        q = rquest.get_best_fit()

        tOut, dep1, dep2, res = get_Txyz(m1,m2,q)
      elif alg == 'QuEst_v0708':
        matches, m1, m2 = prep_matches(dset, matches, kp_p, kp_n, QUEST_MIN_CORRESPS)
        qs = QuEst(m=m1, n=m2)
        # m1, m2 = load_matlab_kps(i, matlab_coefs_outPath)
        # qs = QuEst(m=m1, n=m2)
        q, q_idx = get_closestQuat(qr,qs)
        tOut, dep1, dep2, res = get_Txyz(m1,m2,q)
      else:
        eprint(str('algorithm is not supported: '+alg))
      # find the closet quaternion and translation to the ground truth
      # t = FindClosetTrans(tr, [tOut,-tOut]);
      t = -tOut
      # st()
      # calcualte the estimate error
      Q_err = phi03_dist(qr, q)
      T_err = get_TransError(tr, t)
      # st()
      dlog.log_state(i,q,qs,qr,t,tr,Q_err,T_err,alg)
      # dlog.prt_log()
      # st()


    # end of for alg ----

    # display images with matched feature points
    # plt.imshow(Im_p); plt.show()
    # npprint('matches.p1', matches.p1)
    # npprint('matches.p2', matches.p2)
    # st()
    # kps1, kps2 = retKPs_pxl(matches)




    # imageKeys = cv.drawMatches(Im_p,kps1,Im_n,kps2,matches,None,flags=4)
    # plt.imshow(imageKeys); plt.show(); cv.waitKey(0); st(); plt.close()
    # Im_match = cv.drawMatches(Im_n, kp_n, Im_p, kp_p, matches, None,\
      # flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    # fignum+=1; write_image(fignum, imageKeys, dmm.outDir)
    # st()

    # store the current image for the next iteration
    #todo: test this, keep reference at t=0, instead of frame by frame
    Im_p = Im_n
    kp_p = kp_n
  print('end of quest data iterator ----->>')

  ''' end processing '''

  ''' print results '''
  dlog.prt_log()
  dlog.prt_stats()



  print('\\------>>>>')
  st()

  return # end of main


if __name__ == "__main__":

  main()
  print('---------   End   ---------')
  exit()

# EOF
