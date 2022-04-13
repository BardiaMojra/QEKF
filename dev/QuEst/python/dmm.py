import sys
import pandas as pd
import numpy as np
import quaternion
import matplotlib
import matplotlib.pyplot as plt
import cv2 as cv
# from matplotlib import cm
import os
# import csv

''' private mods '''
# from quest import *

''' NBUG '''
from pdb import set_trace as st
from nbug import *


''' matplotlib config '''
matplotlib.pyplot.ion()
plt.style.use('ggplot')

''' module configs '''
OUT_DIR = '../pout/'
DATA_ROOT = '../Datasets/'
class dmm:
  ''' data management module '''
  def __init__(self,
               bench,
               benchnum,
               start=0,
               end=None,
               _dtype=np.float64,
               prt_en=True,
               save_en=True
               ):
    if bench == 'KITTI':
      numStr = '{:02d}'.format(benchnum)
      imgpath = DATA_ROOT+'KITTI/sequences/'+numStr+'/image_0/'
      datapath = DATA_ROOT+'KITTI/poses/'
    elif  bench == 'ICL':
      imgpath =  DATA_ROOT+'ICL/kt'+str(benchnum)+'/rgb/'
      datapath = DATA_ROOT+'ICL/kt'+str(benchnum)
    elif  bench == 'NAIST':
      numStr = '{:03d}'.format(benchnum)
      imgpath =  DATA_ROOT+'NAIST/naist'+numStr
      datapath = DATA_ROOT+'NAIST/'
    elif  bench == 'TUM':
      tum_names = ['rgbd_dataset_freiburg1_360',
                   'rgbd_dataset_freiburg1_desk',
                   'rgbd_dataset_freiburg1_desk2',
                   'rgbd_dataset_freiburg1_floor',
                   'rgbd_dataset_freiburg1_room',
                   'rgbd_dataset_freiburg2_360_hemisphere',
                   'rgbd_dataset_freiburg2_360_kidnap',
                   'rgbd_dataset_freiburg2_desk',
                   'rgbd_dataset_freiburg2_large_no_loop',
                   'rgbd_dataset_freiburg2_large_with_loop',
                   'rgbd_dataset_freiburg3_long_office_household']
      imgpath =  DATA_ROOT+'TUM/'+tum_names[benchnum]+'/rgb/'
      datapath = DATA_ROOT+'TUM/'+tum_names[benchnum]
    else:
      eprint(lhead+'Err--->> selected dataset not found: '+bench+' '+str(benchnum)+ltail)


    ''' check '''
    numStr = '{:03d}'.format(benchnum)
    outDir = OUT_DIR+bench+'_'+numStr+'/'
    if not os.path.exists(outDir):
      print(lhead+'DOES NOT EXIST: '+outDir)
      print(shead+"create it with 'mkdir -p "+outDir+"'\n\n")
      exit()
    ''' init '''
    self.bench = bench
    self.imgpath = imgpath
    self.datapath = datapath
    self.outDir = outDir
    self.start = start
    self.end = end
    self.dtype = _dtype
    self.prt = prt_en
    self.save = save_en

    fnames = get_images(imgpath, bench)
    camParams, K, dist = get_calib_matrix(bench, datapath, benchnum)
    t0, q0, T_gt, Q_gt  = load_gt(bench, datapath, benchnum)

    # npprint('t0', t0)
    # nprint('q0', q0)
    # npprint('Q_gt[:5]', Q_gt[:5])
    # npprint('T_gt[:5]', T_gt[:5])
    # st()

    self.fnames = fnames
    self.camParams = camParams
    self.K = K
    self.dist = dist
    self.t0 = t0
    self.q0 = q0
    self.Q_gt = Q_gt
    self.T_gt = T_gt

    # end of __init__() <<--------------------------------------

def get_images(imgpath, benchtype):
  files = os.listdir(imgpath)
  files = sorted(files)
  if benchtype ==  'KITTI':
    pass
  elif benchtype ==  'NAIST':
    nprint('files', files)
    st()
    #todo get file names
    # for f in files:
      # fnames.append
  elif benchtype ==  'ICL':
    nprint('files', files)
    st()
    #todo get file names
    # for f in files:
      # fnames.append
  elif benchtype ==  'TUM':
    nprint('files', files)
    st()
    #todo get file names
    # for f in files:
      # fnames.append
  else:
    assert False, 'unknown benchtype '+benchtype

  # nprint('files', files)
  # st()
  return files

def get_calib_matrix(benchtype, dataDir, benchnum, _dtype=np.float64):
  if benchtype == 'KITTI':
    # numStr = '{:02d}'.format(benchnum)
    # fname = DATA_ROOT+'KITTI/sequences/'+numStr+'/calib.txt'
    # with open(fname) as f:
    #   C = f.readline()
    #   f.close()
    # nprint('C', C)
    # st()
    # C.split(' ')
    K = np.array([7.215377000000e+02, 0.000000000000e+00, 6.095593000000e+02,
                  0.000000000000e+00, 7.215377000000e+02, 1.728540000000e+02,
                  0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00],
                  dtype=_dtype).reshape(3,3)

    ''' on radial distortion, tangential distortion, and distortion vector:
    https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#ga69f2545a8b62a6b0fc2ee060dc30559d
    http://researchspace.csir.co.za/dspace/bitstream/handle/10204/3168/De%20Villiers_2008.pdf;jsessionid=5F6421340C2DB3733478D1A1E00DD050?sequence=1
    '''
    dp = np.zeros((1,3), dtype=_dtype) # radDist k
    tp = np.zeros((1,2), dtype=_dtype) # tanDist p
    dist = np.zeros((1,5), dtype=_dtype) # dist = k[0],k[1],p[0],p[1],k[2]
    # nprint('dist', dist)
    # st()
  elif benchtype == 'ICL':
    K = np.array([481.20,	      0.0,      319.50,
                    0.0,     -480.00,     239.50,
                    0.0,        0.0,        1.0], dtype=_dtype).reshape(3,3)
    dp = np.zeros((1,3), dtype=_dtype)
    tp = np.zeros((1,2), dtype=_dtype)
    dist = np.zeros((1,5), dtype=_dtype) # dist = k[0],k[1],p[0],p[1],k[2]
  elif benchtype == 'NAIST':
    K = np.array([884.9574219,    0.0,        634.0410934,
                    0.0,        883.5555857,  367.8930972,
                    0.0,          0.0,          1.0       ], dtype=_dtype)
    K = K.reshape((3,3))
    dp = np.array([-0.2297238599,0.1703723682,-0.09885776185], dtype=_dtype)
    tp = np.array([-0.0002223016917,-0.0003623410498], dtype=_dtype)

  elif benchtype == 'TUM':
    if benchnum <= 5:
      K = np.array([517.3,    0.0,   318.6,
                      0.0,  516.5,   255.3,
                      0.0,    0.0,     1.0], dtype=_dtype)
      K = K.reshape((3,3))
      dp = np.array([0.2624,  -0.9531,  1.1633], dtype=_dtype)
      tp = np.array([-0.0054,  0.0026], dtype=_dtype)
    elif benchnum == 11:
      K = np.array([535.4,      0.0,     320.1,
                      0.0,    539.2,     247.6,
                      0.0,      0.0,       1.0,], dtype=_dtype)
      K = K.reshape((3,3))
      dp = np.array([0.0,   0.0,   0.0], dtype=_dtype)
      tp = np.array([0.0,   0.0], dtype=_dtype)
    else:
      K = np.array([520.9,     0.0,    325.1,
                      0.0,   521.0,    249.7,
                      0.0,     0.0,      1.0], dtype=_dtype)
      dp = np.array([0.2312,   -0.7849,  0.9172], dtype=_dtype)
      tp = np.array([-0.0033,   0.0001], dtype=_dtype)
  else:
    assert False, 'unknown benchtype '+benchtype

  #todo use opencv with np support
  # camParams = cv.calibrateCamera()
  camParams = {'IntrinsicMatrix': K,
                'NumRadialDistortionCoefficients': 3,
                'RadialDistortion': dp,
                'TangentialDistortion': tp}
  return camParams, K, dist

def load_gt(bench, dataDir, benchnum, _dtype=np.float64):
  if bench == 'KITTI':
    numStr = '{:02d}'.format(benchnum)
    fname = dataDir+numStr+'.txt'
    dat = np.loadtxt(fname, dtype=_dtype)
    Tx = dat[:,3].copy()
    Ty = dat[:,7].copy()
    Tz = dat[:,11].copy()
    Tx = Tx.reshape(-1,1)
    Ty = Ty.reshape(-1,1)
    Tz = Tz.reshape(-1,1)
    T_gt = np.concatenate((Tx,Ty,Tz), axis=1)
    Rmats = np.zeros((dat.shape[0],3,3), dtype=_dtype);
    for i in range(dat.shape[0]):
      R = np.concatenate((dat[i,0:3],dat[i,4:7],dat[i,8:11]), axis=0)
      R = R.reshape(3,3)
      Rmats[i,:] = R
    Q_gt = quaternion.from_rotation_matrix(Rmats)
    # Q_gt = Q_gt.reshape(-1,1)
    # nsprint('Q_gt', Q_gt)
    # st()
  elif bench == 'ICL':
    # rawdata = load([datapath '/traj' num2str(benchnum) '.gt.freiburg']);
    # tTru = rawdata(:,2:4)';
    # qTru = [rawdata(:,8) rawdata(:,5:7)]';
    pass
  elif bench == 'NAIST':
    # datafile = [datapath '/naist' num2str(benchnum, '%03d') '_camerapath.csv'];
    # fileid = fopen(datafile);
    # C = textscan(fileid, '%s %f %f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ',', 'CommentStyle', '#');
    # fclose(fileid);
    # tx = C{5};
    # ty = C{9};
    # tz = C{13};
    # Rmats = zeros(3,3,length(C{1}));
    # for k=1:length(C{1})
    #   Rmats(:,:,k) = [C{2}(k)  C{3}(k)  C{4}(k);
    #                   C{6}(k)  C{7}(k)  C{8}(k);
    #                   C{10}(k) C{11}(k) C{12}(k)];
    # qTru = R2Q(Rmats);
    # tTru = [tx, ty, tz].';
    pass
  elif bench == 'TUM':
    # Using textscan instead of textread to deal with comment lines,
    # instead of editing every groundtruth.txt file
    # datafile = [datapath '/groundtruth.txt'];
    # fileid = fopen(datafile);
    # C = textscan(fileid, '%f %f %f %f %f %f %f %f', 'CommentStyle', '#');
    # fclose(fileid);
    # times = C{1};
    # tx = C{2};
    # ty = C{3};
    # tz = C{4};
    # qx = C{5};
    # qy = C{6};
    # qz = C{7};
    # qw = C{8};  # NOTE: In this dataset, instead of first, the last element is cosine of half rotation angle!
    # qTru = [qw, qx, qy, qz] # Ground truth quaternions in world coord frame
    # tTru = [tx, ty, tz]     # Ground truth translations in world coord frame
    # self.times = times;
    pass
  else:
    assert False, 'unknown benchtype '+bench

  # Make sure the first quaternion element is always positive
  for i, q in enumerate(Q_gt):
    if q.w < 0: Q_gt[i] = - Q_gt[i]

  if bench == 'TUM':
    # We need to interpolate pose for TUM dataset
    # Initialize with the first frame
    # fname = fnames{1}
    # ftime = str2double( fname(1:end-4) ) # Time at the current frame
    # [q1, t1] = InterpPoseVer1_1(ftime,times, qTru, tTru) # Interpolate data to find the pose of the current camera frame
    pass

  return T_gt[0], Q_gt[0], T_gt, Q_gt



# EOF
