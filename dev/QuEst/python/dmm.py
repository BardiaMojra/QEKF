import sys
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm
import os
import csv


''' matplotlib config '''
matplotlib.pyplot.ion()
plt.style.use('ggplot')

from pdb import set_trace as st
from nbug import *


''' module configs '''
OUT_DIR = '../out00/'
DATA_DIR = 'Datasets/'
_DTYPE = np.float64

class dmm:
  ''' data management module '''
  def __init__(self,
               benchtype,
               benchnum,
               start=0,
               end=None,
               _DTYPE = _DTYPE,
               prt_en=True,
               save_en=True
               ):
    if benchtype == 'KITTI':
      benchnumStr = '%02d'.format(benchnum)
      imgpath = DATA_DIR+'KITTI/sequences/'+benchnumStr+'/image_0'
      datapath = DATA_DIR+'/KITTI/poses/'
    elif  benchtype == 'ICL':
      imgpath =  DATA_DIR+'/ICL/kt'+str(benchnum)+'/rgb/'
      datapath = DATA_DIR+'/ICL/kt'+str(benchnum)
    elif  benchtype == 'NAIST':
      benchnumStr = '%03d'.format(benchnum)
      imgpath =  DATA_DIR+'/NAIST/naist'+benchnumStr
      datapath = DATA_DIR+'/NAIST/'
    elif  benchtype == 'TUM':
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
      imgpath =  DATA_DIR+'/TUM/'+tum_names[benchnum]+'/rgb'
      datapath = DATA_DIR+'/TUM/'+tum_names[benchnum]
    else:
      eprint(lhead+'Err--->> selected dataset not found: '+benchtype+' '+str(benchnum)+ltail)


    ''' check '''
    benchnumStr = '%03d'.format(benchnum)
    outDir = OUT_DIR+benchtype+'_'+benchnumStr+'/'
    if not os.path.exists(outDir):
      print(lhead+'the following directory DOES NOT EXIST: '+outDir)
      print(shead+"create is it with 'mkdir "+outDir+"'/n/n")
      exit()
    st()
    ''' init '''
    self.benchtype = benchtype
    self.imgpath = imgpath
    self.datapath = datapath
    self.outDir = outDir
    self.start = start
    self.end = end
    self.dtype = _DTYPE
    self.prt = prt_en
    self.save = save_en
    # self.df = None
    # self.len = None

    st()

    fnames = get_images(imgpath, benchtype)
    camParams, K = get_calib_matrix(benchtype, datapath, benchnum)
    t1, q1, qTru, tTru = load_gt(benchtype, datapath, benchnum)
    self.fnames = fnames
    self.camParams = camParams
    self.K = K
    self.t1 = t1
    self.q1 = q1
    self.qTru = qTru
    self.tTru = tTru


    # end of __init__() <<--------------------------------------

def get_images(imgpath, benchtype):
  files = os.listdir(imgpath)
  if benchtype ==  'KITTI':
    nprint('files', files)
    st()
    #todo get file names
    # for f in files:
      # fnames.append
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

  nprint('files', files)
  st()
  return files

def get_calib_matrix(benchtype, dataDir, benchnum):
  if benchtype == 'KITTI':
    benchnumStr = '%02d'.format(benchnum)
    fname = dataDir+'KITTI/sequences/'+'calib.txt'
    with open(fname) as f:
      C = f.readline()
      f.close()
    nprint('C', C)
    C.split(' ')

    K = np.array([C[2],  C[3],  C[4],
                  C[6],  C[7],  C[8],
                  C[10], C[11], C[12]], dtype=_DTYPE)
    dp = np.zeros((1,3), dtype=_DTYPE)
    tp = np.zeros((1,2), dtype=_DTYPE)

  elif benchtype == 'ICL':
    K = np.array([481.20,	      0.0,      319.50,
                    0.0,     -480.00,     239.50,
                    0.0,        0.0,        1.0], dtype=_DTYPE)
    K = K.reshape((3,3))
    dp = np.zeros((1,3), dtype=_DTYPE)
    tp = np.zeros((1,2), dtype=_DTYPE)

  elif benchtype == 'NAIST':
    K = np.array([884.9574219,    0.0,        634.0410934,
                    0.0,        883.5555857,  367.8930972,
                    0.0,          0.0,          1.0       ], dtype=_DTYPE)
    K = K.reshape((3,3))
    dp = np.array([-0.2297238599,0.1703723682,-0.09885776185], dtype=_DTYPE)
    tp = np.array([-0.0002223016917,-0.0003623410498], dtype=_DTYPE)

  elif benchtype == 'TUM':
    if benchnum <= 5:
      K = np.array([517.3,    0.0,   318.6,
                      0.0,  516.5,   255.3,
                      0.0,    0.0,     1.0], dtype=_DTYPE)
      K = K.reshape((3,3))
      dp = np.array([0.2624,  -0.9531,  1.1633], dtype=_DTYPE)
      tp = np.array([-0.0054,  0.0026], dtype=_DTYPE)
    elif benchnum == 11:
      K = np.array([535.4,      0.0,     320.1,
                      0.0,    539.2,     247.6,
                      0.0,      0.0,       1.0,], dtype=_DTYPE)
      K = K.reshape((3,3))
      dp = np.array([0.0,   0.0,   0.0], dtype=_DTYPE)
      tp = np.array([0.0,   0.0], dtype=_DTYPE)
    else:
      K = np.array([520.9,     0.0,    325.1,
                      0.0,   521.0,    249.7,
                      0.0,     0.0,      1.0], dtype=_DTYPE)
      dp = np.array([0.2312,   -0.7849,  0.9172], dtype=_DTYPE)
      tp = np.array([-0.0033,   0.0001], dtype=_DTYPE)
  else:
    assert False, 'unknown benchtype '+benchtype

  #todo use opencv with np support
  # camParams = cv.calibrateCamera()
  camParams = {'IntrinsicMatrix': K,
                'NumRadialDistortionCoefficients': 3,
                'RadialDistortion': dp,
                'TangentialDistortion': tp}
  return camParams, K

def load_gt(benchtype, dataDir, benchnum):
  if benchtype == 'KITTI':
    # The KITTI dataset has exactly one pose per image, so there is no need
    # to interpolate poses
    #todo check file
    st()
    fname = dataDir+'/'+benchnum+' '+'.txt'
    rawdata = load(fname)
    tx = rawdata(:,4);
    ty = rawdata(:,8);
    tz = rawdata(:,12);
    Rmats = zeros(3,3,size(rawdata,1));
    for k in rawdata_df.rows:
      size(rawdata,1)
      Rmats(:,:,k) = [rawdata(k,1:3); rawdata(k,5:7); rawdata(k,9:11)];
    qTru = R2Q(Rmats)
    tTru = [tx, ty, tz]
  elif benchtype == 'ICL':
    # rawdata = load([datapath '/traj' num2str(benchnum) '.gt.freiburg']);
    # tTru = rawdata(:,2:4)';
    # qTru = [rawdata(:,8) rawdata(:,5:7)]';
    pass
  elif benchtype == 'NAIST':
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
  elif benchtype == 'TUM':
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
    assert False, 'unknown benchtype '+benchtype


  # Make sure the first quaternion element is always positive
  negIdx = qTru[1,:] < 0; #todo make the test is performed correctly
  qTru(:,negIdx) = - qTru(:,negIdx);

  if benchtype == 'TUM':
    # We need to interpolate pose for TUM dataset
    # Initialize with the first frame
    # fname = fnames{1}
    # ftime = str2double( fname(1:end-4) ) # Time at the current frame
    # [q1, t1] = InterpPoseVer1_1(ftime,times, qTru, tTru) # Interpolate data to find the pose of the current camera frame
    pass
  else:
    # Initial pose
    q1 = qTru[:,1]
    t1 = tTru[:,1]
  return t1, q1, qTru, tTru







# EOF
