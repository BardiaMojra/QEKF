from util import exp_map
import sys
import pandas as pd
from pdb import set_trace as st
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm
import os
from pprint import pprint as pp
from nbug import *
from scipy.spatial.transform import Rotation as R


''' matplotlib config '''
matplotlib.pyplot.ion()
plt.style.use('ggplot')

    # config
qlabels = ['idx', 'Tx', 'Ty', 'Tz', 'qx', 'qy', 'qz', 'qw']
vlabels = ['idx2', 'vx', 'vy', 'vz', 'wr', 'wp', 'wy']
datasets = [  'dataset-iphone1_clean',
              'bigC_06-Aug2021',
              'kitti_imu_0926_0001',
              'kitti_imu_0926_0002',
              'kitti_imu_0926_0005',
              'kitti_imu_0926_0018',
              'kitti_imu_0926_0060',
              'kitti_imu_0926_0084',
              'kitti_imu_0926_0113',
              'kitti_imu_0928_0001',
              'Y2021M08D05_zoom-twist-jackal_BigC-off_ransac-off',
              'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off',
              'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6',
              'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6',
              'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off',
]
class dmm:
  ''' Data Management Module
  '''
  def __init__(self,
               name,
               VestScale=1.0,
               data_rate_inv=1/30,
               start=0,
               end=None,
               prt=True,
               save=True):

    # set dataset configs
    if name == datasets[0]:
      src_dir = '../data/dataset-iphone1_clean/'
      output_dir = '../out/out_'+name+'/'
      ext = 'xlsx'
      opt = None
    elif name == datasets[1]:
      src_dir = '../data/bigC_06-Aug2021/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = ' ' # space separator for csv file
    elif name ==  datasets[2]:
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0001_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[3]:
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0002_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[4]:
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0005_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[5]:
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0018_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[6]:
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0060_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[7]:
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0084_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[8]:
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0113_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[9]:
      src_dir = '../data/KITTI/2011_09_28/2011_09_28_drive_0001_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
      data_rate_inv = 0.1
      print(longhead+' changed data_rate_inv to: '+str(data_rate_inv))
    elif name == datasets[10]:
      src_dir = '../data/'+name+'/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = ' ' # space separator for csv file
    elif name == datasets[11]:
      src_dir = '../data/'+name+'/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = ' ' # space separator for csv file
    elif name == datasets[12]:
      src_dir = '../data/'+name+'/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = ' ' # space separator for csv file
    elif name == datasets[13]:
      src_dir = '../data/'+name+'/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = ' ' # space separator for csv file
    elif name == datasets[14]:
      src_dir = '../data/'+name+'/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = ' ' # space separator for csv file
    else:
      eprint(longhead+'Err--->> selected dataset not found: '+name+longtail)

    ''' check '''
    if not os.path.exists(output_dir):
      print(longhead+'the following directory DOES NOT EXIST: '+output_dir)
      print(shorthead+"create is it with 'mkdir "+output_dir+"'\n\n")
      exit()

    ''' init '''
    # data
    self.name = name
    self.src_dir = src_dir
    self.ext = ext
    self.dtype = 'float64'
    self.options = opt
    self.prt = prt
    self.save = save
    self.output_dir = output_dir
    # data config
    self.VestScale = VestScale
    self.data_rate_inv = data_rate_inv
    self.start = start
    self.end = end
    # df init
    self.df = None
    self.len = None
    self.qlabels = qlabels
    self.vlabels = vlabels
    self.labels = qlabels + vlabels
    self.quest = None
    self.vest = None
    self.trans_xyz = None
    self.vel_xyz = None
    self.acc_xyz = None
    self.quat_xyzw = None # rotation in quaternion - x,y,z,w
    self.quat_wxyz = None # rotation in quaternion - w,x,y,z
    self.vel_rpy = None # rad/sec - roll, pitch, yaw
    self.acc_rpy = None # rad/sec^2 - roll, pitch, yaw
    # end of __init__() <<--------------------------------------

  def format(self):
    ''' data files:
      iphone_mouse_zoom_2:
      dataset-iphone1 (_clean):
        quest.xlsx
        vest.xlsx
      bigC:
        quest.xlsx
        vest.xlsx
      kitti_imu_001:
        data/KITTI/
        └── 2011_09_26
            ├── 2011_09_26_drive_0001_sync
            │   ├── image_00
            │   ├── image_01
            │   ├── image_02
            │   ├── image_03
            │   ├── oxts <<---------------------  IMU data
            │   ├── tracklet_labels.xml
            │   └── velodyne_points
      Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off
      Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6
    '''
    if self.name=="iphone_mouse_zoom_2" and self.ext=='csv':
      fname = 'u_cord_subpix.csv'
      self.df = pd.read_csv(self.src_dir+fname)
    elif self.name=="dataset-iphone1_clean" and self.ext=='xlsx':
      # load QuEst data
      fname = 'quest.xlsx'
      self.quest = pd.read_excel(self.src_dir+fname, engine='openpyxl',\
        index_col=0, dtype=self.dtype, header=0)
      # load VEst data
      fname = 'vest.xlsx'
      self.vest = pd.read_excel(self.src_dir+fname, engine='openpyxl',\
        index_col=0, dtype=self.dtype, header=0)
      # load data frame
      self.df = pd.concat([self.quest, self.vest], axis=1)
      self.labels = self.df.columns# load state variable labels
    elif self.name=="bigC_06-Aug2021" and self.ext=='csv':
      self.load_QuVest_set()
    elif self.name == 'Y2021M08D05_zoom-twist-jackal_BigC-off_ransac-off'\
      and self.ext=='csv':
      self.load_QuVest_set()
    elif self.name == 'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off'\
      and self.ext=='csv':
      self.load_QuVest_set()
    elif self.name == 'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6'\
      and self.ext=='csv':
      self.load_QuVest_set()
    elif self.name == 'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6'\
      and self.ext=='csv':
      self.load_QuVest_set()
    elif self.name == 'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off'\
      and self.ext=='csv':
      self.load_QuVest_set()
    elif self.name=="kitti_imu_0926_0001" and self.ext=='csv':
        self.load_kitti_set()
    elif self.name=="kitti_imu_0926_0002" and self.ext=='csv':
      self.load_kitti_set()
    elif self.name=="kitti_imu_0926_0005" and self.ext=='csv':
      self.load_kitti_set()
    elif self.name=="kitti_imu_0926_0018" and self.ext=='csv':
      self.load_kitti_set()
    elif self.name=="kitti_imu_0926_0060" and self.ext=='csv':
      self.load_kitti_set()
    elif self.name=="kitti_imu_0926_0084" and self.ext=='csv':
      self.load_kitti_set()
    elif self.name=="kitti_imu_0926_0113" and self.ext=='csv':
      self.load_kitti_set()
    elif self.name=="kitti_imu_0928_0001" and self.ext=='csv':
      self.load_kitti_set()
    else:
      eprint(longhead+'Err--->> invalid name and/or ext!\n\n', file=sys.stderr)
      exit()
    ''' common df section '''
    # load state variables
    self.trans_xyz = self.df[['Tx', 'Ty', 'Tz']]
    self.quat_xyzw = self.df[['qx', 'qy', 'qz', 'qw']]
    self.quat_wxyz = self.df[['qw', 'qx', 'qy', 'qz']]
    self.vel_xyz = self.df[['vx', 'vy', 'vz']]
    self.vel_rpy = self.df[['wr', 'wp', 'wy']]
    self.len = len(self.df.index)
    if self.end == None:
      self.end = self.len
    # save dataset to output dir
    if self.prt == True:
      self.df.to_csv(self.output_dir+self.name+'_df.csv', columns=self.df.columns)
    return

  def get(self, quat_format=None):
    if quat_format=='xyzw':
      print(longhead+'using xyzw quaternion rotation notation...')
      return self.trans_xyz, self.vel_xyz, self.quat_xyzw, self.vel_rpy
    else:
      print(longhead+'using wxyz quaternion rotation notation...')
      return self.trans_xyz, self.vel_xyz, self.quat_wxyz, self.vel_rpy

  def plot(self, labels, show=True, save=True, figname='fig_01', title='_'):
    df = self.df[list(labels)] # plot mentioned columns (labels)
    #fig01 = plt.figure()
    df.plot()
    plt.legend(loc='best')
    if title != '_':
      plt.title('{}'.format(title))
      title = title.replace(' ', '_')
    plt.xlabel('Time')
    plt.ylabel('Magnitute')
    # save and show image utility
    if save==True and self.output_dir is not None:
      file_name = self.output_dir+'{}'.format(figname+'_'+title)
      plt.savefig(file_name, bbox_inches='tight',dpi=400)
      print(shorthead+'saving figure: '+file_name+'.png')
    if show==True:
      plt.show()
    else:
      plt.close()
    return

  def plot_trans_3d(self, show=True, save=True, figname='fig_02', title='_'):
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter3D(self.df['Tx'].iloc[0], self.df['Ty'].iloc[0], self.df['Tz'].iloc[0], s=200, marker='*',c='y', label='start')
    ax.scatter3D(self.df.Tx, self.df.Ty, self.df.Tz, c='g', marker='.', label='translation');
    ax.scatter3D(self.df['Tx'].iloc[-1], self.df['Ty'].iloc[-1], self.df['Tz'].iloc[-1], s=200, marker='*',c='k', label='end')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(loc='best')
    ax = set_axes_equal(ax)
    if title != '_': # add title
      plt.title('{}'.format(title))
      title = title.replace(' ', '_')
    # save and show image utility
    if save==True and self.output_dir is not None:
      file_name = self.output_dir+'{}'.format(figname+'_'+title)
      plt.savefig(file_name, bbox_inches='tight',dpi=400)
      print(shorthead+'saving figure: '+file_name+'.png')
    if show==True:
      plt.show()
    else:
      plt.close()
    return

  def load_kitti_set(self):
    '''
      /data/KITTI/2011_09_26/2011_09_26_drive_0001_sync/oxts/.
        ├── data/ <<< each datum is in separate file
        ├── dataformat.txt
        └── timestamps.txt
    '''
    # get time stamp
    times_dir = self.src_dir+'timestamps.txt'
    times = list()
    periods = list()
    with open(times_dir) as file:
      for i, line in enumerate(file):
        date, time = line.split(sep=' ', maxsplit=1)
        time = time.replace('\n','')
        _, __, time = time.split(':')
        time = float(time)
        if i == 0:
          time_zero = time
          time = 0.0
          period = 0.0
        else:
          time = time - time_zero
          period = time - times[-1]
        times.append(time)
        periods.append(period)
    # get data format
    labels = list()
    briefs = list()
    format_dir = self.src_dir+'dataformat.txt'
    with open(format_dir) as file:
      for line in file:
        [label, brief] = line.split(sep=':', maxsplit=1)
        labels.append(label)
        brief = brief.replace('\n','')
        briefs.append(brief)
    # get data
    data_dir = self.src_dir+'data/'
    files = os.listdir(data_dir)
    imu_data = None
    for file in sorted(files):
      datum = np.genfromtxt(data_dir+file, delimiter=' ')
      if len(labels) != datum.shape[0]:
        eprint(longhead+"Err--->> bad datum: "+file+longtail)
        exit()
      if imu_data is None:
        imu_data = np.expand_dims(datum, axis=1)
      else:
        datum = np.expand_dims(datum, axis=1)
        imu_data = np.concatenate((imu_data, datum), axis=1)
    imu_data = imu_data.T
    # add time
    labels.append('time')
    times = np.asarray(times)
    times = np.expand_dims(times, axis=1)
    imu_data = np.concatenate((imu_data,times), axis=1)
    # add period
    labels.append('period')
    periods = np.asarray(periods)
    periods = np.expand_dims(periods, axis=1)
    imu_data = np.concatenate((imu_data, periods), axis=1)
    # load df
    self.df = pd.DataFrame(imu_data, columns=labels)
    self.df.rename(columns={'roll':           'Rr',
                            'pitch':          'Rp',
                            'yaw':            'Ry',
                            'vf':             'vx',
                            'vl':             'vy',
                            'vu':             'vz',
                            'wx':             'wr',
                            'wy':             'wp',
                            'wz':             'wy',
                            'pos_accuracy':   'Pos_acc',
                            'vel_accuracy':   'Vel_acc',
                            },
                    inplace=True,
                    errors='raise')
    self.df = get_quat_data(self.df)
    self.df = get_pose_data(self.df)
    return

  def load_QuVest_set(self):
    # load QuEst data
    fname = 'quest_post_vest.csv'
    self.quest = pd.read_csv(self.src_dir+fname,
      sep=self.options, index_col=0, dtype=self.dtype)
    # load VEst data
    fname = 'vest.csv'
    self.vest = pd.read_csv(self.src_dir+fname,
      sep=self.options, index_col=0, dtype=self.dtype)
    # load data frame
    self.df = pd.concat([self.quest, self.vest], axis=1)
    #self.df.columns = self.labels# load state variables
    return

  ''' end of dmm class...
  '''

def get_quat_data(df:pd.DataFrame):
  Rrpy = df[['Rr', 'Rp', 'Ry']]
  Q_xyzw = list()
  Qxyzw_labels = ['qx', 'qy', 'qz', 'qw']
  for i, row in Rrpy.iterrows():
    rot_vec = np.asarray([[row[0]], [row[1]], [row[2]]])
    q_xyzw = np.asarray(exp_map(rot_vec), dtype=object)
    # TODO: add in scipy implementation for intrinsic config
    #q_xyzw2 = np.asarray(q_xyzw2)
    q_xyzw = np.reshape(q_xyzw, (1,-1))
    Q_xyzw.append(q_xyzw)
  Q_xyzw = np.asarray(Q_xyzw, dtype=object)
  Q_xyzw = np.reshape(Q_xyzw, (-1,4))
  Qxyzw_df = pd.DataFrame(Q_xyzw, columns=Qxyzw_labels)
  df = pd.concat([df, Qxyzw_df], axis=1)
  return df

def get_pose_data(df:pd.DataFrame):
  Vxyz = df[['vx', 'vy', 'vz']]
  period = df[['period']]
  Txyz_labels = ['Tx', 'Ty', 'Tz']
  for i, row in Vxyz.iterrows():
    if i == 0: # initial frame, reference position
      Txyz = np.asarray([0,0,0])
      Txyz = Txyz.reshape((-1, 3))
    else:
      vxyz = np.asarray([row[0], row[1], row[2]])
      txyz = period.iloc[i,0]*vxyz
      txyz = txyz.reshape((-1, 3))
      txyz = txyz + Txyz[-1,:]
      Txyz = np.concatenate((Txyz, txyz), axis=0)
  #nprint('Txyz', Txyz)
  pos_df = pd.DataFrame(Txyz, columns=Txyz_labels)
  df = pd.concat([df, pos_df], axis=1)#, ignore_index=True)
  #nprint('df', df.head(5))
  return df

# 3d plot
def set_axes_equal(ax):
  '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
  cubes as cubes, etc..  This is one possible solution to Matplotlib's
  ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
  Input
    ax: a matplotlib axis, e.g., as output from plt.gca().
  '''
  x_limits = ax.get_xlim3d()
  y_limits = ax.get_ylim3d()
  z_limits = ax.get_zlim3d()
  x_range = abs(x_limits[1] - x_limits[0])
  x_middle = np.mean(x_limits)
  y_range = abs(y_limits[1] - y_limits[0])
  y_middle = np.mean(y_limits)
  z_range = abs(z_limits[1] - z_limits[0])
  z_middle = np.mean(z_limits)
  # The plot bounding box is a sphere in the sense of the infinity
  # norm, hence I call half the max range the plot radius.
  plot_radius = 0.5*max([x_range, y_range, z_range])
  ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
  ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
  ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
  return ax

def plot_quat_vs_quat(quat_A_df,
  quat_B_df,
  title='_',
  figname='fig_04',
  show=False,
  colors=['r','b'],
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels=None,
  y_range=None,
  figsize=[6,6]):
  if end is None:
    end = min(quat_A_df.size, quat_B_df.size)-1
  if labels is None or len(labels) != len(colors):
    label_A = str('quat_A')
    label_B = str('quat_B')
  else:
    label_A = labels[0]
    label_B = labels[1]
  if output_dir is None:
    eprint(longhead+'Err--->> no output directory assigned for figure: '+figname+' .....\n\n')
  fig04 = plt.figure(figsize=figsize)
  ax1 = fig04.add_subplot(411)
  ax1.plot(quat_A_df.qw[start:end], marker='.',c=colors[0], ms=1, label=label_A)
  ax1.plot(quat_B_df.qw[start:end], marker='.',c=colors[1], ms=2, label=label_B)
  ax1.title.set_text('qw')
  if y_range is not None:
    ax1.set_ylim(y_range)
  ax2 = fig04.add_subplot(412)
  ax2.plot(quat_A_df.qx[start:end], marker='.',c=colors[0], ms=1)
  ax2.plot(quat_B_df.qx[start:end], marker='.',c=colors[1], ms=1)
  ax2.set_title('qx')
  if y_range is not None:
    ax2.set_ylim(y_range)
  ax3 = fig04.add_subplot(413)
  ax3.plot(quat_A_df.qy[start:end], marker='.',c=colors[0], ms=1)
  ax3.plot(quat_B_df.qy[start:end], marker='.',c=colors[1], ms=1)
  ax3.set_title('qy')
  if y_range is not None:
    ax3.set_ylim(y_range)
  ax4 = fig04.add_subplot(414)
  ax4.plot(quat_A_df.qz[start:end], marker='.',c=colors[0], ms=1)
  ax4.plot(quat_B_df.qz[start:end], marker='.',c=colors[1], ms=1)
  if y_range is not None:
    ax4.set_ylim(y_range)
  ax4.set_title('qz')
  ax4.set_xlabel('time step')
  fig04.legend()
  # add title
  if title != '_':
    fig04.suptitle('{}'.format(title))
    title = title.replace(' ', '_')
  # save and show image utility
  if save==True and output_dir is not None:
    file_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(file_name, bbox_inches='tight',dpi=400)
    print(shorthead+'saving figure: '+file_name+'.png')
  if show==True: plt.show()
  else: plt.close()
  return

def plot_quat_vs_quat_vs_quat(quat_A_df,
  quat_B_df,
  quat_C_df,
  title='_',
  figname='fig_0x',
  show=False,
  colors=['r','b','m'],
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels=None,
  y_range=[-1.1,1.1],
  figsize=[6,6]):
  if end is None:
    end = min(quat_A_df.size, quat_B_df.size, quat_C_df.size)-1
  if labels is None or len(labels) != len(colors):
    label_A = str('quat_A')
    label_B = str('quat_B')
    label_C = str('quat_C')
  else:
    label_A = labels[0]
    label_B = labels[1]
    label_C = labels[2]
  if output_dir is None:
    eprint(longhead+'Err--->> no output directory assigned for figure: '+figname+' .....\n\n')
  # plot
  fig05 = plt.figure(figsize=figsize)
  ax1 = fig05.add_subplot(411)
  ax1.plot(quat_A_df.qw[start:end], marker='.',c=colors[0], ms=1, label=label_A)
  ax1.plot(quat_B_df.qw[start:end], marker='.',c=colors[1], ms=1, label=label_B)
  ax1.plot(quat_C_df.qw[start:end], marker='.',c=colors[2], ms=1, label=label_C)
  ax1.title.set_text('qw')
  ax1.set_ylim(y_range)
  ax2 = fig05.add_subplot(412)
  ax2.plot(quat_A_df.qx[start:end], marker='.',c=colors[0], ms=1)
  ax2.plot(quat_B_df.qx[start:end], marker='.',c=colors[1], ms=1)
  ax2.plot(quat_C_df.qx[start:end], marker='.',c=colors[2], ms=1)
  ax2.set_title('qx')
  ax2.set_ylim(y_range)
  ax3 = fig05.add_subplot(413)
  ax3.plot(quat_A_df.qy[start:end], marker='.',c=colors[0], ms=1)
  ax3.plot(quat_B_df.qy[start:end], marker='.',c=colors[1], ms=1)
  ax3.plot(quat_C_df.qy[start:end], marker='.',c=colors[2], ms=1)
  ax3.set_title('qy')
  ax3.set_ylim(y_range)
  ax4 = fig05.add_subplot(414)
  ax4.plot(quat_A_df.qz[start:end], marker='.',c=colors[0], ms=1)
  ax4.plot(quat_B_df.qz[start:end], marker='.',c=colors[1], ms=1)
  ax4.plot(quat_C_df.qz[start:end], marker='.',c=colors[2], ms=1)
  ax4.set_ylim(y_range)
  ax4.set_title('qz')
  ax4.set_xlabel('time')
  fig05.legend()
  # add title
  if title != '_':
    fig05.suptitle('{}'.format(title))
    title = title.replace(' ', '_')
  # save and show image utility
  if save==True and output_dir is not None:
    file_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(file_name, bbox_inches='tight',dpi=400)
    print(shorthead+'saving figure: '+file_name+'.png')
  if show==True: plt.show()
  else: plt.close()
  return

def plot_df(df:pd.DataFrame,
  rows,
  cols,
  title='_',
  figname='fig_0x',
  show=False,
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels=None,
  range_padding:float=0.5,
  figsize=[5,8]):
  if end is None:
    end = df.index.size-1
  if labels is None:
    labels = df.columns
  # set plot colors
  cmap = cm.get_cmap('plasma', 25)
  plot_colors = iter(cmap(np.linspace(0, 1, 25)))
  #plot_colors = iter([plt.cm.tab100(i) for i in range(20)])
  # column labels
  t_cols = ['Tx', 'Ty', 'Tz']
  v_cols = ['vx', 'vy', 'vz']
  w_cols = ['wr', 'wp', 'wy']
  q_cols = ['qx', 'qy', 'qz', 'qw']
  loss_cols = ['L1', 'L2']
  # range padding
  pad = 1+range_padding
  # fig
  fig, axs = plt.subplots(nrows=rows, ncols=cols, figsize=figsize, sharex=True, sharey=False)
  for n, ax in enumerate(axs.flatten()):
    col = labels[n]
    ax.plot(df.loc[start:end,col], marker='.',c=next(plot_colors), ms=1, label=col)
    #ticks = [n % 5 == 0, n > end]
    #ax.tick_params(left=ticks[start], bottom=ticks[end])
    ax.set_title(str(col), size=8)
    if col in t_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(t_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(t_cols)]].max(skipna=False)))
    elif col in v_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(v_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(v_cols)]].max(skipna=False)))
    elif col in w_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(w_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(w_cols)]].max(skipna=False)))
    elif col in q_cols:
      ax.set_ylim(-1.1,1.1)
    elif col in loss_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin([col])]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin([col])]].max(skipna=False)))
    else:
      eprint(longhead+'Err--->> column label missing for figure: '+figname+' .....\n\n')
  fig.subplots_adjust(wspace=0.05)
  ax.set_xlabel('time')
  fig.legend()
  # set title
  if title != '_':
    fig.suptitle('{}'.format(title), y = 0.95)
  plt.suptitle(title)
  title = title.replace(' ', '_')
  # save and show image
  if save==True and output_dir is not None:
    fig_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(fig_name, bbox_inches='tight',dpi=400)
    csv_name = output_dir+title
    df.to_csv(csv_name+'.csv', columns=df.columns)
    print(longhead+'saving figure: '+fig_name)
  if show==True: fig.show()
  else: plt.close()
  return

def plot_Txyz_vs_Txyz_3d(z_Txyz_df:pd.DataFrame,
  x_Txyz_df:pd.DataFrame,
  title='_',
  figname='fig_0x',
  show=False,
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels=None,
  colors=['r','b'],
  figsize=[6,6]):
  if end is None:
    #st()
    end = min(z_Txyz_df.index.size, x_Txyz_df.index.size)-1
  if labels is None:
    labels = ['z_Txyz', 'x_Txyz']
  plt.figure(figsize=figsize)
  ax = plt.subplot(111, projection='3d')
  ax.scatter3D(z_Txyz_df['Tx'].iloc[start], z_Txyz_df['Ty'].iloc[start], z_Txyz_df['Tz'].iloc[start], s=50, marker='*',c='y')#, label='z_0')
  #ax.scatter3D(x_Txyz_df['Tx'].iloc[start], x_Txyz_df['Ty'].iloc[start], x_Txyz_df['Tz'].iloc[start], s=50, marker='*',c=colors[1], label='x_0')
  ax.scatter3D(z_Txyz_df['Tx'].iloc[start:end], z_Txyz_df['Ty'].iloc[start:end], z_Txyz_df['Tz'].iloc[start:end], marker='.',c=colors[0], label='Ground-truth', alpha=.3)#labels[0])
  ax.scatter3D(x_Txyz_df['Tx'].iloc[start:end], z_Txyz_df['Ty'].iloc[start:end], z_Txyz_df['Tz'].iloc[start:end], marker='.',c=colors[1], label='QEKF', alpha=.3)#labels[1])
  #ax.scatter3D(z_Txyz_df['Tx'].iloc[end], z_Txyz_df['Ty'].iloc[end], z_Txyz_df['Tz'].iloc[end], s=50, marker='^',c=colors[0], label='z_f')
  #ax.scatter3D(x_Txyz_df['Tx'].iloc[end], x_Txyz_df['Ty'].iloc[end], x_Txyz_df['Tz'].iloc[end], s=50, marker='^',c=colors[1], label='x_f')
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_zlabel('z')
  plt.legend(loc='best')
  ax = set_axes_equal(ax)
  if title != '_': # add title
    plt.title('{}'.format(title))
    title = title.replace(' ', '_')
  # save and show image utility
  if save==True and output_dir is not None:
    file_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(file_name, bbox_inches='tight',dpi=400)
    print(shorthead+'saving figure: '+file_name+'.png')
  if show==True:
    plt.show()
  else:
    plt.close()
  return

def plot_z_df_vs_x_df_iso(z_df:pd.DataFrame,
  x_df:pd.DataFrame,
  labels:list,
  rows=None,
  cols=None,
  title='_',
  figname='fig_0x',
  show=False,
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels_z=None,
  labels_x=None,
  range_padding:float=0.5,
  range_q_padding:float=0.2,
  figsize=[5,8]):
  # check
  if len(z_df.columns) != len(x_df.columns):
    eprint(longhead+'Err--->> in plot_z_df_vs_x_df_iso(), dataframes have UNMATCHED \
      number of columns...'+longtail)
  if labels is None:
    eprint(longhead+'Err--->> in plot_z_df_vs_x_df_iso(), no df label passed...'+longtail)
  if end is None:
    end = min(z_df.index.size,x_df.index.size)-1
  if rows is None or cols is None:
    rows = len(z_df.columns)
    cols = int(1)
    #cols = int(2)
  if labels_z is None:
    labels_z = z_df.columns
  if labels_x is None:
    labels_x = x_df.columns
  # column labels
  t_cols = ['Tx', 'Ty', 'Tz']
  v_cols = ['vx', 'vy', 'vz']
  w_cols = ['wr', 'wp', 'wy']
  q_cols = ['qx', 'qy', 'qz', 'qw']
  # range padding
  pad = 1+range_padding
  qpad = 1+range_q_padding
  # init min and max variables
  t_df_min = None
  t_df_max = None
  v_df_min = None
  v_df_max = None
  w_df_min = None
  w_df_max = None
  q_df_min = None
  q_df_max = None
  ''' set plot colors
  '''
  #plt.style.use("seaborn-dark")
  #plot_colors = iter([plt.cm.tab20(i) for i in range(20)])
  palette_size = 20
  cmap01 = cm.get_cmap('autumn',palette_size)
  palette01 = iter(cmap01(np.linspace(0,1,20)))
  cmap02 = cm.get_cmap('winter',palette_size)
  palette02 = iter(cmap02(np.linspace(0,1,20)))
  # fig
  fig, axs = plt.subplots(nrows=rows, ncols=cols, figsize=figsize, sharex=True,\
    sharey=False)
  for n, ax in enumerate(axs.flatten()):
    zcol = labels_z[n]; xcol = labels_x[n]
    zlab = labels[0]+'_'+zcol
    xlab = labels[1]+'_'+xcol
    ax.plot(z_df.loc[start:end,zcol], marker='.', c=next(palette01), ms=.01, label=zlab)
    ax.plot(x_df.loc[start:end,xcol], marker='.', c=next(palette02), ms=.01, label=xlab)
    if zcol != xcol:
      print(longhead+' Labels '+zcol+' and '+xcol+'DO NOT MATCH for column {}'.format(n))
      eprint(longhead+'Err--->> plot_z_df_vs_x_df_iso(): UNMATCHED column LABELS...'+longtail)
    else:
      ax.set_title(str(zcol), size=8)
    # col y range limits
    if zcol in t_cols:
      if t_df_min is None or t_df_max is None:
        t_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(t_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(t_cols)]].min(skipna=True)))
        t_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(t_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(t_cols)]].max(skipna=True)))
      ax.set_ylim(pad*t_df_min, pad*t_df_max)
    elif zcol in v_cols:
      if v_df_min is None or v_df_max is None:
        v_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(v_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(v_cols)]].min(skipna=True)))
        v_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(v_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(v_cols)]].max(skipna=True)))
      ax.set_ylim(pad*v_df_min, pad*v_df_max)
    elif zcol in w_cols:
      if w_df_min is None or w_df_max is None:
        w_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(w_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(w_cols)]].min(skipna=True)))
        w_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(w_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(w_cols)]].max(skipna=True)))
      ax.set_ylim(pad*w_df_min, pad*w_df_max)
    elif zcol in q_cols:
      if q_df_min is None or q_df_max is None:
        q_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(q_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(q_cols)]].min(skipna=True)))
        q_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(q_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(q_cols)]].max(skipna=True)))
      ax.set_ylim(pad*q_df_min, pad*q_df_max)
      #ax.set_ylim(-qpad, qpad)
    else:
      eprint(longhead+'Err--->> column label missing for figure: '+figname+'...')
  fig.subplots_adjust(wspace=0.05)
  ax.set_xlabel('time')
  fig.legend()
  # set title
  if title != '_':
    fig.suptitle('{}'.format(title), y = 0.95)
  plt.suptitle(title)
  title = title.replace(' ', '_')
  title = title.replace('.','-')
  # save and show image
  if save==True and output_dir is not None:
    file_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(file_name, bbox_inches='tight',dpi=400)
    print(longhead+'saving figure: '+file_name+'.png')
    csv_name = output_dir+title
    z_df.to_csv(csv_name+'__z_.csv', columns=z_df.columns)
    x_df.to_csv(csv_name+'__x_.csv', columns=x_df.columns)
  if show==True:
    fig.show()
  else:
    plt.close()
  return

#TODO: finish this variation of plotting routines
def plot_z_df_vs_x_df_grp(z_df:pd.DataFrame,
  x_df:pd.DataFrame,
  labels:list,
  title='_',
  figname='fig_0x',
  show=False,
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels_z=None,
  labels_x=None,
  range_padding:float=0.5,
  range_q_padding:float=0.2,
  figsize=[5,8]):
  # check
  if len(z_df.columns) != len(x_df.columns):
    eprint(longhead+'Err--->> in plot_z_df_vs_x_df_grp(), dataframes have UNMATCHED \
      number of columns...'+longtail)
  if labels is None:
    eprint(longhead+'Err--->> in plot_z_df_vs_x_df_grp(), no df label passed...'+longtail)
  if end is None:
    end = min(z_df.index.size,x_df.index.size)-1
  rows = int(3)
  cols = int(2)
  if labels_z is None:
    labels_z = z_df.columns
  if labels_x is None:
    labels_x = x_df.columns
  # column labels
  t_cols = ['Tx', 'Ty', 'Tz']
  v_cols = ['vx', 'vy', 'vz']
  w_cols = ['wr', 'wp', 'wy']
  q_cols = ['qx', 'qy', 'qz', 'qw']
  # range padding
  pad = 1+range_padding
  qpad = 1+range_q_padding
  # init min and max variables
  t_df_min = None
  t_df_max = None
  v_df_min = None
  v_df_max = None
  w_df_min = None
  w_df_max = None
  q_df_min = None
  q_df_max = None
  ''' set plot colors
  '''
  #plt.style.use("seaborn-dark")
  #plot_colors = iter([plt.cm.tab20(i) for i in range(20)])
  palette_size = 20
  cmap01 = cm.get_cmap('autumn',palette_size)
  palette01 = iter(cmap01(np.linspace(0,1,20)))
  cmap02 = cm.get_cmap('winter',palette_size)
  palette02 = iter(cmap02(np.linspace(0,1,20)))
  # fig
  fig, axs = plt.subplots(nrows=rows, ncols=cols, figsize=figsize, sharex=True,\
    sharey=False)
  for n, ax in enumerate(axs.flatten()):
    zcol = labels_z[n]; xcol = labels_x[n]
    zlab = labels[0]+'_'+zcol
    xlab = labels[1]+'_'+xcol
    #ax.plot(z_df.loc[start:end,zcol], marker='.', c=next(palette01), ms=.01, label=zlab)
    #ax.plot(x_df.loc[start:end,xcol], marker='.', c=next(palette02), ms=.01, label=xlab)
    if zcol != xcol:
      print(longhead+' Labels '+zcol+' and '+xcol+'DO NOT MATCH for column {}'.format(n))
      eprint(longhead+'Err--->> plot_z_df_vs_x_df_iso(): UNMATCHED column LABELS...'+longtail)
    else:
      ax.set_title(str(zcol), size=8)
    # col y range limits
    if zcol in t_cols:
      if n%2 == 1:  # z
        ax.plot(z_df.loc[start:end,zcol], marker='.', c=next(palette01), ms=1, label=zlab)
      elif n%2 == 0: # x
        ax.plot(x_df.loc[start:end,xcol], marker='.', c=next(palette02), ms=1, label=xlab)
      else:
        eprint(longhead+'Err--->> must be unreachable...'+longtail)
      if t_df_min is None or t_df_max is None:
        t_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(t_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(t_cols)]].min(skipna=True)))
        t_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(t_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(t_cols)]].max(skipna=True)))
      ax.set_ylim(pad*t_df_min, pad*t_df_max)
    elif zcol in v_cols:
      if n%2 == 1:  # z
        ax.plot(z_df.loc[start:end,zcol], marker='.', c=next(palette01), ms=.01, label=zlab)
      elif n%2 == 0: # x
        ax.plot(x_df.loc[start:end,xcol], marker='.', c=next(palette02), ms=1, label=xlab)
      else:
        eprint(longhead+'Err--->> must be unreachable...'+longtail)
      if v_df_min is None or v_df_max is None:
        v_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(v_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(v_cols)]].min(skipna=True)))
        v_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(v_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(v_cols)]].max(skipna=True)))
      ax.set_ylim(pad*v_df_min, pad*v_df_max)
    elif zcol in w_cols:
      if n%2 == 1:  # z
        ax.plot(z_df.loc[start:end,zcol], marker='.', c=next(palette01), ms=.01, label=zlab)
      elif n%2 == 0: # x
        ax.plot(x_df.loc[start:end,xcol], marker='.', c=next(palette02), ms=1, label=xlab)
      else:
        eprint(longhead+'Err--->> must be unreachable...'+longtail)
      if w_df_min is None or w_df_max is None:
        w_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(w_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(w_cols)]].min(skipna=True)))
        w_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(w_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(w_cols)]].max(skipna=True)))
      ax.set_ylim(pad*w_df_min, pad*w_df_max)
    elif zcol in q_cols:
      if n%2 == 1:  # z
        ax.plot(z_df.loc[start:end,zcol], marker='.', c=next(palette01), ms=.01, label=zlab)
      elif n%2 == 0: # x
        ax.plot(x_df.loc[start:end,xcol], marker='.', c=next(palette02), ms=1, label=xlab)
      else:
        eprint(longhead+'Err--->> must be unreachable...'+longtail)
      if q_df_min is None or q_df_max is None:
        q_df_min =min(min(z_df[z_df.columns[z_df.columns.isin(q_cols)]].min(skipna=True)),\
                      min(x_df[x_df.columns[x_df.columns.isin(q_cols)]].min(skipna=True)))
        q_df_max =max(max(z_df[z_df.columns[z_df.columns.isin(q_cols)]].max(skipna=True)),\
                      max(x_df[x_df.columns[x_df.columns.isin(q_cols)]].max(skipna=True)))
      ax.set_ylim(pad*q_df_min, pad*q_df_max)
      #ax.set_ylim(-qpad, qpad)
    else:
      eprint(longhead+'Err--->> column label missing for figure: '+figname+'...')
  fig.subplots_adjust(wspace=0.05)
  ax.set_xlabel('time')
  fig.legend()
  # set title
  if title != '_':
    fig.suptitle('{}'.format(title), y = 0.95)
  plt.suptitle(title)
  title = title.replace(' ', '_')
  title = title.replace('.','-')
  # save and show image
  if save==True and output_dir is not None:
    file_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(file_name, bbox_inches='tight',dpi=400)
    print(shorthead+'saving figure: '+file_name+'.png')
    csv_name = output_dir+title
    z_df.to_csv(csv_name+'__z_.csv', columns=z_df.columns)
    x_df.to_csv(csv_name+'__x_.csv', columns=x_df.columns)
  if show==True:
    fig.show()
  else:
    plt.close()
  return

def plot_df_grp(df:pd.DataFrame,
  rows,
  cols,
  title='_',
  figname='fig_0x',
  show=False,
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels=None,
  range_padding:float=0.5,
  figsize=[5,8]):
  if end is None:
    end = df.index.size-1
  if labels is None:
    labels = df.columns
  # set plot colors
  cmap = cm.get_cmap('plasma', 15)
  plot_colors = iter(cmap(np.linspace(0, 1, 15)))
  #plot_colors = iter([plt.cm.tab100(i) for i in range(20)])
  # column labels
  t_cols = ['Tx', 'Ty', 'Tz']
  v_cols = ['vx', 'vy', 'vz']
  w_cols = ['wr', 'wp', 'wy']
  q_cols = ['qx', 'qy', 'qz', 'qw']
  # range padding
  pad = 1+range_padding
  # fig
  fig, axs = plt.subplots(nrows=rows, ncols=cols, figsize=figsize, sharex=True, sharey=False)
  for n, ax in enumerate(axs.flatten()):
    col = labels[n]
    ax.plot(df.loc[start:end,col], marker='.',c=next(plot_colors), ms=1, label=col)
    #ticks = [n % 5 == 0, n > end]
    #ax.tick_params(left=ticks[start], bottom=ticks[end])
    ax.set_title(str(col), size=8)
    if col in t_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(t_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(t_cols)]].max(skipna=False)))
    elif col in v_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(v_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(v_cols)]].max(skipna=False)))
    elif col in w_cols:
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(w_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(w_cols)]].max(skipna=False)))
    elif col in q_cols:
      #ax.set_ylim(-1.1,1.1)
      ax.set_ylim(pad*min(df[df.columns[df.columns.isin(q_cols)]].min(skipna=False)),\
                  pad*max(df[df.columns[df.columns.isin(q_cols)]].max(skipna=False)))
    else:
      eprint(longhead+'Err--->> column label missing for figure: '+figname+' .....\n\n')
  fig.subplots_adjust(wspace=0.05)
  ax.set_xlabel('time')
  fig.legend()
  # set title
  if title != '_':
    fig.suptitle('{}'.format(title), y=0.95)
  plt.suptitle(title)
  title = title.replace(' ', '_')
  # save and show image
  if save==True and output_dir is not None:
    fig_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(fig_name, bbox_inches='tight',dpi=400)
    csv_name = output_dir+title
    df.to_csv(csv_name+'.csv', columns=df.columns)
    print(longhead+'saving figure: '+fig_name)
  if show==True: fig.show()
  else: plt.close()
  return


def plot_df_grp_K(df:pd.DataFrame,
  title='_',
  figname='fig_0x',
  show=False,
  save=True,
  output_dir='../out/',
  start=0,
  end=None,
  labels=None,
  range_padding:float=0.5,
  figsize=[6,6]):
  if end is None:
    end = df.index.size-1
  if labels is None:
    labels = df.columns
  # set plot colors
  cmap = cm.get_cmap('plasma', 15)
  plot_colors = iter(cmap(np.linspace(0, 1, 15)))
  #plot_colors = iter([plt.cm.tab100(i) for i in range(20)])
  ax = df.plot(figsize=figsize)
  ax.set_xlabel('time')
  #plt.legend(loc='best')
  # set title
  if title != '_':
    plt.suptitle('{}'.format(title), y = 0.95)
  plt.suptitle(title)
  title = title.replace(' ', '_')
  title = title.replace('.', '-')
  # save and show image
  if save==True and output_dir is not None:
    fig_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(fig_name, bbox_inches='tight',dpi=400)
    csv_name = output_dir+title
    df.to_csv(csv_name+'.csv', columns=df.columns)
    print(longhead+'saving figure: '+fig_name)
  if show==True: plt.show()
  else: plt.close()
  return

# EOF
