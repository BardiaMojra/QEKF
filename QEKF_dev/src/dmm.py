
import sys
import pandas as pd
from pdb import set_trace as st
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm
import os
from pprint import pprint as pp

''' matplotlib config '''
matplotlib.pyplot.ion()
plt.style.use('ggplot')

longhead = '\n\n  \--->> '
shorthead = '\n  \--->> '
longtail = '\n\n'
shorttail = '\n'

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

class dmm:
  ''' Data Management Module
  '''
  def __init__(self, name):
    # config
    prt = True
    labels = None
    # set dataset configs
    if name == 'dataset-iphone1_clean':
      src_dir = '../data/dataset-iphone1_clean/'
      output_dir = '../out/out_'+name+'/'
      ext = 'xlsx'
      opt = None
    elif name =='bigC_06-Aug2021':
      src_dir = '../data/bigC_06-Aug2021/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = ' ' # space separator for csv file
    elif name =='kitti_imu_001':
      src_dir = '../data/KITTI/2011_09_26/2011_09_26_drive_0001_sync/oxts/'
      output_dir = '../out/out_'+name+'/'
      ext = 'csv'
      opt = None
    else:
      eprint(longhead+'Err--->> no data selected.....\n\n')
    self.name = name
    self.src_dir = src_dir
    self.ext = ext
    self.dtype = 'float64'
    self.options = opt
    self.labels = labels
    self.prt = prt
    self.output_dir = output_dir
    if not os.path.exists(self.output_dir):
      #os.makedirs(self.output_dir)
      print(longhead+'the following directory DOES NOT EXIST: '+self.output_dir)
      print(shorthead+"create is it with 'mkdir "+self.output_dir+"'\n\n")
      exit()
    self.df = None
    self.quest = None
    self.vest = None
    self.trans_xyz = None
    self.vel_xyz = None
    self.acc_xyz = None
    self.quat_xyzw = None # rotation in quaternion - x,y,z,w
    self.quat_wxyz = None # rotation in quaternion - w,x,y,z
    self.vel_rpy = None # rad/sec - roll, pitch, yaw
    self.acc_rpy = None # rad/sec^2 - roll, pitch, yaw
    # end of __init__()

  def format(self):
    ''' data files:
      iphone_mouse_zoom_2:
        u_cord_subpix.csv: u axes position for 8 points: x1, x2, ... x8;
        u_flow_subpix.csv
        v_cord_subpix.csv
        v_flow_subpix.csv
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
    '''
    if self.name=="iphone_mouse_zoom_2" and self.ext=='csv':
      fname = 'u_cord_subpix.csv'
      self.df = pd.read_csv(self.src_dir+fname)
    elif self.name=="dataset-iphone1_clean" and self.ext=='xlsx':
      # load QuEst data
      fname = 'quest.xlsx'
      self.quest = pd.read_excel(self.src_dir+fname, engine='openpyxl',\
        index_col=0, dtype=self.dtype)
      # load VEst data
      fname = 'vest.xlsx'
      self.vest = pd.read_excel(self.src_dir+fname, engine='openpyxl',\
        index_col=0, dtype=self.dtype)
      # load data frame
      self.df = pd.concat([self.quest, self.vest], axis=1)
    elif self.name=="bigC_06-Aug2021" and self.ext=='csv':
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
    elif self.name=="kitti_imu_001" and self.ext=='csv':
      '''
        /data/KITTI/2011_09_26/2011_09_26_drive_0001_sync/oxts/.
          ├── data/ <<< each datum is in separate file
          ├── dataformat.txt
          └── timestamps.txt
      '''

      # get time stamp
      tstamp_dir = self.src_dir+'timestamps.txt'
      data_dir = self.src_dir+'data/'
      tstamp_df = pd.read_csv(tstamp_dir, sep=' ', header=None, names=['date', 'time'])
      '''NBUG'''
      #print(longhead+tstamp_dir+':')
      #pp(tstamp_df.head(5))

      # get data format
      labels = list()
      labels_ = {}
      format_dir = self.src_dir+'dataformat.txt'
      with open('format_dir') as file:
        for line in file:
          (key, val) = line.split(sep=':')
          labels.append(key, val)
          labels_[int(key)] = val
        '''NBUG'''
        print(shorthead+'labels:')
        print(labels); print(shorttail)
        print(shorthead+'labels_:')
        print(labels_); print(shorttail)

      st()


      # get data
      files = os.listdir(data_dir)
      print(longhead+data_dir+':')
      print(sorted(files))

      st()
      for file in sorted(files):
        datum = np.genfromtxt(data_dir+file, delimiter=' ')
        '''NBUG'''
        print(shorthead+file+':')
        print(datum); print(shorttail)


    else:
      eprint(longhead+'Err--->> invalid name and/or ext!\n\n', file=sys.stderr)
      exit()
    st()
    # load state variables
    self.trans_xyz = self.df[['Tx', 'Ty', 'Tz']]
    self.quat_xyzw = self.df[['qx', 'qy', 'qz', 'qw']]
    self.quat_wxyz = self.df[['qw', 'qx', 'qy', 'qz']]
    self.vel_xyz = self.df[['vx', 'vy', 'vz']]
    self.vel_rpy = self.df[['wr', 'wp', 'wy']]
    self.labels = self.df.columns
    if self.prt == True:
      print(longhead+'trans_xyz:')
      pp(self.trans_xyz)
      print(longhead+'rot_xyzw:')
      pp(self.quat_xyzw)
      print(longhead+'rot_wxyz:')
      pp(self.quat_wxyz)
      print(longhead+'vel_xyz:')
      pp(self.vel_xyz)
      print(longhead+'vel_rpy:')
      pp(self.vel_rpy)
      print(longtail)
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
      file_name = self.output_dir+'/'+'{}'.format(figname+'_'+title)
      plt.savefig(file_name, bbox_inches='tight',dpi=400)
      print(longhead+'saving figure: '+file_name)
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
      print(longhead+'saving figure: '+file_name)
    if show==True:
      plt.show()
    else:
      plt.close()
    return

# # 3d plot
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
  figsize=[8,8]):
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
    print(longhead+'saving figure: '+file_name)
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
  figsize=[8,8]):
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
    print(longhead+'saving figure: '+file_name)
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
  range_padding:float=1.0,
  figsize=[5,10]):
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
    ax.set_title(str(col), size=12)
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
    file_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(file_name, bbox_inches='tight',dpi=400)
    print(longhead+'saving figure: '+file_name)
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
  figsize=[8,8]):
  if end is None:
    end = min(z_Txyz_df.index.size, x_Txyz_df.index.size)-1
  if labels is None:
    labels = ['z_Txyz', 'x_Txyz']
  plt.figure(figsize=figsize)
  ax = plt.subplot(111, projection='3d')
  ax.scatter3D(z_Txyz_df['Tx'].iloc[start], z_Txyz_df['Ty'].iloc[start], z_Txyz_df['Tz'].iloc[start], s=100, marker='*',c='y', label='start')
  ax.scatter3D(z_Txyz_df.Tx, z_Txyz_df.Ty, z_Txyz_df.Tz, marker='.',c=colors[0], label=labels[0])
  ax.scatter3D(x_Txyz_df.Tx, x_Txyz_df.Ty, x_Txyz_df.Tz, marker='.',c=colors[1], label=labels[1])
  ax.scatter3D(z_Txyz_df['Tx'].iloc[end], z_Txyz_df['Ty'].iloc[end], z_Txyz_df['Tz'].iloc[end], s=100, marker='*',c='k', label='end')
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
    print(longhead+'saving figure: '+file_name)
  if show==True:
    plt.show()
  else:
    plt.close()
  return

def plot_z_df_vs_x_df(z_df:pd.DataFrame,
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
  range_padding:float=1.0,
  range_q_padding:float=0.1,
  figsize=[5,10]):
  # check
  if len(z_df.columns) != len(x_df.columns):
    eprint(longhead+'Err--->> in plot_z_df_vs_x_df(), dataframes have UNMATCHED \
      number of columns...'+longtail)
  if labels is None:
    eprint(longhead+'Err--->> in plot_z_df_vs_x_df(), no df label passed...'+longtail)
  if end is None:
    end = min(z_df.index.size,x_df.index.size)-1
  if rows is None or cols is None:
    rows = len(z_df.columns)
    cols = int(1)
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
  palette_size = 25
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
      eprint(longhead+'Err--->> plot_z_df_vs_x_df(): UNMATCHED column LABELS...')
    else:
      ax.set_title(str(zcol), size=8,)
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
      ax.set_ylim(-qpad, qpad)
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
  # save and show image
  if save==True and output_dir is not None:
    file_name = output_dir+'{}'.format(figname+'_'+title)
    plt.savefig(file_name, bbox_inches='tight',dpi=400)
    print(longhead+'saving figure: '+file_name)
  if show==True:
    fig.show()
  else:
    plt.close()
  return


# EOF
