import numpy as np
from scipy.linalg import norm
import pandas as pd
from nbug import *
from pdb import set_trace as st
import datetime






def toQuat_set(Rrpy_df:pd.DataFrame) -> pd.DataFrame:

  #todo

  return Qxyzw_df


def get_skew_symm_X(x):
  X = np.zeros((3,3))
  X[0,1] = -x[2]
  X[0,2] =  x[1]
  X[1,0] =  x[2]
  X[1,2] = -x[0]
  X[2,0] = -x[1]
  X[2,1] =  x[0]
  return X

def get_unix_time_from_str(dateStr):
  datetime_format = '%Y-%m-%d %H:%M:%S.%f'
  date_format = datetime.datetime.strptime(dateStr,datetime_format)
  unix_time = datetime.datetime.timestamp(date_format)
  return int(unix_time*1000)

def get_epoch_series(start_epoch, end_epoch, numSamps):
  timestamps = list()
  delta = end_epoch - start_epoch
  period = delta/numSamps
  timestamps = np.arange(start_epoch, end_epoch, period)
  epochs = [ int(i) for i in timestamps]
  return epochs

def get_omega(q):
  omega = np.array([  [-q[1], -q[2], -q[3] ],
                      [ q[0],  q[3], -q[2] ],
                      [-q[3],  q[0],  q[1] ],
                      [ q[2], -q[1],  q[0] ]
                      ])
  return np.reshape(omega,(4,3))

def check_symmetric(a, tol=1e-8):
  return np.all(np.abs(a-a.T) < tol)

### Exponential Map of Quaternion: this is correct...
### https://www.cs.cmu.edu/~spiff/moedit99/expmap.pdf
### https://arxiv.org/pdf/1711.02508.pdf
def exp_map(x):
  if np.shape(x)[0] !=3:
    print("Vector size is not 3")
    return -1
  norm_x = norm(x)
  x = np.asarray(x)
  if norm_x ==0: return np.array([0,0,0,1])
  #nprint(_prt, 'norm_x', norm_x)
  temp_ = np.sin(norm_x/2)*x/norm_x
  temp_2 = np.cos(norm_x/2)
  return [ temp_[0],temp_[1],temp_[2], temp_2] # xyzw




### Logarithmic map of Quaternion wxyz
def Q_log(q):
  q_v = q[0]
  q_n = np.array([q[1],q[2],q[3]])
  q_n = q_n.reshape(-1,1)
  norm_q_n = np.linalg.norm(q_n)
  #todo: re-evluate this
  if q_v>1:
    q_v=1
    #st()
  if q_v<-1:
    q_v=-1
    #st()
  if (norm_q_n!=0 and q_v>=0):
    return 2*np.arccos(q_v)*q_n/norm_q_n
  elif (norm_q_n!=0 and q_v<0):
    return -2*np.arccos(-q_v)*q_n/norm_q_n
  elif norm_q_n==0:
    return np.zeros((3,1))

def get_Qwxyz(xyz: np.array):
  ''' Calculate the real term (w), given the imaginary terms (xyz)
    calculates unit quaternion from Qxyz (point quaternion)
  '''
  # sqrt(1-x^2-y%2-z^2) to confirm real part calc
  w = np.sqrt(1 -xyz[0]**2 -xyz[1]**2 -xyz[2]**2)
  return [w, xyz[0], xyz[1], xyz[2]]

def get_Qxyzw(xyz: np.array):
  ''' Calculate the real term (w), given the imaginary terms (xyz)
    calculates unit quaternion from Qxyz (point quaternion)
  '''
  # sqrt(1-x^2-y%2-z^2) to confirm real part calc
  w = np.sqrt(1 -xyz[0]**2 -xyz[1]**2 -xyz[2]**2)
  return [xyz[0], xyz[1], xyz[2], w]


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
  return


# def qProd(q,r):
#   '''
#     This routine uses quaternions of the form of
#       q=q0+iq1+jq2+kq3
#     and
#       r=r0+ir1+jr2+kr3.
#     The quaternion product has the form of
#       t=q×r=t0+it1+jt2+kt3,
#     where
#       t0=(r0q0−r1q1−r2q2−r3q3)
#       t1=(r0q1+r1q0−r2q3+r3q2)
#       t2=(r0q2+r1q3+r2q0−r3q1)
#       t3=(r0q3−r1q2+r2q1+r3q0)
#     @link https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html?searchHighlight=quaternion%20multiplication&s_tid=srchtitle_quaternion%20multiplication_1
#   '''
#   st()
#   q[0],q[1],q[2],q[3]
#   r[0],r[1],r[2],r[3]
#   t0=(r0q0−r1q1−r2q2−r3q3)
#   t1=(r0q1+r1q0−r2q3+r3q2)
#   t2=(r0q2+r1q3+r2q0−r3q1)
#   t3=(r0q3−r1q2+r2q1+r3q0)
#   t = [t0,t1,t2,t3]
#   return t
