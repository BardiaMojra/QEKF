import numpy as np
from scipy.linalg import norm
import pandas as pd
from nbug import *
from pdb import set_trace as st

def get_skew_symm_X(x):
  X = np.zeros((3,3))
  X[0,1] = -x[2]
  X[0,2] =  x[1]
  X[1,0] =  x[2]
  X[1,2] = -x[0]
  X[2,0] = -x[1]
  X[2,1] =  x[0]
  return X

def get_omega(q):
  omega = np.array([  [-q[1], -q[2], -q[3] ],
                      [ q[0],  q[3], -q[2] ],
                      [-q[3],  q[0],  q[1] ],
                      [ q[2], -q[1],  q[0] ]
                      ])
  return np.reshape(omega,(4,3))

def check_symmetric(a, tol=1e-8):
  return np.all(np.abs(a-a.T) < tol)

### Exponential Map of Quaternion
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
  return [ temp_[0],temp_[1],temp_[2], temp_2]

### Logarithmic map of Quaternion
def Q_log(q):
  q_v = q[0]
  q_n = np.array([q[1],q[2],q[3]])
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



def get_losses(res: pd.DataFrame):
  L1 = list()
  L2 = list()
  # nprint_2('get_L2loss: res', res.head(5))

  # L2 = np.zeros((len(res.index), len(res.columns)))
  for i in range(len(res.index)):
    state_l1 = 0.0
    state_l2 = 0.0
    for j in range(len(res.columns)):
      # st()
      l1 = abs(res.iloc[i,j])
      l2 = res.iloc[i,j] ** 2
      state_l1 += l1
      state_l2 += l2
      # nprint(shorthead+'row sum ', res.iloc[i,:].sum())
    L1.append(state_l1)
    L2.append(state_l2)
  L1_df = pd.DataFrame(L1, columns=['L1'])
  L2_df = pd.DataFrame(L2, columns=['L2'])
  res = pd.concat([res,L1_df, L2_df], axis=1)
  # nprint_2('get_L2loss: res', res.head(5))
  return res

def print_losses(df: pd.DataFrame):
  nprint(longhead+"L1 (total)", df['L1'].sum())
  nprint(longhead+"L2 (total)", df['L2'].sum())
  return
