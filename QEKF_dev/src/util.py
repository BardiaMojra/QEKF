import sys, csv
import numpy as np
from numpy import dot, zeros, eye
from scipy.linalg import norm
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
import os
from scipy.spatial.transform import Rotation as R

# syned to orientation data
def read_utari_data(imu_dir, vicon_file):
  acc_file = open(imu_dir+'/acce.txt', "r")
  gyro_file = open(imu_dir+'/gyro.txt', "r")
  quat_file = open(imu_dir+'/rv.txt', "r")

  acc_lines = acc_file.readlines()
  gyro_lines = gyro_file.readlines()
  quat_lines = quat_file.readlines()

#     sz = len(acc_lines)-1
  acceleration = np.zeros((len(acc_lines)-1,4))
  _gyro = []
#     _quat = []
  _acc = []
  for i in range(len(acc_lines)-1-1):
    data = acc_lines[i+1].split()
    acceleration[i,:] = data[0:4]

  sz = len(quat_lines)-1
  quat_ = np.zeros((sz,5))
  for i in range(sz-1):
    data = quat_lines[i+1].split()
    quat_[i,:] = data[0:5]


#     sz = len(gyro_lines)-1
  gyro = np.zeros((len(gyro_lines)-1,4))
  _gyro = []
  for i in range(len(gyro_lines)-1):
    data = gyro_lines[i+1].split()
    gyro[i,:] = data[0:4]

  # down sample imu gyro data
  k=0
  for i in range(sz):
    t_ = quat_[i,0]
    for j in range(k,len(gyro_lines)-2):
      a = np.sign(gyro[j,0]-t_)
      b = np.sign(gyro[j+1,0]-t_)
      if a!=b:
        if a<=0:
          _gyro.append(gyro[j,1:4])
          k=j
          break
        elif b==0:
          _gyro.append(gyro[j+1,1:4])
          k=j
          break

  # down sample imu acceleration data
  k=0
  for i in range(sz):
    t_ = quat_[i,0]
    for j in range(k,len(acc_lines)-2):
      a = np.sign(acceleration[j,0]-t_)
      b = np.sign(acceleration[j+1,0]-t_)
      if a!=b:
        if a<=0:
          _acc.append(acceleration[j,1:4])
          k=j
          break
        elif b==0:
          _acc.append(acceleration[j+1,1:4])
          k=j
          break

  gyro_bias_file = open(imu_dir+'/gyro_bias.txt', 'r')
  lines = gyro_bias_file.readlines()
  data = lines[1].split()
  gyro_bias = data[0:3]

  ## Vicon
  with open(vicon_file, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    data_length = sum(1 for row in reader) -9

  with open(vicon_file, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    translation = np.zeros((data_length,3))
    rotation = np.zeros((data_length,4))
    index =0
    for row in reader:
      if len(row)>0:
        ss = row[0].split(',')
      else:
        continue
      if len(ss)<9 or index<10:
        index=index+1
        continue
      translation[index-10,:] = float(ss[6]) ,float(ss[7]),float(ss[8])
      rotation[index-10,:] = float(ss[2]) ,float(ss[3]),float(ss[4]),float(ss[5])
      index = index+1
  return np.asarray(_acc), np.asarray(_gyro), np.asarray(quat_[:,1:5]),\
    np.double(gyro_bias), translation*1e-3, rotation


def read_oxford_data(filename):
    # read csv files from oxford inertial odometry dataset
  # timestamp, roll, pitch, yaw, angular_rate, gravity,  acceleration, magnetic_field = [],[],[],[],[],[],[],[]
  with open(filename, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    data_length = sum(1 for row in reader)

  with open(filename, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    timestamp = np.zeros(data_length)
    roll = np.zeros_like(timestamp)
    pitch = np.zeros_like(timestamp)
    yaw = np.zeros_like(timestamp)
    angular_rate = np.zeros((data_length,3))
    acceleration = np.zeros((data_length,3))
    magnetic_field = np.zeros((data_length,3))
    gravity = np.zeros((data_length,3))

    index =0
    # set_trace()
    for row in reader:
      ss = row[0].split(',')
      # print(row[0])
      timestamp[index] = float(ss[0])
      roll[index] =  float(ss[1])
      pitch[index] = float(ss[2])
      yaw[index] = float(ss[3])
      angular_rate[index,:] = float(ss[4]) ,float(ss[5]),float(ss[6])
      gravity[index,:] = float(ss[7]) ,float(ss[8]),float(ss[9])
      acceleration[index,:] = float(ss[10]) ,float(ss[11]),float(ss[12])
      magnetic_field[index,:] = float(ss[13]) ,float(ss[14]),float(ss[15])
      index = index+1

  return timestamp, roll, pitch, yaw, angular_rate, gravity,  acceleration, magnetic_field

def read_oxford_vicon_data(filename):
  with open(filename, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    data_length = sum(1 for row in reader)
    timestamp = np.zeros(data_length)
    translation = np.zeros((data_length,3))
    rotation = np.zeros((data_length,4))
  with open(filename, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    timestamp = np.zeros(data_length)
    index =0
    for row in reader:
      ss = row[0].split(',')
      timestamp[index] = float(ss[0])
      translation[index,:] = float(ss[2]) ,float(ss[3]),float(ss[4])
      rotation[index,:] = float(ss[5]) ,float(ss[6]),float(ss[7]),float(ss[8])
      index = index +1
  return timestamp, translation, rotation



def get_skew_symm_X(x):
  X = np.zeros((3,3))
  X[0,1] = -x[2]
  X[0,2] = x[1]
  X[1,0] = x[2]
  X[1,2] = -x[0]
  X[2,0] = -x[1]
  X[2,1] = x[0]
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


## fixed the error in implementation but results donot match matlab quat2rotm
def transform_(q):
  Q=np.array(
    [ [2*q[0]**2-1+2*q[3]**2,   2*q[0]*q[1]+2*q[2]*q[3], 2*q[0]*q[2]-2*q[1]*q[3]],
      [2*q[0]*q[1]-2*q[2]*q[3], 2*q[1]**2-1+2*q[3]**2,   2*q[1]*q[2]+2*q[0]*q[3]],
      [2*q[0]*q[2]+2*q[1]*q[3], 2*q[1]*q[2]-2*q[0]*q[3], 2*q[2]**2-1+2*q[3]**2]   ]
      )
  return np.reshape(Q,(3,3))

### Exponential Map of Quaternion
def exp_map(x):
  if np.shape(x)[0] !=3:
    print("Vector size is not 3")
    return -1
  norm_x = norm(x)
  x = np.asarray(x)
  if norm_x ==0: return np.array([1,0,0,0])
  temp_ = np.sin(norm_x/2)*x/norm_x
  temp_2 = np.cos(norm_x/2)
  return [temp_[0],temp_[1],temp_[2],temp_2]

### Logarithmic map of Quaternion
def Q_log(q):
  q_v = q[0]
  q_n = np.array([q[1],q[2],q[3]])
  norm_q_n = np.linalg.norm(q_n)
  if q_v>1:
    q_v=1
  if q_v<-1:
    q_v=-1
  if (norm_q_n!=0 and q_v>=0):
    return 2*np.arccos(q_v)*q_n/norm_q_n
  elif (norm_q_n!=0 and q_v<0):
    return -2*np.arccos(-q_v)*q_n/norm_q_n
  elif norm_q_n==0:
    return zeros((3,1))

def read_utari_data_raw(imu_dir):
  acc_file = open(imu_dir+'/acce.txt', "r")
  gyro_file = open(imu_dir+'/gyro.txt', "r")
  quat_file = open(imu_dir+'/rv.txt', "r")

  acc_lines = acc_file.readlines()
  gyro_lines = gyro_file.readlines()
  quat_lines = quat_file.readlines()

#     sz = len(acc_lines)-1
  acceleration = np.zeros((len(acc_lines)-1,4))
  _gyro = []
#     _quat = []
  _acc = []
  for i in range(len(acc_lines)-1-1):
    data = acc_lines[i+1].split()
    acceleration[i,:] = data[0:4]
  return acceleration


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
