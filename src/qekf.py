from copy import deepcopy
from math import log, exp, sqrt
import sys, csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from math import isnan,isinf
from numpy import dot, zeros, eye
import scipy.linalg as linalg
from scipy.linalg import det, eig
from scipy.linalg import norm
from pdb import set_trace
import h5py
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
# from filterpy.stats import logpdf
# from filterpy.common import pretty_str, reshape_z
from pyquaternion import Quaternion
import pdb

debug = False #
# debug = True

adaptive_ekf_window = 25
print_output = True

matplotlib.pyplot.ion()
plt.style.use('ggplot')
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
  return np.asarray(_acc), np.asarray(_gyro), np.asarray(quat_[:,1:5]), np.double(gyro_bias), translation*1e-3, rotation


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

np.random.seed(5555)
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

# def transform_conj(q):
#   Q = np.array([   [2*q[0]**2-1+2*q[1]**2,    2*q[1]*q[2]-2*q[0]*q[3],  2*q[1]*q[3]+2*q[0]*q[2] ],
#           [2*q[1]*q[2]+2*q[0]*q[3],  2*q[0]**2-1+2*q[2]**2,    2*q[2]*q[3]-2*q[0]*q[1] ],
#           [2*q[1]*q[3]-2*q[0]*q[2],  2*q[2]*q[3]+2*q[0]*q[1],  2*q[0]**2-1+2*q[3]**2] ])
#   return np.reshape(Q,(3,3))

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

class ExtendedKalmanFilter(object):
  def __init__(self, dim_x, dim_z, dim_u=0):

    self.dim_x = dim_x
    self.dim_z = dim_z
    self.dim_u = dim_u

    self.x = zeros((dim_x, 1)) # state
    self.P = eye(dim_x)        # uncertainty covariance
    self.B = 0                 # control transition matrix
    self.F = np.eye(dim_x)     # state transition matrix
    self.R = eye(dim_z)        # state uncertainty
    self.Q = eye(dim_x)        # process uncertainty
    self.y = zeros((dim_z, 1)) # residual
    self.G = None
    self.T_ = 1 #time-period
    self.w_=zeros((9,1))
    # z = np.array([None]*self.dim_z)
    self.z = zeros((dim_z,1))
    self.v = zeros((dim_z,adaptive_ekf_window))
    # gain and residual are computed during the innovation step. We
    # save them so that in case you want to inspect them for various
    # purposes
    self.K = np.zeros(self.x.shape) # kalman gain
    self.y = zeros((dim_z, 1))
    self.S = np.zeros((dim_z, dim_z))   # system uncertainty
    self.SI = np.zeros((dim_z, dim_z))  # inverse system uncertainty
    self.L = None
    # identity matrix. Do not alter this.
    self._I = np.eye(dim_x)

    # self._log_likelihood = log(sys.float_info.min)
    # self._likelihood = sys.float_info.min
    # self._mahalanobis = None

    # these will always be a copy of x,P after predict() is called
    self.x_prior = self.x.copy()
    self.P_prior = self.P.copy()

    # these will always be a copy of x,P after update() is called
    self.x_post = self.x.copy()
    self.P_post = self.P.copy()
    self.C = None
    self.sensor_measurements = None
    self.x_q= Quaternion([1,0,0,0])

  def update(self, z, R=None, args=(), hx_args=(), residual=np.subtract):
    if z is None:
      self.z = np.array([[None]*self.dim_z]).T
      self.x_post = self.x.copy()
      self.P_post = self.P.copy()
      return

    if not isinstance(args, tuple):
      args = (args,)

    if not isinstance(hx_args, tuple):
      hx_args = (hx_args,)

    if R is None:
      R = self.R
    elif np.isscalar(R):
      R = eye(self.dim_z) * R

    if np.isscalar(z) and self.dim_z == 1:
      z = np.asarray([z], float)

    # H = HJacobian(self.x, *args)
#         H = self.H
    PHT = dot(self.P_prior, self.H.T)
    self.S = dot(self.H, PHT) + R
    self.K = PHT.dot(linalg.inv(self.S))

    # hx = Hx(self.x, *hx_args)
#         hx = np.dot(self.H,self.x_prior)
#         self.y = residual(z, hx.T).T
    # modified for qekf and quarternion states
#         set_trace()
    x_temp = zeros((self.dim_x,1))
    x_temp[0:6,0] = self.x_prior[0:6,0]
    x_temp[6:15,0] = self.x_prior[7:16,0]
    hx = np.dot(self.H,x_temp)

    self.y = residual(self.z[0:6,0], hx.T).T
#         set_trace()
    q_estimate = Quaternion(self.x_prior[6:10,0]) ## w,x,y,z
    q_measurement = Quaternion(self.z[6],self.z[3],self.z[4],self.z[5]) # w,x,y,z
    e__ = ((q_measurement*q_estimate.inverse)) ## check
    e__log = Q_log(e__.elements)#
#         set_trace()
    self.y[3:6,0] = [e__log[0],e__log[1],e__log[2]]
    ky = dot(self.K, self.y)
    self.x_post = x_temp + ky # dot(self.K, self.y)
#         print(e__.elements)
    temp_exp_map = exp_map(ky[6:9])
#         print(temp_exp_map)
    e_map = Quaternion([temp_exp_map[3],temp_exp_map[0],temp_exp_map[1],temp_exp_map[2]]) * self.x_prior[6:10,0]
#         e_map = Quaternion(exp_map(ky[6:9])) * self.x_prior[6:10,0]
#         self.x_post = x_temp.copy()
    self.x_post[6:9,0] = e_map.elements[1:4]
    self.x_q = e_map
#         self.x_q[1:3,0] = self.x_post[1:3,0]
#         self.x_q[0,0] = temp__[0]
    # print(self.x[0:3].T)
    # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
    # and works for non-optimal K vs the equation
    # P = (I-KH)P usually seen in the literature.
    I_KH = self._I - dot(self.K, self.H)
    self.P_post = dot(I_KH, self.P_prior).dot(I_KH.T) + dot(self.K, R).dot(self.K.T)
#         self.P_post = deepcopy(self.P)
    # set to None to force recompute
    self._log_likelihood = None
    self._likelihood = None
    self._mahalanobis = None
    self.x = self.x_post.copy()
    self.P = self.P_post.copy()

  def predict_x(self, u=0):
    ## gravity added -- check again
    temp_ = self.C.T@ (self.sensor_measurements[0:3,0]-self.x[9:12,0])+self.sensor_measurements[3:6,0]
    self.x_prior[0:3,0] = self.x[0:3,0] + self.T_*self.x[3:6,0]+0.5*(self.T_**2)*temp_
    self.x_prior[3:6,0] = self.x[3:6,0] + self.T_*temp_
    temp = exp_map(self.T_/2*(self.sensor_measurements[6:9,0]-self.sensor_measurements[13:16,0]))
    temp_q = Quaternion(temp[3],temp[0],temp[1],temp[2]) ##w,x,y,z
    self.x_prior[6:10,0] = (temp_q * self.x_q).elements
    self.x_prior[10:13,0]= self.x[9:12,0]
    self.x_prior[13:16,0]= self.x[12:15,0]

  def predict(self, u=0):
    self.set_F()
    self.predict_x()
    Q_ = self.T_ * self.F @ self.L @ self.Q @ self.L.T @ self.F.T
    self.P_prior = dot(self.F, self.P).dot(self.F.T) + Q_#self.Q

  def partial_update(self,gamma,beta):
    for i in range(self.dim_x):
      self.x[i] = gamma[i]*self.x_post[i] + (1-gamma[i])*self.x_prior[i]
      for j in range(self.dim_x):
        self.P[i,j] = gamma[i]*gamma[j]*self.P_post[i,j]+(1-gamma[i]*gamma[j])*self.P_prior[i,j]

  def set_F(self):
    self.F = np.eye(self.dim_x)
    self.F[0:3,3:6] = self.T_*np.eye(3)
    self.F[3:6,6:9] = -self.T_* self.C.T @ get_skew_symm_X(self.sensor_measurements[0:3,0])
    self.F[3:6,9:12] = -self.T_* self.C.T
    self.F[6:9,6:9] = np.eye(3) - self.T_*get_skew_symm_X(self.sensor_measurements[6:9,0])
    self.F[6:9,12:15] = - self.T_*np.eye(3)

  def set_G(self,W,Omega,V):
    self.G = np.zeros((self.dim_x,9))
    self.G[0:3,0:3] = ((self.T_**3)/6)*np.eye(3)
    self.G[3:6,0:3] = ((self.T_**2)/2)*np.eye(3)
    self.G[6:9,0:3] = self.T_*np.eye(3)
    self.G[6:9,3:6] = self.T_*V
    self.G[9:12,6:9] = self.T_*np.eye(3)
    self.G[12:16,3:6] = (self.T_/2)*(np.sin(norm(W)*self.T_/2)/norm(W)) * Omega
    self.G[16:19,3:6] = self.T_*np.eye(3)

  def get_sensor_measurements(self,acceleration,gravity,gyro,quat,gyro_bias):
    if gravity!=None:
      self.sensor_measurements[3:6,0] = gravity
    self.sensor_measurements[0:3,0] = acceleration
    self.sensor_measurements[6:9,0] = gyro
    self.sensor_measurements[9:13,0] = quat
    self.sensor_measurements[13:16,0] = gyro_bias

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






def main():
  imu_dir = r"/home/smerx/git/QEKF/data/5/imu"
  acceleration = read_utari_data_raw(imu_dir)

  t__ = acceleration[1:-1,0] - acceleration[0:-2,0]
  print(acceleration.shape[0])


  t__[1000:5000].mean()
  # plt.plot(acceleration[:-1,1])
  # plt.plot(acceleration[:-1,2])
  # plt.plot(acceleration[:-1,3])
  st = 200000

  plt.plot(t__[st:st+50000])
  # plt.plot(acceleration[:-1,0])
  plt.show()



  # Read utari data
  imu_dir = r"/home/smerx/git/QEKF/data/5/imu"
  vicon_file = r"/home/smerx/git/QEKF/data/5/vicon/vi.csv"
  acceleration, gyro, quat_, gyro_bias,translation, rotation = read_utari_data(imu_dir, vicon_file)
  fig = plt.figure()
  t=2500
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(translation[0,0],translation[0,1,],translation[0,2], s=200,marker='*',c='g')
  ax.scatter(translation[1:t,0],translation[1:t,1,],translation[1:t,2], marker='.',c='b',label='vicon')
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.legend()
  plt.show()
  ## plot database
  # fig = plt.figure()
  # ax = fig.add_subplot(111, projection='3d')
  # ax.scatter(translation[0,0],translation[0,1,],translation[0,2], marker='*',c='g')
  # ax.scatter(translation[1:t,0],translation[1:t,1,],translation[1:t,2], marker='.',c='b',label='vicon')
  # ax.set_xlabel('X')
  # ax.set_ylabel('Y')
  # ax.set_zlabel('Z')
  # plt.legend()
  # plt.show()


  # t=-1
  # plt.figure('Blue-vicon, Red-imu')
  # # plt.title('Blue-Ground Truth, Red-Tracker')
  # plt.subplot(311)
  # plt.plot(acceleration[1:t,0], marker='.',c='r')
  # plt.plot(translation[1:t,0], marker='.',c='b')
  # plt.title('X')

  # plt.subplot(312)
  # plt.plot(acceleration[1:t,1], marker='.',c='r')
  # plt.plot(translation[1:t,1], marker='.',c='b')
  # plt.title('Y')

  # plt.subplot(313)
  # plt.plot(acceleration[1:t,2], marker='.',c='r')
  # plt.plot(translation[1:t,2], marker='.',c='b')
  # plt.title('Z')

  plt.figure('Blue-vicon, Red-imu 2 ')
  # plt.title('Blue-Ground Truth, Red-Tracker')
  plt.subplot(311)
  # plt.plot(quat_old[1:t,0], marker='.',c='r')
  plt.plot(quat_[1:t,0], marker='.',c='b')
  # plt.plot(rotation[1:t,0], marker='.',c='b')
  plt.title('X')

  plt.subplot(312)
  # plt.plot(quat_old[1:t,1], marker='.',c='r')
  plt.plot(quat_[1:t,1], marker='.',c='b')
  # plt.plot(rotation[1:t,1], marker='.',c='b')
  plt.title('Y')

  plt.subplot(313)
  # plt.plot(quat_old[1:t,2], marker='.',c='r')
  plt.plot(quat_[1:t,2], marker='.',c='b')
  # plt.plot(rotation[1:t,2], marker='.',c='b')
  plt.title('Z')

  ## fix imu vicon orientation
  i=2000
  imu_q = Quaternion(quat_[i,3],quat_[i,0],quat_[i,1],quat_[i,2])
  vicon_q = Quaternion(rotation[i,3],rotation[i,1],rotation[i,0],rotation[i,2])

  new_q = imu_q * vicon_q.conjugate
  print('new quaternion from imu & vicon data: '+str(new_q))

  pdb.set_trace()

  # #fix quaternion sign change
  # def avoidQjumps(q):
  #     q_new = q.copy()
  #     for i in range(1,q.shape[0]):
  #         if norm(q_new[i,:]-q[i-1,:]) < norm((q_new[i,:]+q[i-1,:])):
  #             q_new[i-1,:] = -q_new[i-1,:]
  #     return q_new

  # quat_old = quat_.copy()
  # quat_ = avoidQjumps(quat_old)

  # for i range(quat_.shape[1]):
  #     t_ = np.diff(quat_[:,i])
  #     loc = np.where(t_)

  start_=0
  t=50000
  tx = np.diff(quat_[:,0])
  ty = np.diff(quat_[:,1])
  tz = np.diff(quat_[:,2])
  # plt.plot(tx[start_:t])
  # plt.show()
  medi = np.median(np.abs(ty[start_:t] ))
  loc = np.where(ty[start_:t] > medi )

  quat_old = quat_.copy()
  quat_new = quat_.copy()
  max_ = np.max(np.abs(ty[start_:t] ))
  loc = np.where(np.abs(ty[start_:t]) > 0.5 )
  sign =-1

  for i in range(loc[0].shape[0]-1):
    quat_new[loc[0][i]:loc[0][i+1],:] *= sign
    sign*=-1

  # quat_ = quat_old.copy()
  t=30000
  start_= 0
  plt.figure()
  plt.subplot(311)
  plt.plot(quat_new[start_:t,0], marker='.',c='g')
  plt.title('X')
  # plt.plot(tx[start_:t])
  plt.plot(quat_new[start_:t,3], marker='.',c='y')
  # plt.title('X')


  plt.subplot(312)
  plt.plot(quat_new[start_:t,1], marker='.',c='g')
  plt.title('Y')
  # plt.plot(ty[start_:t])
  # plt.plot(loc[0])

  plt.subplot(313)
  plt.plot(quat_new[start_:t,2], marker='.',c='g')
  plt.title('Z')
  # plt.plot(tz[start_:t])

  plt.show()

  ## main EKF program
  drk = ExtendedKalmanFilter(dim_x=15, dim_z=6)
  drk.x_prior = zeros((drk.dim_x+1,1))
  # sensor time period -- ask gans -- check again
  drk.T_ = 1/100
  drk.H = np.zeros((6,15))

  '''------>>>>>'''

  # imu_q = Quaternion(quat_[0,3],quat_[0,0],quat_[0,1],quat_[0,2])
  # vicon_q = Quaternion(rotation[0,3],rotation[0,0],rotation[0,1],rotation[0,2])

  # new_q = imu_q * vicon_q.conjugate
  # print(new_q)
  # r = R.from_quat(rotation[i,:])
  ## number of frames or iterations to run the qekf
  n=50000
  start_= 0

  track = np.zeros((start_+n,3))
  track_q = np.zeros((start_+n,3))
  track_v = np.zeros((start_+n,3))

  ## initilizatiuon
  drk.x = np.zeros((drk.dim_x,1))+.00001
  drk.x[0] = translation[start_,0]
  drk.x[1] = translation[start_,1]
  drk.x[2] = translation[start_,2]
  drk.x[6] = quat_[start_,0]
  drk.x[7] = quat_[start_,1]
  drk.x[8] = quat_[start_,2]

  ## Noise parameter initialization
  q_ = np.array([0, 1.5e-6, 0.5e-6, 0.5e-6, 0.1e-6])
  r_ = np.array([1, 1]) *0.1e-5

  drk.Q = np.diag(np.array([q_[0], q_[0],q_[0],\
              q_[1],q_[1],q_[1],\
              q_[2],q_[2],q_[2],\
                          q_[3],q_[3],q_[3],\
                          q_[4],q_[4],q_[4]]))

  drk.R = np.diag(np.array([r_[0],r_[0],r_[0],\
                          r_[1],r_[1],r_[1]]))

  # x_prior_history = zeros((start_+n,6))
  drk.x_q = Quaternion([quat_[0,3], quat_[0,0], quat_[0,1], quat_[0,2]])
  drk.P *= 1e-4
  drk.sensor_measurements = np.zeros((16,1)) # acceleration 3, gyro 3, gravity 3, quat 4
  drk.z = zeros((7,1))
  for i in range(start_,start_+n-1):
    track_q[i,:] = drk.x[6:9,0]
    track[i,:] = drk.x[0:3,0]
    track_v[i,:] = drk.x[3:6,0]
    drk.get_sensor_measurements(acceleration[i,:],None,gyro[i,:],quat_[i,:],gyro_bias)
    drk.C = transform_(quat_[i,:])#transform_(rotation[i,:])#
    #drk.C = Quaternion([quat_[i,3],quat_[i,0],quat_[i,1],quat_[i,2]]).rotation_matrix
    ## Measurement Matrix  -- check again
    drk.H[0:3,3:6] = drk.C.T
    drk.H[0:3,6:9] = -drk.C.T @ get_skew_symm_X(drk.x[3:6,0])
    drk.H[3:6,6:9] = np.eye(3)

    ## QEKF2 L matrix
    drk.L = np.eye(15)
    drk.L[3:6,3:6] = -drk.C.T
    drk.L[0:3,0:3] = 0
    drk.L[6:9,6:9] = -np.eye(3)
    drk.predict()

    v_norm = np.array([norm(drk.x_prior[3:6,0]),0,0])
    v = v_norm #drk.C @ v_norm
    #v = drk.C @ drk.x[3:6,0]
#     drk.x_prior[5,0] = 0.001
    drk.z[0,0] = drk.x_prior[3,0]
    drk.z[1,0] = 0#drk.x_prior[4,0]#
    drk.z[2,0] = drk.x_prior[5,0]#
    drk.z[3:7,0] = quat_[i]#rotation[i,:] #quat_[i] #x,y,z,w

#     x_prior_history[i,0:3] =drk.x_prior[0:3,0]
#     x_prior_history[i,3:6] =drk.x_prior[3:6,0]
    drk.update(drk.z.T)


  pdb.set_trace()
  acceleration[start_:t,0]

  # plt.figure()
  gt = translation
  # gt = rotation
  n=30000
  t=start_+n-2
  # ax.set_xlim([-2,2])
  # ax.set_ylim([-2,2])
  # ax.set_zlim([-2,2])

  plt.figure('Blue-Ground Truth, Red-Tracker')
  # plt.title('Blue-Ground Truth, Red-Tracker')
  plt.subplot(311)
  plt.plot(track[start_:t,0], marker='.',c='r')
  # plt.plot(acceleration[start_:t,0], marker='.',c='m')
  # plt.plot(gt[start_:t,0], marker='.',c='b')
  plt.title('X')

  plt.subplot(312)
  plt.plot(track[start_:t,1], marker='.',c='r')
  # plt.plot(acceleration[start_:t,1], marker='.',c='m')
  # plt.plot(gt[start_:t,1], marker='.',c='b')
  plt.title('Y')

  plt.subplot(313)
  plt.plot(track[start_:t,2], marker='.',c='r')
  # plt.plot(acceleration[start_:t,2], marker='.',c='m')
  # plt.plot(gt[start_:t,2], marker='.',c='b')
  plt.title('Z')


  # bx.set_xlim([-2,2])
  # bx.set_ylim([-2,2])
  # bx.set_zlim([-2,2])

  # plt.figure("GT")


  plt.show()

  # np.save('ekf',trk)`



  '''---------------->>>>>>>>>>>>>>>>>'''
  plt.figure("Orientaion Green-Phone Imu Red-EKF")
  # gt = translation

  t=start_+n-2

  # gt_q = rotation.copy()
  plt.subplot(311)
  plt.plot(track_q[start_:t,0], marker='.',c='r')
  # plt.plot(quat_[start_:t,3], marker='.',c='m')
  plt.plot(quat_[start_:t,0], marker='.',c='g')

  # plt.plot(gt_q[start_:t,0], marker='.',c='b')
  # plt.plot(gt_q[1:t,1], marker='.',c='m')
  # plt.plot(gt_q[1:t,3], marker='.',c='y')
  # plt.plot(gt[1:t,3], marker='.',c='y')
  plt.title('X')

  plt.subplot(312)
  plt.plot(track_q[start_:t,1], marker='.',c='r')
  plt.plot(quat_[start_:t,1], marker='.',c='g')
  # plt.plot(gt_q[start_:t,1], marker='.',c='b')
  # plt.plot(gt_q[1:t,0], marker='.',c='m')
  plt.title('Y')

  plt.subplot(313)
  plt.plot(track_q[start_:t,2], marker='.',c='r')
  plt.plot(quat_[start_:t,2], marker='.',c='g')
  # plt.plot(gt_q[start_:t,2], marker='.',c='b')
  plt.title('Z')


  # bx.set_xlim([-2,2])
  # bx.set_ylim([-2,2])
  # bx.set_zlim([-2,2])

  # plt.figure("GT")


  plt.show()

  # np.save('ekf',trk)

  start_=0
  t=30000
  plt.figure()
  plt.plot(track_q[start_,0],track_q[start_,2],markersize=12, marker='o',c='b')
  plt.plot(track_q[start_:t,0],track_q[start_:t,2], marker='.',c='r')

  plt.show()


  t=60000
  step = 10
  fig = plt.figure()
  # # # plt.title('Reference Frame position')
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(track[start_,0],track[start_,1,],track[start_,2],s=200, marker='*',c='g',)
  ax.scatter(track[start_:t:step,0],track[start_:t:step,1],track[start_:t:step,2], marker='.',c='r',label='ekf')
  ax.plot(track[start_:t:step,0],track[start_:t:step,1],track[start_:t:step,2], marker='.',c='r',label='ekf')

  # ax.scatter(gt[start_,0],gt[start_,1,],gt[start_,2], marker='*',c='g')
  # ax.scatter(gt[start_:t,0],gt[start_:t,1],gt[start_:t,2], marker='.',c='b',label='vicon')
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.legend()
  set_axes_equal(ax)
  # ax.set_xlim([-2,2])
  # ax.set_ylim([-2,2])
  # ax.set_zlim([-2,2])
  # # # ax.scatter(gt[0,0],gt[0,1,],gt[0,2], marker='*',c='r')
  # # # ax.scatter(gt[:,0],gt[:,1,],gt[:,2], marker='.',c='g',label='slam pose')


  # # #2d plot
  # # # plt.plot(trk[0,0],trk[0,1,], marker='o',c='b')
  # plt.plot(trk[1:,0],trk[1:,1,], marker='.',c='r',label='ekf-world-frame')
  # plt.legend()

  # plt.figure()
  # plt.plot(track[0,0],track[0,1], marker='o',c='r')
  # plt.plot(track[1:t,0],track[1:t,1], marker='.',c='b',label='ekf pos')

  # # # plt.scatter(gt[:,0],gt[:,1,],gt[:,2], marker='.',c='g',label='slam pose')

  # plt.legend()


  # fig_2 = plt.figure()
  # bx = fig_2.add_subplot(111, projection='3d')
  # bx.scatter(gt[0,0],gt[0,1,],gt[0,2], marker='*',c='r')
  # bx.scatter(gt[1:t,0],gt[1:t,1,],gt[1:t,2], marker='.',c='g',label='vicon pose')
  # bx.set_xlabel('X')
  # bx.set_ylabel('Y')
  # bx.set_zlabel('Z')
  # bx.set_xlim([-2,2])
  # bx.set_ylim([-2,2])
  # bx.set_zlim([-2,2])

  # plt.figure()
  # plt.plot(gt[0,0],gt[0,1], marker='o',c='r')
  # plt.plot(gt[1:t,0],gt[1:t,1], marker='.',c='g',label='vicon pos')
  # # # ax.scatter(gt[:,0],gt[:,1,],gt[:,2], marker='.',c='g',label='slam pose')



  # # plt.legend()
  # fig = plt.figure()
  # plt.title('Tracking error')
  # plt.plot(diff[:-1,0:3])
  # # plt.plot(diff[:-1,3],'g')
  # plt.legend()



  plt.show()

  # np.save('ekf',trk)

  # trk = track[1:i,1:4]
  # trk = track[1:i,:]
  # print(trk.shape)
  # time = np.arange(0, len(xs)*drk.T_, drk.T_)
  t=n-15
  # t=500
  # # 3d plot
  fig = plt.figure()

  # # #2d plot
  plt.plot(track[0,0],track[0,1,], marker='o',c='b')
  plt.plot(track[1:,0],track[1:,1,], marker='.',c='r',label='ekf')
  plt.legend()

  # plt.figure()
  plt.plot(gt[0,0],gt[0,1], marker='o',c='r')
  plt.plot(gt[1:t,0],gt[1:t,1], marker='.',c='g',label='vicon pos')
  # # # ax.scatter(gt[:,0],gt[:,1,],gt[:,2], marker='.',c='g',label='slam pose')
  plt.legend()
  plt.show()

  track[0,0],track[0,1,], gt[0,0],gt[0,1],translation[0,:]

  return # end of main

if __name__ == "__main__":
  main()
