import numpy as np
import pandas as pd
from pdb import set_trace as st
import csv
from pprint import pprint as pp

def read_utari_data_pandas(imu_dir, vicon_file):
  acc_file = open(imu_dir+'/acce.txt', "r")
  gyro_file = open(imu_dir+'/gyro.txt', "r")
  quat_file = open(imu_dir+'/rv.txt', "r")

  acc_lines = acc_file.readlines()
  gyro_lines = gyro_file.readlines()
  quat_lines = quat_file.readlines()

  df_acc = pd.read_csv(imu_dir+'/acce.txt', sep=' ', header=0)
  df_acc = df_acc.iloc[:, [0,1,2,3]]
  df_acc.columns = ["t", "Tx", "Ty", "Tz"]
  print('df_acc:')
  pp(df_acc)

  df_gyro = pd.read_csv(imu_dir+'/gyro.txt', sep=' ', header=0)
  df_gyro = df_acc.iloc[:, [0,1,2,3]]
  df_gyro.columns = ["t", "wr", "wp", "wy"]
  print('df_gyro:')
  pp(df_gyro)

  #st()

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
  print('_gyro:')
  pp(_gyro)
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
