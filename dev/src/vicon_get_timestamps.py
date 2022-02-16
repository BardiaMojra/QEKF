

from posixpath import split
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd
import json
import datetime


from nbug import *
from vi_config import *

''' to write '''
# test_id         = 1
# description     = 'vicon_training_day'
# capture_rate_Hz = 30
# start_time      = "2022-02-15 14:58:32.402 ST"
# end_time        = "2022-02-15 15:00:21.850 ST"


''' module config
'''
TEST_ = 'test_001_vicon_training_day'

DATA_FILE = 'vi_clean.csv'
OUT_FILE  = 'vi_w_timestamps.txt'
CONFIG_FILE = 'vi.json'
viDir = '../data/'+TEST_+"/vicon/"
imuDir = '../data/'+TEST_+"/imu/"


def main():
  fname = viDir+CONFIG_FILE
  with open(fname) as f:
    config = json.load(f)
    f.close()

  nprint('config', config)
  # start_epoch = get_unix_time_from_str(config['start_time'])
  # end_epoch = get_unix_time_from_str(config['end_time'])
  start_epoch = get_rv_unix_time(imuDir, 'start')
  end_epoch = get_rv_unix_time(imuDir, 'end')
  nprint('start_epoch', start_epoch)
  nprint('end_epoch', end_epoch)

  fname = viDir+DATA_FILE
  vicon_df = pd.read_csv(fname, header=0)
  vicon_df.drop(labels=['Frame'], axis=1, inplace=True)
  numSamps = vicon_df.shape[0] #- 1
  # st()
  timestamps = get_epoch_series(start_epoch, end_epoch, numSamps)
  vicon_df.index = timestamps
  Qxyzw_df = toQuat_set(vicon_df[['RX', 'RY', 'RZ']])
  dfspp('vicon_df', vicon_df)
  dfspp('Qxyzw_df', Qxyzw_df)
  vicon_df.drop(['RX', 'RY', 'RZ'],  axis=1, inplace=True)

  nsprint('Qxyzw_df.head(5)', Qxyzw_df.head(5))
  nsprint('vicon_df.head(5)', vicon_df.head(5))

  vicon_df = pd.concat([Qxyzw_df, vicon_df],  axis=1)

  nsprint('vicon_df.head(5)', vicon_df.head(5))
  nsprint('vicon_df.tail(5)', vicon_df.tail(5))

  dfspp('vicon_df', vicon_df)
  # st()

  vicon_df.to_csv(viDir+OUT_FILE, sep=',', mode='w', header=False)
  print('end of process...')
  plt()
  return # end of main

def get_rv_unix_time(imuDir, ins):

  fname = imuDir+'rv.txt'
  with open(fname, 'r') as f:
    data = f.read().splitlines()
    f.close()
    if ins == 'start': i = 0;
    if ins == 'end': i = -1;
    line = data[i]
    utime = line.split(' ')
  return int(utime[0])



def toQuat_set(Rxyz_df:pd.DataFrame):
  Qlabels = ['Qx','Qy','Qz','Qw']
  nprint('Rxyz_df.columns', Rxyz_df.columns)
  nprint('Qlabels', Qlabels)
  Rxyz_np = Rxyz_df.to_numpy()
  rot_np = R.from_rotvec(Rxyz_np)
  Qxyzw_np = rot_np.as_quat()
  Qxyzw_df = pd.DataFrame(Qxyzw_np, columns=Qlabels, index=Rxyz_df.index)
  return Qxyzw_df

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


############ protos
# fname = DIR+CONFIG_FILE
# with open(fname) as f:
#   config = json.load(f)
#   f.close()
# nprint('config', config)
# # config2 = Config(test_id,description,capture_rate_Hz,start_time,end_time )
# # fname = DIR+'vi3.json'
# with open(fname, 'w') as f:
#     json.dump(config2.toJSON(), f)
#     f.close()


if __name__ == '__main__':

  main()
