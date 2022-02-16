

import pandas as pd
import json

from util import *
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
DATA_FILE = 'vi_clean.csv'
OUT_FILE  = 'vi_w_timestamps.txt'
CONFIG_FILE = 'vi.json'
DIR = "../data/test_001_vicon_training_day/vicon/"

fname = DIR+CONFIG_FILE
with open(fname) as f:
  config = json.load(f)
  f.close()

nprint('config', config)
start_epoch = get_unix_time_from_str(config['start_time'])
end_epoch = get_unix_time_from_str(config['end_time'])
nprint('start_epoch', start_epoch)
nprint('end_epoch', end_epoch)
# st()
# end_epoch=855981016016
# start_epoch=540666478166

fname = DIR+DATA_FILE
vicon_df = pd.read_csv(fname, header=0)
vicon_df.drop(labels=['Frame'], axis=1, inplace=True)
numSamps = vicon_df.shape[0] - 1
# st()
timestamps = get_epoch_series(start_epoch, end_epoch, numSamps)
vicon_df.index = timestamps

vicon_df = toQuat_set(vicon_df)

vicon_df.to_csv(DIR+OUT_FILE, sep=',', mode='w', header=False)
nsprint('vicon_df.head(5)', vicon_df.head(5))
nsprint('vicon_df.tail(5)', vicon_df.tail(5))






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
