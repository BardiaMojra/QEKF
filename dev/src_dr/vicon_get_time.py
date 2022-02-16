

import pandas as pd
import json

from util_dr import *
from nbug import *




''' module config
'''
DATA_FILE = 'vi_clean.csv'
CONFIG_FILE = 'vi.json'
DIR = "../data/test_001_vicon_training_day/vicon/"

fname = DIR+CONFIG_FILE
with open(fname) as f:
  config = json.load(f)

nprint('config', config)
st()
end_epoch=855981016016
start_epoch=540666478166

fname = DIR+DATA_FILE
vicon_df = pd.read_csv(, header=0)
vicon_df.drop(labels=['Frame'], axis=1, inplace=True)
# start_epoch = get_unix_time_from_str(START_TIME)
# end_epoch = get_unix_time_from_str(END_TIME)
numSamps = vicon_df.shape[0]

nprint('start_epoch', start_epoch)
nprint('end_epoch', end_epoch)
timestamps = get_epoch_series(start_epoch, end_epoch, numSamps)
vicon_df.index = timestamps
vicon_df.to_csv(DIR+'vi_w_timestamps.txt', sep=',', mode='w', header=False)
nsprint('vicon_df.head(5)', vicon_df.head(5))
