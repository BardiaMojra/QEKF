

import pandas as pd
import json

from util_dr import *
from nbug import *




''' module config
'''
VICON__DATA_FNAME = 'vi_clean.csv'
VICON__DATA_FNAME = 'vicon_test_001.json'
# imuDir = r"../data/dead_reckoning_data/1/imu/"
viconDir = "../data/dead_reckoning_data/1/vicon/"


with open('path_to_file/person.json') as f:
  data = json.load(f)

print(data)
end_epoch=855981016016
start_epoch=540666478166

fname =
vicon_df = pd.read_csv(viconDir+fname, header=0)
# vicon_df.drop(labels=['Frame'], axis=1, inplace=True)
# start_epoch = get_unix_time_from_str(START_TIME)
# end_epoch = get_unix_time_from_str(END_TIME)
numSamps = vicon_df.shape[0]

nprint('start_epoch', start_epoch)
nprint('end_epoch', end_epoch)
timestamps = get_epoch_series(start_epoch, end_epoch, numSamps)
vicon_df.index = timestamps
vicon_df.to_csv(viconDir+'vi_w_timestamps.txt', sep=',', mode='w', header=False)
nsprint('vicon_df.head(5)', vicon_df.head(5))
