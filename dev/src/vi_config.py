

import json
from nbug import *
from pdb import set_trace as st


class Config():
  def __init__(self,
               test_id:int,
               description:str,
               capture_Hz:int,
               start_time:str,
               end_time:str,
               units:str,
               ):
    self.test_id = str(test_id)
    self.description = description
    self.capture_Hz = str(capture_Hz)
    self.start_time = start_time
    self.end_time = end_time
    self.units = units

  def toJSON(self):
    return json.dumps(self, default=lambda o: o.__dict__,\
            sort_keys=True, indent=1)
