import numpy as np
import quaternion as quaternion
from numpy import exp as exp

from pprint import pprint as pp

quaternion(1,0,0,0)
q1 = quaternion(1,2,3,4)
q2 = quaternion(5,6,7,8)
q3 = q1 * q2
pp(q3)
a = np.array([q1, q2])
pp(a)

pp(exp(a))
