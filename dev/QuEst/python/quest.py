import numpy as np

''' global QuEst config '''
# QuEst alg
from quest_algs.QuEst_v0708_spd import QuEst_5Pt_Ver7_8_spd as Q0708spd
from quest_algs.QuEst_v0708 import QuEst_5Pt_Ver7_8 as Q0708
from quest_algs import QuEst_RANSAC_v0102 as QRANSAC0102
# Coefs Alg
# from quest_algs.coefs import CoefsVer3_1_1 as COEFS_V0311

''' module configs '''
OUT_DIR = '../out00/'
DATA_ROOT = '../Datasets/'
_DTYPE = np.float64


# EOF
