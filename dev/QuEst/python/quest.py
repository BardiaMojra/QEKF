''' global QuEst config
'''

''' select QuEst '''
# from quest_algs.QuEst_v0708_spd import QuEst_5Pt_Ver7_8_spd as QuEst
from quest_algs.QuEst_v0708 import QuEst_5Pt_Ver7_8 as QuEst
from quest_algs.QuEst_v0708 import get_qs_residues as get_residues
# from quest_algs import QuEst_RANSAC_v0102 as RQuEst

''' select rQuEst '''
from ransac_h import ransac_QuEst_h as rQuEst


''' select get Trans Depth '''
from quest_algs.QuEst_getTransDepth_v0100 import get_Txyz_v0100 as get_Txyz

# Coefs Alg

import numpy as np
''' module configs '''
OUT_DIR = '../out00/'
DATA_ROOT = '../Datasets/'



# EOF
