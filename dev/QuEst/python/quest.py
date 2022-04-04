''' global QuEst config
'''

''' select QuEst '''
# from quest_algs.QuEst_v0708_spd import QuEst_5Pt_Ver7_8_spd as QuEst
from quest_algs.QuEst_v0708 import QuEst_5Pt_Ver7_8 as QuEst
from quest_algs.QuEst_v0708 import get_qs_residues as get_residues

''' select rQuEst '''
from ransac_h import QUEST_RANSAC as RQUEST


''' select get Trans Depth '''
from quest_algs.QuEst_getTransDepth_v0100 import get_Txyz_v0100 as get_Txyz

# Coefs Alg





# EOF
