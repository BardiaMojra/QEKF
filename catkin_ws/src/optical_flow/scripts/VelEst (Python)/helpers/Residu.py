import numpy as np
from Find_C_Matrix import *


def Residu(m, m_dot, omega):

    # Each column of the residue matrix is the polynomial we want to be 0 (from
    # double page) so we sum each column and in PoCo pick the minimum residue
    # as our candidate solution
    C0 = Find_C(m, m_dot)     # size of C0: 20x20
    omega_x = omega[0,:] 
    omega_y = omega[1,:] 
    omega_z = omega[2,:] 
   

    xVec = np.stack((np.power(omega_x, 3), np.power(omega_y, 3), np.power(omega_z, 3), np.power(omega_x, 2)*omega_y, np.power(omega_x, 2)*omega_z, np.power(omega_y, 2)*omega_x, np.power(omega_y, 2)*omega_z, np.power(omega_z, 2)*omega_x, np.power(omega_z, 2)*omega_y, omega_x*omega_y*omega_z, np.power(omega_x, 2), np.power(omega_y, 2), np.power(omega_z, 2), omega_x*omega_y, omega_x*omega_z, omega_y*omega_z, omega_x, omega_y, omega_z, np.ones((20,))), axis=0) 
    
    residuMat = np.matmul(C0, xVec)      # size of residuMat: 20x20 
    # take absolute value first because each element of the matrix represents an error (the entire polynomial)
    residu = np.sum(np.absolute(residuMat), axis=0) 
    return residu



def main():
    m = np.array([[1,2,3,4,5],[6,7,8,9,10],[11,12,13,14,15]])
    m_dot = m+3

    omega = np.stack((np.ones(20,), np.ones(20,)*2, np.ones(20,)*3), axis=0)
    print(omega)
    omega[0,0] = 5
    omega[1,10] = 0
    r = Residu(m, m_dot, omega)


if __name__ == "__main__":
    main()
