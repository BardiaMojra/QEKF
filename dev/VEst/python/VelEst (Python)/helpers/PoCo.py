import numpy as np
from Find_A_Matrix import *
from Find_B_Matrix import *
from Find_C_Matrix import *
from Find_AngularVelocity import *
from Find_LinearVelocity import *
from Residu import *
from Large_C_Matrix import *

from random import sample

np.set_printoptions(threshold=10000, precision=8)


def PoCo(m,m_dot, R_mat, N_mat, UseSVD, UseBigC):
    
    #inner_threshold = 0.35
    #outer_threshold = 1
    if m.shape[1] < 6 or R_mat.shape != (3,3) or N_mat.shape != (3,5):
        return (None, None, None, None, None)
    index_m_mdot = sample(range(m.shape[1]), 6)
    
    #index_m_mdot = [0,1,2,3,4,5]

    m6 = np.stack([m[:, i] for i in index_m_mdot], axis=1)
    m6_dot = np.stack([m_dot[:, i] for i in index_m_mdot], axis=1)
    
    m5 = m6[:,:-1]
    m5_dot = m6_dot[:,:-1]
    
    A = Find_A(m5, m5_dot)          # size of A: 30x34, A is coefficient matrix 

    # The cases are wy/wx, wz/wx, wx/wy, wz/wy, wx/wz, wy/wz since we don't
    # know which eigenvalue will be correct we choose the one that yields
    # the residue closest to 0

    ## case 1 
    B1 = Find_B(A,1)                             # size of B: 20x20, B is used for solving eigenvectors 
    omega1, lamda1 = Find_AngularVelocity(B1,1)       # find angular velocities, 20 solution candidates 
    
    omega1 = np.matmul(R_mat, omega1)

    res1 = Residu(m6, m6_dot, omega1)                     # find the best result out of 20 solutions 
    min_val1 = np.nanmin(np.absolute(res1))
    min_idx1 = np.nanargmin(np.absolute(res1))
    
    # notes by Cody based on assumptions
    # Find_LinearVelocity original version, uses svd to solve original C matrix
    # find_transform developed by David to solve "large C matrix", this can use
    # either svd or quadratic programing 
    
    if not UseBigC:
        v1,depth1,C1,V1 = Find_LinearVelocity(m5, m5_dot, omega1[:,min_idx1])     # find linear velocity and depth 
    else:
        v1, t1, C1, V1, depth1 = find_transform(m5, m5_dot, omega1[:, min_idx1], N_mat, R_mat, UseSVD)


    ## case 2 
    B2 = Find_B(A,2)                             # size of B: 20x20, B is used for solving eigenvectors 
    omega2, lamda2 = Find_AngularVelocity(B2,2)       # find angular velocities, 20 solution candidates 

    omega2 = np.matmul(R_mat, omega2)

    res2 = Residu(m6, m6_dot, omega2)                     # find the best result out of 20 solutions 
    min_val2 = np.nanmin(np.absolute(res2))
    min_idx2 = np.nanargmin(np.absolute(res2))

    if not UseBigC:
        v2,depth2,C2,V2 = Find_LinearVelocity(m5, m5_dot, omega2[:,min_idx2])     # find linear velocity and depth 
    else:
        v2, t2, C2, V2, depth2 = find_transform(m5, m5_dot, omega2[:, min_idx2], N_mat, R_mat, UseSVD)

    ## case 3 
    B3 = Find_B(A,3)                             # size of B: 20x20, B is used for solving eigenvectors 
    omega3, lamda3 = Find_AngularVelocity(B3,3)       # find angular velocities, 20 solution candidates 
    
    omega3 = np.matmul(R_mat, omega3)
    
    res3 = Residu(m6, m6_dot, omega3)                     # find the best result out of 20 solutions 
    min_val3 = np.nanmin(np.absolute(res3))
    min_idx3 = np.nanargmin(np.absolute(res3))

    if not UseBigC:
        v3,depth3,C3,V3 = Find_LinearVelocity(m5, m5_dot, omega3[:,min_idx3])     # find linear velocity and depth 
    else:
        v3, t3, C3, V3, depth3 = find_transform(m5, m5_dot, omega3[:, min_idx3], N_mat, R_mat, UseSVD)

    ## case 4 
    B4 = Find_B(A,4)                             # size of B: 20x20, B is used for solving eigenvectors 
    omega4, lamda4 = Find_AngularVelocity(B4,4)       # find angular velocities, 20 solution candidates 

    omega4 = np.matmul(R_mat, omega4)

    res4 = Residu(m6, m6_dot, omega4)                     # find the best result out of 20 solutions 
    min_val4 = np.nanmin(np.absolute(res4))
    min_idx4 = np.nanargmin(np.absolute(res4))

    if not UseBigC:
        v4,depth4,C4,V4 = Find_LinearVelocity(m5, m5_dot, omega4[:,min_idx4])     # find linear velocity and depth 
    else:
        v4, t4, C4, V4, depth4 = find_transform(m5, m5_dot, omega4[:, min_idx4], N_mat, R_mat, UseSVD)

    ## case 5 
    B5 = Find_B(A,5)                             # size of B: 20x20, B is used for solving eigenvectors 
    omega5, lamda5 = Find_AngularVelocity(B5,5)       # find angular velocities, 20 solution candidates 

    omega5 = np.matmul(R_mat, omega5)

    res5 = Residu(m6, m6_dot, omega5)                     # find the best result out of 20 solutions 
    min_val5 = np.nanmin(np.absolute(res5))
    min_idx5 = np.nanargmin(np.absolute(res5))

    if not UseBigC:
        v5,depth5,C5,V5 = Find_LinearVelocity(m5, m5_dot, omega5[:,min_idx5])     # find linear velocity and depth 
    else:
        v5, t5, C5, V5, depth5 = find_transform(m5, m5_dot, omega5[:, min_idx5], N_mat, R_mat, UseSVD)

    ## case 6 
    B6 = Find_B(A,6)                             # size of B: 20x20, B is used for solving eigenvectors 
    omega6, lamda6 = Find_AngularVelocity(B6,6)       # find angular velocities, 20 solution candidates 

    omega6 = np.matmul(R_mat, omega6)

    res6 = Residu(m6, m6_dot, omega6)                     # find the best result out of 20 solutions 
    min_val6 = np.nanmin(np.absolute(res6))
    min_idx6 = np.nanargmin(np.absolute(res6))

    if not UseBigC:
        v6,depth6,C6,V6 = Find_LinearVelocity(m5, m5_dot, omega6[:,min_idx6])     # find linear velocity and depth 
    else:
        v6, t6, C6, V6, depth6 = find_transform(m5, m5_dot, omega6[:, min_idx6], N_mat, R_mat, UseSVD)

    ## results 
    omega1_ans = omega1[:,min_idx1] 
    # v1 
    # 
    omega2_ans = omega2[:,min_idx2] 
    # v2
    # 
    omega3_ans = omega3[:,min_idx3] 
    # v3
    # 
    omega4_ans = omega4[:,min_idx4] 
    # v4
    # 
    omega5_ans = omega5[:,min_idx5] 
    # v5
    # 
    omega6_ans = omega6[:,min_idx6] 
    # v6

    vals = np.array([min_val1, min_val2, min_val3, min_val4, min_val5, min_val6]) 
    val = np.nanmin(vals)
    idx = np.nanargmin(vals)

    if idx == 0: 
        v = v1 
        omega = omega1_ans
        depth = depth1
        C = C1
        V = V1
        if UseBigC:
            t = t1
    elif idx == 1: 
        v = v2 
        omega = omega2_ans 
        depth = depth2
        C = C2
        V = V2
        if UseBigC:
            t = t2
    elif idx == 2:
        v = v3 
        omega = omega3_ans 
        depth = depth3
        C = C3
        V = V3
        if UseBigC:
            t = t3
    elif idx == 3: 
        v = v4 
        omega = omega4_ans 
        depth = depth4
        C = C4
        V = V4
        if UseBigC:
            t = t4
    elif idx == 4: 
        v = v5 
        omega = omega5_ans 
        depth = depth5
        C = C5
        V = V5
        if UseBigC:
            t = t5
    elif idx == 5: 
        v = v6 
        omega = omega6_ans
        depth = depth6
        C = C6
        V = V6
        if UseBigC:
            t = t6
    else:
        print("This should never happen")
        pass
    
    #epsilon = 0.001
    #if np.linalg.norm(omega, 2) < epsilon:
    #    return (None, None)


    # ~ f = open("/home/developer/Desktop/depth.txt", "a")
    #f = open("log/depth.txt", "a")
    #for i in depth:
    #    if i>=0:
    #        f.write("+")
    #    else:
    #        f.write("-")
    #f.write("\n")
    #f.close()


    #f = open("/home/developer/Desktop/air_simulation2/C_best/C_%4d", "w")
    # ~ depth_u, depth_v, depth_prime = depth
    
    #v = np.matmul(R_mat, v)
    
    # ~ print("depth \n%s" %(depth))
    # ~ print("depthu \n%s" %(depth_u))
    # ~ print("depthv \n%s" %(depth_v))
    # ~ print("depthp \n%s" %(depth_prime))
    if not UseBigC:
        t = [0, 0, 0]
    return (v, omega, t, C, V, depth)


def main():
    pass

if __name__ == "__main__":
    main()
