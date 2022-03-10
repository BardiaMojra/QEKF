import numpy as np

def Find_LinearVelocity(m, m_dot, omega):
    omega = omega.reshape((3,1))
    Identity_Matrix     = np.identity(3) 
    Num_Points          = m.shape[1]
    Num_Candidates      = omega.shape[1]
    v                   = np.zeros((3,Num_Candidates))              # linear velocity candidates 
    u                   = np.zeros((Num_Points,Num_Candidates))     # depth candidates 
    u_dot               = np.zeros((Num_Points,Num_Candidates))     # depth derivative candidates 
    depth               = np.zeros((2*Num_Points, Num_Candidates))

    for k in range(Num_Candidates):

        omega_skew = np.array([[0, -omega[2,k], omega[1,k]], [omega[2,k], 0, -omega[0,k]], [-omega[1,k], omega[0,k], 0]])
        # build C matrix, size: 15x13
        C = np.zeros((3*Num_Points, 2*Num_Points+3))
        
        for i in range(Num_Points):
            C[i*3:(i+1)*3, 0:3] = Identity_Matrix 
            # omega_skew @ m_dot[:,1] has size (3,) instead of (3,1) for some reason, so we use stack instead of concatenate
            C[i*3:(i+1)*3, i*2+3:i*2+5] = np.stack((np.matmul(omega_skew,m[:,i]) - m_dot[:,i], -1 * m[:,i]), axis=1)
        
        # null space of C 
        # using the smallest eige value from C gives us the the one most
        # likely to give 0 when we do (lambda)*v, where v is the last
        # column in V of the svd of C
        U, S, V = np.linalg.svd(C) 
        # Matlab code transposes V automatically
        V = V.T

        # ~ V = np.divide(V, V[3,-1]) # moved to velest file

        # first three rows of last column of V are v_x, v_y, v_z 
        v_temp = V[0:3, -1]
        depth_temp = V[3:13, -1] 

        v[:,k] = v_temp 
        depth[:,k] = depth_temp 
        

    return (v, depth, C, V)    
    
    


def main():
    m = []
    for i in range(3):
        k = []
        for j in range(5):
            k.append(5*i + j + 1)
        m.append(k)

    m = np.array(m)
    m[1,2] = 10
    m[2,2] = 100

    m_dot = m + 4
    omega = np.array([[1],[2],[3]])
    v, d = Find_LinearVelocity(m, m_dot, omega)
    print(v)
    print("-----------------------------------------------------------------------")
    print(d)

if __name__ == "__main__":
    main()
