
import numpy as np
import cvxopt as cp

np.set_printoptions(linewidth=1000)

# n is base frame feature points, m is current frame feature points, m_dot is change from previous frame to current frame
def construct_large_C_matrix(R, n, m, m_dot, omega):
    
    NumPoints = m.shape[1]
    Q = np.zeros((6*NumPoints, 6 + 3*NumPoints))

    for i in range(NumPoints):
        Q[6*i:(i+1)*6, 0:6] = np.identity(6)
        K = np.zeros((6, 3))
        t1 = np.matmul(R, n[:,i])
        t2 = np.matmul(omega, m[:,i]) - m_dot[:,i]
        
        t1 = t1.reshape((3,1))
        t2 = t2.reshape((3,1))

        N = -1*n[:,i]
        M = -1*m[:,i]

        N = N.reshape((3,1))
        M = M.reshape((3,1))


        #K = np.concatenate( (np.concatenate((t1, t2), axis = 0), 
        #                     np.concatenate((N, np.zeros((3,1))), axis = 0), 
        #                     np.concatenate((np.zeros((3,1)), M), axis = 0) ), axis = 1)

        K = np.concatenate( (np.concatenate((t1, np.zeros((3,1))), axis = 0), 
                             np.concatenate((M, t2), axis = 0), 
                             np.concatenate((np.zeros((3,1)), M), axis = 0) ), axis = 1)
        
        Q[6*i:(i+1)*6, (6+3*i):(6+3*(i+1)) ] = K


    return Q


def nullspace_SVD(Q):
    U, S, V = np.linalg.svd(Q)
    V = V.T

    result = V[:,-1]
    # ~ result = np.divide(result, result[7])
    # ~ result = np.divide(result, result[6]) # moved to velest file

    t = result[0:3]
    v = result[3:6]
    sub_result = result[6:]
    depths_u = sub_result[0::3]
    depths_v = sub_result[1::3]
    depths_prime = sub_result[2::3]

    #print(result)
    #print(t)
    #print(v)

    #print(depths_u)
    #print(depths_v)
    #print(depths_prime)

    return t, v, Q, V, (depths_u, depths_v, depths_prime)

def nullspace_QP(Q):
    # Ax == b
    A = cp.matrix(Q)
    b = cp.matrix(0.0, (Q.shape[0], 1))

    # Px <= q
    P = cp.matrix(2*np.matmul(Q.T, Q))
    q = cp.matrix(0.0, (Q.shape[1], 1))

    # Gx <= h
    G = np.identity(Q.shape[1])
    for i in range(Q.shape[1]):
        # No specific constraints on v and t
        if i < 6:
            G[i,i] = 0
        # depth derivatives should (sometimes) be negative (1*u_dot < 0)
        elif i%3 == 2:
            G[i,i] = 0 # turned off
        # depths themselves should be positive (-1*u < 0) and (-1*v < 0)
        else:
            G[i,i] = -1

    h = np.zeros(Q.shape[1])
    # To enforce something similar to a strict inequality
    h[:] = 10e-4
    h = h.reshape(-1,1)

    G = cp.matrix(G)
    h = cp.matrix(h)

    cp.solvers.options['show_progress'] = False
    sol = cp.solvers.qp(P, q, G, h)
    result = sol['x']
    result = np.array(result)
    # ~ print("result \n%s" %(result))
    # ~ result = np.divide(result, result[7])
    # ~ result = np.divide(result, result[6])  # moved to velest file
    # ~ print("result \n%s" %(result))

    t = result[0:3]
    v = result[3:6]
    sub_result = result[6:]
    depths_u = sub_result[0::3]
    depths_v = sub_result[1::3]
    depths_prime = sub_result[2::3]
    # ~ print("depths_prime \n%s" %(depths_prime))

    return t, v, Q, None, (depths_u, depths_v, depths_prime)


def find_transform(m, m_dot, W, n, R, UseSVD):
	if UseSVD:
		return(nullspace_SVD(construct_large_C_matrix(R, n, m, m_dot, W)))
	else:
		return(nullspace_QP(construct_large_C_matrix(R, n, m, m_dot, W)))


def main():
    R = 2*np.identity(3)
    W = 3*np.identity(3)
    m = np.array([[1,2,3],[4,5,6],[7,8,9],[10,11,12],[13,14,15]])
    n = m
    m_dot = np.ones((5,3))
    m = m.T
    n = n.T
    m_dot = m_dot.T

    Q = construct_large_C_matrix(R, n, m, m_dot, W)

    print(str(Q))
    result = nullspace_QP(Q)
    
    t, v, Q2, V, depths = find_transform(m, m_dot, W, n, R, False)
    #assert(Q.all() == Q2.all())
    assert((Q == Q2).all())
    print(t)
    print(v)

if __name__ == "__main__":
    main()



