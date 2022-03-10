import numpy as np

def Find_C(m, m_dot):
    
    # select 3 points out of 5 points, all combinations 
    index_2 = np.array([[1, 2, 3], [1, 2, 4], [1, 2, 5], [1, 3, 4], [1, 3, 5], [1, 4, 5], [2, 3, 4], [2, 3, 5], [2, 4, 5], [3, 4, 5]]) 
    index_1 = np.array([[1, 2, 3], [1, 2, 4], [1, 2, 5], [1, 2, 6], [1, 3, 4], [1, 3, 5], [1, 3, 6], [1, 4, 5], [1, 4, 6], [1, 5, 6], [2, 3, 4], [2, 3, 5], [2, 3, 6], [2, 4, 5], [2, 4, 6], [2, 5, 6], [3, 4, 5], [3, 4, 6], [3, 5, 6], [4, 5, 6]]) 

    index_1 = index_1 - 1 #matlab to python index       
    index_2 = index_2 - 1

    # using 5 points version ----------------------------------------------
    # first point in all ten cases 
    mx1 = np.array([m[0, index_1[i,0]] for i in range(20)]).reshape(20,)
    my1 = np.array([m[1, index_1[i,0]] for i in range(20)]).reshape(20,)
    nx1 = np.array([m_dot[0, index_1[i,0]] for i in range(20)]).reshape(20,)
    ny1 = np.array([m_dot[1, index_1[i,0]] for i in range(20)]).reshape(20,)

    # second point in all ten cases 
    mx2 = np.array([m[0, index_1[i,1]] for i in range(20)]).reshape(20,)
    my2 = np.array([m[1, index_1[i,1]] for i in range(20)]).reshape(20,)
    nx2 = np.array([m_dot[0, index_1[i,1]] for i in range(20)]).reshape(20,)
    ny2 = np.array([m_dot[1, index_1[i,1]] for i in range(20)]).reshape(20,)
    
    # third point in all ten cases 
    mx3 = np.array([m[0, index_1[i,2]] for i in range(20)]).reshape(20,)
    my3 = np.array([m[1, index_1[i,2]] for i in range(20)]).reshape(20,)
    nx3 = np.array([m_dot[0, index_1[i,2]] for i in range(20)]).reshape(20,)
    ny3 = np.array([m_dot[1, index_1[i,2]] for i in range(20)]).reshape(20,)
    

    # formulate a,b,c,...,t coefficients, 20 terms in total 
    # These are the coefficients from solving det(M) = 0
    # where M is a 6x6 matrix (from page 2 of the scanned notes)
    a = mx1*mx2*my1 - mx1*mx2*my2 - mx1*mx3*my1 + mx1*mx3*my3 + mx2*mx3*my2 - mx2*mx3*my3 + mx1*mx2*my1*np.power(my3, 2) - mx1*mx3*my1*np.power(my2, 2) - mx1*mx2*my2*np.power(my3, 2) + mx2*mx3*np.power(my1, 2)*my2 + mx1*mx3*np.power(my2, 2)*my3 - mx2*mx3*np.power(my1, 2)*my3 

    b = -mx1*my1*my2+mx1*my1*my3+my1*mx2*my2-my1*mx3*my3-mx2*my2*my3+my2*mx3*my3 -mx1*my1*my2*np.power(mx3, 2)+mx1*my1*np.power(mx2, 2)*my3+my1*mx2*my2*np.power(mx3, 2) - np.power(mx1, 2)*mx2*my2*my3-my1*np.power(mx2, 2)*mx3*my3+np.power(mx1, 2)*my2*mx3*my3

    c = mx1*my2*np.power(mx3, 2)-mx1*np.power(mx2, 2)*my3-my1*mx2*np.power(mx3, 2)+my1*np.power(mx2, 2)*mx3+np.power(mx1, 2)*mx2*my3-np.power(mx1, 2)*my2*mx3 + mx1*my2*np.power(my3, 2)-mx1*np.power(my2, 2)*my3-my1*mx2*np.power(my3, 2)+my1*np.power(my2, 2)*mx3+np.power(my1, 2)*mx2*my3-np.power(my1, 2)*my2*mx3
    
    d = mx1*np.power(mx2, 2)-np.power(mx1, 2)*mx2-mx1*np.power(my2, 2)-mx1*np.power(mx3, 2)+np.power(mx1, 2)*mx3+np.power(my1, 2)*mx2 + mx1*np.power(my3, 2)+mx2*np.power(mx3, 2)-np.power(my1, 2)*mx3-np.power(mx2, 2)*mx3-mx2*np.power(my3, 2)+np.power(my2, 2)*mx3 + mx1*np.power(mx2, 2)*np.power(my3, 2)-mx1*np.power(my2, 2)*np.power(mx3, 2)-np.power(mx1, 2)*mx2*np.power(my3, 2)+np.power(mx1, 2)*np.power(my2, 2)*mx3+np.power(my1, 2)*mx2*np.power(mx3, 2)-np.power(my1, 2)*np.power(mx2, 2)*mx3 + mx1*my1*my2-mx1*my1*my3-my1*mx2*my2+my1*mx3*my3+mx2*my2*my3-my2*mx3*my3 + mx1*my1*my2*np.power(my3, 2)-mx1*my1*np.power(my2, 2)*my3-my1*mx2*my2*np.power(my3, 2)+np.power(my1, 2)*mx2*my2*my3+my1*np.power(my2, 2)*mx3*my3-np.power(my1, 2)*my2*mx3*my3

    e = -mx1*my2+my1*mx2+mx1*my3-my1*mx3-mx2*my3+my2*mx3 - mx1*my1*np.power(mx2, 2)-mx1*my1*np.power(my2, 2)+mx1*my1*np.power(mx3, 2)+np.power(mx1, 2)*mx2*my2+mx1*my1*np.power(my3, 2)+np.power(my1, 2)*mx2*my2 - mx1*my2*np.power(my3, 2)+mx1*np.power(my2, 2)*my3+my1*mx2*np.power(my3, 2)-my1*np.power(my2, 2)*mx3-mx2*my2*np.power(mx3, 2)-np.power(mx1, 2)*mx3*my3 - np.power(my1, 2)*mx2*my3+np.power(my1, 2)*my2*mx3-mx2*my2*np.power(my3, 2)-np.power(my1, 2)*mx3*my3+np.power(mx2, 2)*mx3*my3+np.power(my2, 2)*mx3*my3 - mx1*my1*np.power(mx2, 2)*np.power(my3, 2)+mx1*my1*np.power(my2, 2)*np.power(mx3, 2)+np.power(mx1, 2)*mx2*my2*np.power(my3, 2)-np.power(my1, 2)*mx2*my2*np.power(mx3, 2)-np.power(mx1, 2)*np.power(my2, 2)*mx3*my3+np.power(my1, 2)*np.power(mx2, 2)*mx3*my3

    f = my1*np.power(mx2, 2)- np.power(mx1, 2)*my2- my1*np.power(my2, 2)- my1*np.power(mx3, 2)+ np.power(mx1, 2)*my3+ np.power(my1, 2)*my2 + my1*np.power(my3, 2)+ my2*np.power(mx3, 2)- np.power(my1, 2)*my3- np.power(mx2, 2)*my3- my2*np.power(my3, 2)+ np.power(my2, 2)*my3 + my1*np.power(mx2, 2)*np.power(my3, 2)- my1*np.power(my2, 2)*np.power(mx3, 2)- np.power(mx1, 2)*my2*np.power(my3, 2)+ np.power(mx1, 2)*np.power(my2, 2)*my3+ np.power(my1, 2)*my2*np.power(mx3, 2)- np.power(my1, 2)*np.power(mx2, 2)*my3 - mx1*my1*mx2+ mx1*my1*mx3+ mx1*mx2*my2- mx1*mx3*my3- mx2*my2*mx3+ mx2*mx3*my3 - mx1*my1*mx2*np.power(mx3, 2)+ mx1*my1*np.power(mx2, 2)*mx3+ mx1*mx2*my2*np.power(mx3, 2)- np.power(mx1, 2)*mx2*my2*mx3- mx1*np.power(mx2, 2)*mx3*my3+ np.power(mx1, 2)*mx2*mx3*my3

    g = - mx1*my2+ my1*mx2+ mx1*my3- my1*mx3- mx2*my3+ my2*mx3 + mx1*my1*np.power(mx2, 2)+ mx1*my1*np.power(my2, 2)- mx1*my1*np.power(mx3, 2)- np.power(mx1, 2)*mx2*my2- mx1*my1*np.power(my3, 2)- mx1*my2*np.power(mx3, 2) + mx1*np.power(mx2, 2)*my3+ my1*mx2*np.power(mx3, 2)- my1*np.power(mx2, 2)*mx3- np.power(mx1, 2)*mx2*my3+ np.power(mx1, 2)*my2*mx3- np.power(my1, 2)*mx2*my2 + mx2*my2*np.power(mx3, 2)+ np.power(mx1, 2)*mx3*my3+ mx2*my2*np.power(my3, 2)+ np.power(my1, 2)*mx3*my3- np.power(mx2, 2)*mx3*my3- np.power(my2, 2)*mx3*my3 - mx1*my1*np.power(mx2, 2)*np.power(my3, 2)+ mx1*my1*np.power(my2, 2)*np.power(mx3, 2)+ np.power(mx1, 2)*mx2*my2*np.power(my3, 2)- np.power(my1, 2)*mx2*my2*np.power(mx3, 2)- np.power(mx1, 2)*np.power(my2, 2)*mx3*my3+ np.power(my1, 2)*np.power(mx2, 2)*mx3*my3 

    h = - my1*np.power(mx2, 2)+ np.power(mx1, 2)*my2- my1*np.power(my2, 2)+ my1*np.power(mx3, 2)- np.power(mx1, 2)*my3+ np.power(my1, 2)*my2 + my1*np.power(my3, 2)- my2*np.power(mx3, 2)- np.power(my1, 2)*my3+ np.power(mx2, 2)*my3- my2*np.power(my3, 2)+ np.power(my2, 2)*my3 - my1*np.power(mx2, 2)*np.power(my3, 2)+ my1*np.power(my2, 2)*np.power(mx3, 2)+ np.power(mx1, 2)*my2*np.power(my3, 2)- np.power(mx1, 2)*np.power(my2, 2)*my3- np.power(my1, 2)*my2*np.power(mx3, 2)+ np.power(my1, 2)*np.power(mx2, 2)*my3 - mx1*my1*mx2*np.power(mx3, 2)+ mx1*my1*np.power(mx2, 2)*mx3- mx1*my1*mx2*np.power(my3, 2)+ mx1*my1*np.power(my2, 2)*mx3+ mx1*mx2*my2*np.power(mx3, 2)- np.power(mx1, 2)*mx2*my2*mx3 + mx1*mx2*my2*np.power(my3, 2)- mx1*np.power(mx2, 2)*mx3*my3+ np.power(mx1, 2)*mx2*mx3*my3- np.power(my1, 2)*mx2*my2*mx3- mx1*np.power(my2, 2)*mx3*my3+ np.power(my1, 2)*mx2*mx3*my3 

    i = mx1*np.power(mx2, 2)- np.power(mx1, 2)*mx2+ mx1*np.power(my2, 2)- mx1*np.power(mx3, 2)+ np.power(mx1, 2)*mx3- np.power(my1, 2)*mx2 - mx1*np.power(my3, 2)+ mx2*np.power(mx3, 2)+ np.power(my1, 2)*mx3- np.power(mx2, 2)*mx3+ mx2*np.power(my3, 2)- np.power(my2, 2)*mx3 - mx1*np.power(mx2, 2)*np.power(my3, 2)+ mx1*np.power(my2, 2)*np.power(mx3, 2)+ np.power(mx1, 2)*mx2*np.power(my3, 2)- np.power(mx1, 2)*np.power(my2, 2)*mx3- np.power(my1, 2)*mx2*np.power(mx3, 2)+ np.power(my1, 2)*np.power(mx2, 2)*mx3 + mx1*my1*my2*np.power(mx3, 2)- mx1*my1*np.power(mx2, 2)*my3+ mx1*my1*my2*np.power(my3, 2)- mx1*my1*np.power(my2, 2)*my3- my1*mx2*my2*np.power(mx3, 2)+ np.power(mx1, 2)*mx2*my2*my3 - my1*mx2*my2*np.power(my3, 2)+ my1*np.power(mx2, 2)*mx3*my3- np.power(mx1, 2)*my2*mx3*my3+ np.power(my1, 2)*mx2*my2*my3+ my1*np.power(my2, 2)*mx3*my3- np.power(my1, 2)*my2*mx3*my3 

    j = 2*np.power(mx1, 2)*np.power(my2, 2)- 2*np.power(my1, 2)*np.power(mx2, 2)- 2*np.power(mx1, 2)*np.power(my3, 2)+ 2*np.power(my1, 2)*np.power(mx3, 2)+ 2*np.power(mx2, 2)*np.power(my3, 2)- 2*np.power(my2, 2)*np.power(mx3, 2) + 2*mx1*my1*mx2*my3- 2*mx1*my1*my2*mx3- 2*mx1*mx2*my2*my3+ 2*my1*mx2*my2*mx3+ 2*mx1*my2*mx3*my3- 2*my1*mx2*mx3*my3

    k = nx1*mx2- nx2*mx1- nx1*mx3+ nx3*mx1+ nx2*mx3- nx3*mx2 + nx1*mx2*np.power(my3, 2)- nx1*np.power(my2, 2)*mx3- nx2*mx1*np.power(my3, 2)+ nx2*np.power(my1, 2)*mx3+ nx3*mx1*np.power(my2, 2)- nx3*np.power(my1, 2)*mx2 + nx1*my1*mx2*my2- ny1*mx1*mx2*my2- nx2*mx1*my1*my2+ ny2*mx1*my1*mx2- ny2*mx1*my1*mx3+ ny3*mx1*my1*mx2 - nx1*my1*mx3*my3+ ny1*mx1*mx3*my3+ ny1*mx2*my2*mx3+ nx3*mx1*my1*my3- ny3*mx1*my1*mx3- ny3*mx1*mx2*my2 - ny1*mx2*mx3*my3+ ny2*mx1*mx3*my3+ nx2*my2*mx3*my3- ny2*mx2*mx3*my3- nx3*mx2*my2*my3+ ny3*mx2*my2*mx3 + nx1*my1*mx2*my2*np.power(my3, 2)- ny1*mx1*mx2*my2*np.power(my3, 2)- nx2*mx1*my1*my2*np.power(my3, 2)+ ny2*mx1*my1*mx2*np.power(my3, 2)- nx1*my1*np.power(my2, 2)*mx3*my3+ ny1*mx1*np.power(my2, 2)*mx3*my3 + nx3*mx1*my1*np.power(my2, 2)*my3- ny3*mx1*my1*np.power(my2, 2)*mx3+ nx2*np.power(my1, 2)*my2*mx3*my3- ny2*np.power(my1, 2)*mx2*mx3*my3- nx3*np.power(my1, 2)*mx2*my2*my3+ ny3*np.power(my1, 2)*mx2*my2*mx3 

    l = ny1*my2- ny2*my1- ny1*my3+ ny3*my1+ ny2*my3- ny3*my2 + ny1*my2*np.power(mx3, 2)- ny1*np.power(mx2, 2)*my3- ny2*my1*np.power(mx3, 2)+ ny2*np.power(mx1, 2)*my3+ ny3*my1*np.power(mx2, 2)- ny3*np.power(mx1, 2)*my2 - nx1*my1*mx2*my2+ ny1*mx1*mx2*my2+ nx2*mx1*my1*my2- ny2*mx1*my1*mx2- nx2*mx1*my1*my3+ nx3*mx1*my1*my2 + nx1*my1*mx3*my3+ nx1*mx2*my2*my3- ny1*mx1*mx3*my3- nx3*mx1*my1*my3- nx3*my1*mx2*my2+ ny3*mx1*my1*mx3 - nx1*my2*mx3*my3+ nx2*my1*mx3*my3- nx2*my2*mx3*my3+ ny2*mx2*mx3*my3+ nx3*mx2*my2*my3- ny3*mx2*my2*mx3 - nx1*my1*mx2*my2*np.power(mx3, 2)+ ny1*mx1*mx2*my2*np.power(mx3, 2)+ nx2*mx1*my1*my2*np.power(mx3, 2)- ny2*mx1*my1*mx2*np.power(mx3, 2)+ nx1*my1*np.power(mx2, 2)*mx3*my3- ny1*mx1*np.power(mx2, 2)*mx3*my3 - nx3*mx1*my1*np.power(mx2, 2)*my3+ ny3*mx1*my1*np.power(mx2, 2)*mx3- nx2*np.power(mx1, 2)*my2*mx3*my3+ ny2*np.power(mx1, 2)*mx2*mx3*my3+ nx3*np.power(mx1, 2)*mx2*my2*my3- ny3*np.power(mx1, 2)*mx2*my2*mx3

    m = - nx1*mx2*np.power(mx3, 2)+ nx1*np.power(mx2, 2)*mx3+ nx2*mx1*np.power(mx3, 2)- nx2*np.power(mx1, 2)*mx3- nx3*mx1*np.power(mx2, 2)+ nx3*np.power(mx1, 2)*mx2 - nx1*mx2*np.power(my3, 2)+ nx1*np.power(my2, 2)*mx3+ nx2*mx1*np.power(my3, 2)- nx2*np.power(my1, 2)*mx3- nx3*mx1*np.power(my2, 2)+ nx3*np.power(my1, 2)*mx2 - ny1*my2*np.power(mx3, 2)+ ny1*np.power(mx2, 2)*my3+ ny2*my1*np.power(mx3, 2)- ny2*np.power(mx1, 2)*my3- ny3*my1*np.power(mx2, 2)+ ny3*np.power(mx1, 2)*my2 - ny1*my2*np.power(my3, 2)+ ny1*np.power(my2, 2)*my3+ ny2*my1*np.power(my3, 2)- ny2*np.power(my1, 2)*my3- ny3*my1*np.power(my2, 2)+ ny3*np.power(my1, 2)*my2 + nx1*my1*mx2*my3- nx1*my1*my2*mx3- ny1*mx1*mx2*my3+ ny1*mx1*my2*mx3- nx2*mx1*my2*my3+ nx2*my1*my2*mx3 + ny2*mx1*mx2*my3- ny2*my1*mx2*mx3+ nx3*mx1*my2*my3- nx3*my1*mx2*my3- ny3*mx1*my2*mx3+ ny3*my1*mx2*mx3 

    n = nx1*my2+ ny1*mx2- nx2*my1- ny2*mx1- nx1*my3- ny1*mx3 + nx3*my1+ ny3*mx1+ nx2*my3+ ny2*mx3- nx3*my2- ny3*mx2 - nx1*my1*np.power(mx2, 2)+ ny1*mx1*np.power(mx2, 2)+ nx1*my1*np.power(my2, 2)+ nx1*my1*np.power(mx3, 2)- ny1*mx1*np.power(my2, 2)- ny1*mx1*np.power(mx3, 2) + nx2*np.power(mx1, 2)*my2- ny2*np.power(mx1, 2)*mx2- nx1*my1*np.power(my3, 2)+ ny1*mx1*np.power(my3, 2)+ ny1*mx2*np.power(mx3, 2)- ny1*np.power(mx2, 2)*mx3 - nx2*np.power(my1, 2)*my2- ny2*mx1*np.power(mx3, 2)+ ny2*np.power(mx1, 2)*mx3+ ny2*np.power(my1, 2)*mx2+ ny3*mx1*np.power(mx2, 2)- ny3*np.power(mx1, 2)*mx2 + nx1*my2*np.power(my3, 2)- nx1*np.power(my2, 2)*my3- nx2*my1*np.power(my3, 2)- nx2*my2*np.power(mx3, 2)+ nx2*np.power(my1, 2)*my3+ ny2*mx2*np.power(mx3, 2) + nx3*my1*np.power(my2, 2)- nx3*np.power(mx1, 2)*my3- nx3*np.power(my1, 2)*my2+ ny3*np.power(mx1, 2)*mx3+ nx2*my2*np.power(my3, 2)- ny2*mx2*np.power(my3, 2) + nx3*np.power(my1, 2)*my3+ nx3*np.power(mx2, 2)*my3- ny3*np.power(my1, 2)*mx3- ny3*np.power(mx2, 2)*mx3- nx3*np.power(my2, 2)*my3+ ny3*np.power(my2, 2)*mx3 - nx1*my1*np.power(mx2, 2)*np.power(my3, 2)+ nx1*my1*np.power(my2, 2)*np.power(mx3, 2)+ ny1*mx1*np.power(mx2, 2)*np.power(my3, 2)- ny1*mx1*np.power(my2, 2)*np.power(mx3, 2)+ nx2*np.power(mx1, 2)*my2*np.power(my3, 2)- nx2*np.power(my1, 2)*my2*np.power(mx3, 2) - ny2*np.power(mx1, 2)*mx2*np.power(my3, 2)+ ny2*np.power(my1, 2)*mx2*np.power(mx3, 2)- nx3*np.power(mx1, 2)*np.power(my2, 2)*my3+ nx3*np.power(my1, 2)*np.power(mx2, 2)*my3+ ny3*np.power(mx1, 2)*np.power(my2, 2)*mx3- ny3*np.power(my1, 2)*np.power(mx2, 2)*mx3 - nx2*mx1*my1*mx3+ nx3*mx1*my1*mx2+ nx1*mx2*my2*mx3- nx3*mx1*mx2*my2- nx1*mx2*mx3*my3+ nx2*mx1*mx3*my3 - ny2*mx1*my1*my3+ ny3*mx1*my1*my2+ ny1*mx2*my2*my3- ny3*my1*mx2*my2- ny1*my2*mx3*my3+ ny2*my1*mx3*my3   
        
    o = - nx1*np.power(mx2, 2)+ nx2*np.power(mx1, 2)- nx1*np.power(my2, 2)+ nx1*np.power(mx3, 2)+ nx2*np.power(my1, 2)- nx3*np.power(mx1, 2) + nx1*np.power(my3, 2)- nx2*np.power(mx3, 2)- nx3*np.power(my1, 2)+ nx3*np.power(mx2, 2)- nx2*np.power(my3, 2)+ nx3*np.power(my2, 2) - nx1*np.power(mx2, 2)*np.power(my3, 2)+ nx1*np.power(my2, 2)*np.power(mx3, 2)+ nx2*np.power(mx1, 2)*np.power(my3, 2)- nx2*np.power(my1, 2)*np.power(mx3, 2)- nx3*np.power(mx1, 2)*np.power(my2, 2)+ nx3*np.power(my1, 2)*np.power(mx2, 2) + nx1*my1*my2- ny1*mx1*my2- nx1*my1*my3+ ny1*mx1*my3- nx2*my1*my2+ ny2*my1*mx2 - ny1*mx2*my3+ ny1*my2*mx3+ ny2*mx1*my3- ny2*my1*mx3- ny3*mx1*my2+ ny3*my1*mx2 + nx2*my2*my3- ny2*mx2*my3+ nx3*my1*my3- ny3*my1*mx3- nx3*my2*my3+ ny3*my2*mx3 + ny2*mx1*my1*np.power(mx3, 2)- ny3*mx1*my1*np.power(mx2, 2)+ nx1*my1*my2*np.power(my3, 2)- nx1*my1*np.power(my2, 2)*my3- ny1*mx1*my2*np.power(my3, 2)+ ny1*mx1*np.power(my2, 2)*my3 - ny1*mx2*my2*np.power(mx3, 2)+ ny2*mx1*my1*np.power(my3, 2)- ny3*mx1*my1*np.power(my2, 2)+ ny3*np.power(mx1, 2)*mx2*my2- ny1*mx2*my2*np.power(my3, 2)+ ny1*np.power(mx2, 2)*mx3*my3 - nx2*my1*my2*np.power(my3, 2)+ nx2*np.power(my1, 2)*my2*my3+ ny2*my1*mx2*np.power(my3, 2)- ny2*np.power(mx1, 2)*mx3*my3- ny2*np.power(my1, 2)*mx2*my3+ ny3*np.power(my1, 2)*mx2*my2 + ny1*np.power(my2, 2)*mx3*my3- ny2*np.power(my1, 2)*mx3*my3+ nx3*my1*np.power(my2, 2)*my3- nx3*np.power(my1, 2)*my2*my3- ny3*my1*np.power(my2, 2)*mx3+ ny3*np.power(my1, 2)*my2*mx3 - nx1*my1*mx2*my2*mx3+ ny1*mx1*mx2*my2*mx3+ nx2*mx1*my1*my2*mx3- ny2*mx1*my1*mx2*mx3+ nx1*my1*mx2*mx3*my3- ny1*mx1*mx2*mx3*my3 - nx3*mx1*my1*mx2*my3+ ny3*mx1*my1*mx2*mx3- nx2*mx1*my2*mx3*my3+ ny2*mx1*mx2*mx3*my3+ nx3*mx1*mx2*my2*my3- ny3*mx1*mx2*my2*mx3 

    p = - ny1*np.power(mx2, 2)+ ny2*np.power(mx1, 2)- ny1*np.power(my2, 2)+ ny1*np.power(mx3, 2)+ ny2*np.power(my1, 2)- ny3*np.power(mx1, 2) + ny1*np.power(my3, 2)- ny2*np.power(mx3, 2)- ny3*np.power(my1, 2)+ ny3*np.power(mx2, 2)- ny2*np.power(my3, 2)+ ny3*np.power(my2, 2) + ny1*np.power(mx2, 2)*np.power(my3, 2)- ny1*np.power(my2, 2)*np.power(mx3, 2)- ny2*np.power(mx1, 2)*np.power(my3, 2)+ ny2*np.power(my1, 2)*np.power(mx3, 2)+ ny3*np.power(mx1, 2)*np.power(my2, 2)- ny3*np.power(my1, 2)*np.power(mx2, 2) - nx1*my1*mx2+ ny1*mx1*mx2+ nx1*my1*mx3- ny1*mx1*mx3+ nx2*mx1*my2- ny2*mx1*mx2 + nx1*mx2*my3- nx1*my2*mx3- nx2*mx1*my3+ nx2*my1*mx3+ nx3*mx1*my2- nx3*my1*mx2 - nx2*my2*mx3+ ny2*mx2*mx3- nx3*mx1*my3+ ny3*mx1*mx3+ nx3*mx2*my3- ny3*mx2*mx3 - nx1*my1*mx2*np.power(mx3, 2)+ nx1*my1*np.power(mx2, 2)*mx3+ ny1*mx1*mx2*np.power(mx3, 2)- ny1*mx1*np.power(mx2, 2)*mx3+ nx2*mx1*my1*np.power(mx3, 2)- nx3*mx1*my1*np.power(mx2, 2) - nx1*mx2*my2*np.power(mx3, 2)+ nx2*mx1*my1*np.power(my3, 2)+ nx2*mx1*my2*np.power(mx3, 2)- nx2*np.power(mx1, 2)*my2*mx3- ny2*mx1*mx2*np.power(mx3, 2)+ ny2*np.power(mx1, 2)*mx2*mx3 - nx3*mx1*my1*np.power(my2, 2)+ nx3*np.power(mx1, 2)*mx2*my2- nx1*mx2*my2*np.power(my3, 2)+ nx1*np.power(mx2, 2)*mx3*my3- nx2*np.power(mx1, 2)*mx3*my3- nx3*mx1*np.power(mx2, 2)*my3 + nx3*np.power(mx1, 2)*mx2*my3+ nx3*np.power(my1, 2)*mx2*my2+ ny3*mx1*np.power(mx2, 2)*mx3- ny3*np.power(mx1, 2)*mx2*mx3+ nx1*np.power(my2, 2)*mx3*my3- nx2*np.power(my1, 2)*mx3*my3 + nx1*my1*mx2*my2*my3- ny1*mx1*mx2*my2*my3- nx2*mx1*my1*my2*my3+ ny2*mx1*my1*mx2*my3- nx1*my1*my2*mx3*my3+ ny1*mx1*my2*mx3*my3 + nx3*mx1*my1*my2*my3- ny3*mx1*my1*my2*mx3+ nx2*my1*my2*mx3*my3- ny2*my1*mx2*mx3*my3- nx3*my1*mx2*my2*my3+ ny3*my1*mx2*my2*mx3 

    q = nx1*nx2*my1- ny1*nx2*mx1- nx1*nx2*my2+ nx1*ny2*mx2- nx1*nx3*my1+ ny1*nx3*mx1 - nx1*ny2*mx3+ nx1*ny3*mx2+ ny1*nx2*mx3- ny1*nx3*mx2- nx2*ny3*mx1+ ny2*nx3*mx1 + nx1*nx3*my3- nx1*ny3*mx3+ nx2*nx3*my2- ny2*nx3*mx2- nx2*nx3*my3+ nx2*ny3*mx3 + nx1*nx2*my1*np.power(my3, 2)- nx1*nx3*my1*np.power(my2, 2)- ny1*nx2*mx1*np.power(my3, 2)+ ny1*nx3*mx1*np.power(my2, 2)- nx1*nx2*my2*np.power(my3, 2)+ nx1*ny2*mx2*np.power(my3, 2) + nx2*nx3*np.power(my1, 2)*my2- ny2*nx3*np.power(my1, 2)*mx2+ nx1*nx3*np.power(my2, 2)*my3- nx1*ny3*np.power(my2, 2)*mx3- nx2*nx3*np.power(my1, 2)*my3+ nx2*ny3*np.power(my1, 2)*mx3 + nx1*ny3*my1*mx2*my2- ny1*ny3*mx1*mx2*my2- nx2*ny3*mx1*my1*my2+ ny2*ny3*mx1*my1*mx2- nx1*ny2*my1*mx3*my3+ ny1*ny2*mx1*mx3*my3 + ny2*nx3*mx1*my1*my3- ny2*ny3*mx1*my1*mx3+ ny1*nx2*my2*mx3*my3- ny1*ny2*mx2*mx3*my3- ny1*nx3*mx2*my2*my3+ ny1*ny3*mx2*my2*mx3 

    r = nx1*ny2*my1- ny1*ny2*mx1- nx1*ny3*my1- ny1*nx2*my2+ ny1*ny2*mx2+ ny1*ny3*mx1 - nx1*ny2*my3+ nx1*ny3*my2+ ny1*nx2*my3- ny1*nx3*my2- nx2*ny3*my1+ ny2*nx3*my1 + ny1*nx3*my3- ny1*ny3*mx3+ nx2*ny3*my2- ny2*ny3*mx2- ny2*nx3*my3+ ny2*ny3*mx3 + nx1*ny2*my1*np.power(mx3, 2)- nx1*ny3*my1*np.power(mx2, 2)- ny1*ny2*mx1*np.power(mx3, 2)+ ny1*ny3*mx1*np.power(mx2, 2)- ny1*nx2*my2*np.power(mx3, 2)+ ny1*ny2*mx2*np.power(mx3, 2) + nx2*ny3*np.power(mx1, 2)*my2- ny2*ny3*np.power(mx1, 2)*mx2+ ny1*nx3*np.power(mx2, 2)*my3- ny1*ny3*np.power(mx2, 2)*mx3- ny2*nx3*np.power(mx1, 2)*my3+ ny2*ny3*np.power(mx1, 2)*mx3 + nx1*nx3*my1*mx2*my2- ny1*nx3*mx1*mx2*my2- nx2*nx3*mx1*my1*my2+ ny2*nx3*mx1*my1*mx2- nx1*nx2*my1*mx3*my3+ ny1*nx2*mx1*mx3*my3 + nx2*nx3*mx1*my1*my3- nx2*ny3*mx1*my1*mx3+ nx1*nx2*my2*mx3*my3- nx1*ny2*mx2*mx3*my3- nx1*nx3*mx2*my2*my3+ nx1*ny3*mx2*my2*mx3

    s = nx1*ny2*np.power(mx3, 2)- nx1*ny3*np.power(mx2, 2)- ny1*nx2*np.power(mx3, 2)+ ny1*nx3*np.power(mx2, 2)+ nx2*ny3*np.power(mx1, 2)- ny2*nx3*np.power(mx1, 2) + nx1*ny2*np.power(my3, 2)- nx1*ny3*np.power(my2, 2)- ny1*nx2*np.power(my3, 2)+ ny1*nx3*np.power(my2, 2)+ nx2*ny3*np.power(my1, 2)- ny2*nx3*np.power(my1, 2) - nx1*nx2*my1*mx3+ nx1*nx3*my1*mx2+ ny1*nx2*mx1*mx3- ny1*nx3*mx1*mx2+ nx1*nx2*my2*mx3- nx1*ny2*mx2*mx3 - nx2*nx3*mx1*my2+ ny2*nx3*mx1*mx2- nx1*ny2*my1*my3- nx1*nx3*mx2*my3+ nx1*ny3*my1*my2+ nx1*ny3*mx2*mx3 + ny1*ny2*mx1*my3- ny1*ny3*mx1*my2+ nx2*nx3*mx1*my3- nx2*ny3*mx1*mx3+ ny1*nx2*my2*my3- ny1*ny2*mx2*my3 - nx2*ny3*my1*my2+ ny2*ny3*my1*mx2- ny1*nx3*my2*my3+ ny1*ny3*my2*mx3+ ny2*nx3*my1*my3- ny2*ny3*my1*mx3 

    t = nx1*nx2*ny3*my1- nx1*ny2*nx3*my1- ny1*nx2*ny3*mx1+ ny1*ny2*nx3*mx1- nx1*nx2*ny3*my2+ nx1*ny2*ny3*mx2 + ny1*nx2*nx3*my2- ny1*ny2*nx3*mx2+ nx1*ny2*nx3*my3- nx1*ny2*ny3*mx3- ny1*nx2*nx3*my3+ ny1*nx2*ny3*mx3 

    # A matrix, size: 30x34 
    # zeroes are used as a placeholder for the combinations that don't have
    # a coefficient (see double page)
    
    # each coefficient (e.g. 'a') is a 10x1 vector that represents all the
    # values of the coefficient for every combination of 3 points from the
    # original 5.


    C = np.stack((a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t), axis=1)
    return C


def main():
    m = []
    for i in range(3):
        k = []
        for j in range(6):
            k.append(6*i + j)
        m.append(k)
    m = np.array(m)
    n = m +3
    C = Find_C(m, n)
    print(C)
    print(C.shape)

if __name__ == "__main__":
    main()
