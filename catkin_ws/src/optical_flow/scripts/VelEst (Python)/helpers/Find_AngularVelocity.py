import numpy as np

# numpy's eig, like matlab's eig, does not return the eigen values and eigen vectors in any particular order, 
# so this function appears to return the same eigen vectors just with the terms in different order to
# reflect the different order of the eigen values

def Find_AngularVelocity(B, number):
    #   the 4th, 3rd, and 2nd to last elements in the np.linalg.eigen vector are wx, wy, and wz respectively (see page 8 of scanned notes) 
    switcher = { 1: one, 2: two, 3: three, 4: four, 5: five, 6: six }
    func = switcher.get(number, lambda: "Invalid Option")
    return func(B)


def one(B): 
    e_value, e_vector = np.linalg.eig(B) # find eigenvectors 
    e_vector_real = np.real(e_vector) # np.real parts
    e_vector_real_norm = np.divide(e_vector_real, e_vector_real[-1, :]) # normalize each eigenvector 

    omega_x = e_vector_real_norm[-4,:] 
    omega_y = e_vector_real_norm[-3,:] 
    omega_z = e_vector_real_norm[-2,:] 

    omega = np.stack((omega_x, omega_y, omega_z), axis = 0)
    
    lamda = np.diag(e_value) 
    return (omega, lamda)

def two(B): 
    e_value, e_vector = np.linalg.eig(B) 
    e_vector_real = np.real(e_vector) 
    e_vector_real_norm = np.divide(e_vector_real, e_vector_real[-1, :]) # normalize each eigenvector 

    omega_x = e_vector_real_norm[-4,:] 
    omega_y = e_vector_real_norm[-3,:] 
    omega_z = e_vector_real_norm[-2,:] 

    omega = np.stack((omega_x, omega_y, omega_z), axis = 0)

    lamda = np.diag(e_value)   
    return (omega, lamda)

def three(B): 
    e_value, e_vector = np.linalg.eig(B) 
    e_vector_real = np.real(e_vector) 
    e_vector_real_norm = np.divide(e_vector_real, e_vector_real[-1, :]) # normalize each eigenvector 

    omega_x = e_vector_real_norm[-3,:] 
    omega_y = e_vector_real_norm[-4,:] 
    omega_z = e_vector_real_norm[-2,:] 

    omega = np.stack((omega_x, omega_y, omega_z), axis = 0)

    lamda = np.diag(e_value)  
    return (omega, lamda)
    
def four(B): 
    e_value, e_vector = np.linalg.eig(B) 
    e_vector_real = np.real(e_vector) 
    e_vector_real_norm = np.divide(e_vector_real, e_vector_real[-1, :]) # normalize each eigenvector 

    omega_x = e_vector_real_norm[-3,:] 
    omega_y = e_vector_real_norm[-4,:] 
    omega_z = e_vector_real_norm[-2,:] 

    omega = np.stack((omega_x, omega_y, omega_z), axis = 0)

    lamda = np.diag(e_value) 
    return (omega, lamda)
    
def five(B): 
    e_value, e_vector = np.linalg.eig(B) 
    e_vector_real = np.real(e_vector) 
    e_vector_real_norm = np.divide(e_vector_real, e_vector_real[-1, :]) # normalize each eigenvector 
    omega_x = e_vector_real_norm[-3,:] 
    omega_y = e_vector_real_norm[-2,:] 
    omega_z = e_vector_real_norm[-4,:] 

    omega = np.stack((omega_x, omega_y, omega_z), axis = 0)

    lamda = np.diag(e_value) 
    return (omega, lamda)
    
def six(B): 
    e_value, e_vector = np.linalg.eig(B) 
    e_vector_real = np.real(e_vector) 
    e_vector_real_norm = np.divide(e_vector_real, e_vector_real[-1, :]) # normalize each eigenvector 

    omega_x = e_vector_real_norm[-3,:] 
    omega_y = e_vector_real_norm[-2,:] 
    omega_z = e_vector_real_norm[-4,:] 

    omega = np.stack((omega_x, omega_y, omega_z), axis = 0)

    lamda = np.diag(e_value) 
    return (omega, lamda)



def main():
    m = []
    for i in range(20):
        k = []
        for j in range(20):
            k.append(20*i + j)
        m.append(k)
    m[1][1] = 10
    m[1][3] = 20
    m[4][17] = -2
    B = np.array(m)
    print(B.shape)
    for i in range(6):
        o, l = Find_AngularVelocity(B, i+1)
        print(o)
        #print(l)
        print("------------------------------------")

if __name__ == "__main__":
    main()
