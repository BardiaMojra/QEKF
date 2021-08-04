#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


def angular_error((w_est, w_true)):
    return np.linalg.norm(w_true - w_est)

def linear_error((v_est, v_true)):
    return (1/np.pi)*np.arccos(np.dot(v_est, v_true)/(np.linalg.norm(v_est)*np.linalg.norm(v_true)))

# Allows for custom error function provided the function wraps all its args into a tuple
def calculate_error(est, true, error_func=angular_error):
    assert(est.size == true.size)
    error_list = list(map(error_func, [(est[:,i], true[:,i]) for i in range(est.shape[1])]))
    return error_list


def calculate_covariance(good_sections, est, truth):
    
    est_list = []
    truth_list = []

    data_file = open(est, "r")
    for line in data_file:
        data = list(map(float, line.split(" ")))
        if data[0] in good_sections:
            vec = np.array(data[1:]).reshape(6,1)
            try:
                est_list = np.append(est_list, vec, axis=1)
            except IndexError:
                est_list = vec
    
    data_file.close()

    data_file = open(truth, "r")
    for line in data_file:
        data = list(map(float, line.split(" ")))
        if data[0] in good_sections:
            vec = np.array(data[1:]).reshape(6,1)
            try:
                truth_list = np.append(truth_list, vec, axis=1)
            except IndexError:
                truth_list = vec
    
    data_file.close()

    return est_list - truth_list


def load_calculated_estimates(num_images, file_name, normalize=False, d1=3, d2=3):

    est_list_even = np.zeros((d1, num_images-1))
    est_list_odd = np.zeros((d2, num_images-1))
    est_list_even.fill(np.nan)
    est_list_odd.fill(np.nan)

    data_file = open(file_name, "r")
    for idx, line in enumerate(data_file):
        data = list(map(float, line.split(" ")))
        if idx < num_images:
                est_list_even[:,int(data[0])-1] = np.array(data[1:1+d1])
                est_list_odd[:,int(data[0])-1] = np.array(data[1+d1:])
        idx += 1
    data_file.close()

    return est_list_even, est_list_odd



#file_path is the absolute directory .../oxts/data/
def load_KITTI_truth(num_images, file_path, normalize=False):

    v_true_list = np.zeros((3, num_images-1))
    w_true_list = np.zeros((3, num_images-1))
    q_true_list = np.zeros((3, num_images-1))

    #The first image is skipped since we can't measure optical flow without two images.
    for counter in range(num_images-1):
        file_idx = "%010d.txt" % (counter+1)
        data_file = open(file_path+file_idx, "r")
        data = []
        for line in data_file:
            data.append(list(map(float, line.split(" "))))
        data_file.close()
        #This is the hardcoded location of v and w in KITTI's file format
        v_true = np.array([-1*float(data[0][9]), -1*float(data[0][10]), float(data[0][8])])
        w_true = np.array([-1*float(data[0][21]), -1*float(data[0][22]), float(data[0][20])])
        q_true = np.array([float(data[0][3]), float(data[0][4]), float(data[0][5])]) 
        if normalize:
            v_true = np.divide(v_true, np.linalg.norm(v_true, 2))

        v_true_list[:,counter] = v_true
        w_true_list[:,counter] = w_true
        q_true_list[:,counter] = q_true
    
    return v_true_list, w_true_list, q_true_list




def show_results(est_list, true, data_name, est_name, x_name, y_name, size=3):
    
    counter = 1
    rows = 0
    if true is not None:
        rows += 1
    if est_list is not None:
        rows += len(est_list)
    
    # Create seperate window for each plot
    plt.figure(figsize=(20,10))

   
    if true is not None:
        plt.subplot(rows, 1, counter)
        plt.ylabel(y_name)
        plt.xlabel(x_name)
        
        axes = plt.gca()
        plt.title("True " + data_name)
        x_range = np.array(range(true.shape[1]))
        
        plt.plot(x_range, true[0,:], linestyle = '-', color = 'red')
        plt.plot(x_range, true[1,:], linestyle = '-', color = 'green')
        plt.plot(x_range, true[2,:], linestyle = '-', color = 'blue')
        if size==4:
            plt.plot(x_range, true[3,:], linestyle = '-', color = 'yellow')
            
        plt.legend(('x*', 'y*', 'z*'), loc = 'right')
        if size==4:
            plt.legend(('x*', 'y*', 'z*', 'w*'), loc = 'right')


        axes.set_xlim([0,len(x_range)])
        #axes.set_ylim([-1.5,1.5])
        counter += 1
    
    
    # Setup to take a list of all the estimates for the same value as well as a list of their names
    if est_list is not None:
        for idx, est in enumerate(est_list):
            plt.subplot(rows, 1, counter)
            plt.ylabel(y_name)
            plt.xlabel(x_name)
            
            axes = plt.gca()
            plt.title("Estimated " + est_name[idx])
            x_range = np.array(range(est.shape[1]))
            
            plt.plot(x_range, est[0,:], linestyle = '--', color = 'red')
            plt.plot(x_range, est[1,:], linestyle = '--', color = 'green')
            plt.plot(x_range, est[2,:], linestyle = '--', color = 'blue')
            if size==4:
                plt.plot(x_range, est[3,:], linestyle = '--', color = 'yellow')
            
            plt.legend(('x', 'y', 'z'), loc = 'right')
            if size==4:
                plt.legend(('x', 'y', 'z', 'w'), loc = 'right')
            
            axes.set_xlim([0,len(x_range)])
            #axes.set_ylim([-0.5,0.5])
            counter += 1
    
    fig = plt.gcf() 
    fig.tight_layout()
    plt.subplots_adjust(left=0.06, right=0.97, hspace=0.5)

def normalize_results(est_list):
    est_sums = np.linalg.norm(est_list, 2, axis=0)
    est_sums = np.concatenate((est_sums, est_sums, est_sums), axis=0).reshape(est_list.shape)
    est_norm = np.divide(est_list, est_sums, dtype=np.float64)
    assert(est_norm.shape == est_list.shape)
    return est_norm


def correct_negatives(est_list):
    est_corrected = []
    est = est_list.T
    for v in est:
        if v[2] < 0:
            v = -1*v
        v = v.reshape(3,1)

        try:
            est_corrected = np.append(est_corrected, v, axis=1)
        except IndexError:
            est_corrected = np.array(v, ndmin = 2).reshape(3,1)

    for v in est_corrected.T:
        assert(np.isnan(v[2]) or v[2] >= 0)

    return est_corrected

def vel_to_trans(velocity):
    dt = 0.1
    transform_list = []
    
    for idx, v in enumerate(velocity.T):
        try:
            vec = np.array([0,0,0])
            if len(transform_list) > 0:
                vec = transform_list[:,idx-1].reshape(3,1) 
            vec = vec + dt*(v.reshape(3,1))
            transform_list = np.append(transform_list, vec, axis=1)
        except IndexError:
            transform_list = (dt*v).reshape(3,1)
    return transform_list

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def rpy_to_q(rpy):
    q_list = []
    for row in rpy.T:
        q = np.array(euler_to_quaternion(row[0], row[1], row[2])).reshape(4,1)
        
        try:
            q_list = np.append(q_list, q, axis=1)
        except IndexError:
            q_list = q.reshape(4,1)

    return q_list


def run_analysis(num_images, path_to_data, path_to_truth, isKITTI = True):
    v_vest, w_vest = load_calculated_estimates(num_images, path_to_data+"vest.txt")
    t_vest, v_vest2 = load_calculated_estimates(num_images, path_to_data+"covariance_data.txt")
    t_ekf, v_ekf = load_calculated_estimates(num_images, path_to_data+"ekf_linear.txt")
    q_ekf, w_ekf = load_calculated_estimates(num_images, path_to_data+"ekf_angular.txt", d1=4, d2=3)
    t_quest, q_quest = load_calculated_estimates(num_images, path_to_data+"quest.txt", d1=3, d2=4)
    
    v_true = None
    w_true = None
    t_true = None
    q_true = None
    
    print(v_vest == v_vest2)    
    #assert((v_vest == v_vest2).all())

    #v_vest = 10*v_vest
    #v_ekf = 10*v_ekf

    #v_vest = normalize_results(v_vest)
    #v_ekf = normalize_results(v_ekf)

    #v_vest = correct_negatives(v_vest)
    #v_ekf = correct_negatives(v_ekf)
    np.set_printoptions(linewidth=1000)

    if isKITTI:
        v_true, w_true, rpy_true = load_KITTI_truth(num_images, path_to_truth, normalize=False)
        q_true = rpy_to_q(rpy_true)

        # quest solves for relative rotation
        q_true = q_true - q_true[0]
        
        t_true = vel_to_trans(v_true)
       
        # KITTI has positive and negative direction reversed compared to Velest
        if w_true is not None:
            w_true = -1*w_true
        
        v_error = calculate_error(v_vest, v_true, error_func=linear_error)
        w_error = calculate_error(w_vest, w_true, error_func=angular_error)
    
        v_ekf_error = calculate_error(v_ekf, v_true, error_func=linear_error)
        
        w_error_mean = np.mean(w_error)
        w_error_std = np.std(w_error, ddof=1, dtype=np.float64)
        print("Plain Velest Angular Velocity:\nmean error = %f\nstd of error = %f\n\n" % (w_error_mean, w_error_std))

        v_error_mean = np.mean(v_error)
        v_error_std = np.std(v_error, ddof=1, dtype=np.float64)
        print("Plain Velest Linear Velocity:\nmean error = %f\nstd of error = %f\n\n" % (v_error_mean, v_error_std))
        
        v_ekf_error_mean = np.mean(v_ekf_error)
        v_ekf_error_std = np.std(v_ekf_error, ddof=1, dtype=np.float64)
        print("EKF filtered Velest Linear Velocity:\nmean error = %f\nstd of error = %f\n\n" % (v_ekf_error_mean, v_ekf_error_std))
   
        #r_cov_data = calculate_covariance(range(0,445), path_to_data+"covariance_data.txt", v_true, w_true)
        r_cov_data = np.concatenate((q_quest - q_true, w_vest - w_true), axis=0)
        #print(r_cov_data)
        cov_matrix = np.cov(r_cov_data)
        print(cov_matrix)


    #show_results(est_list=[v_vest], true=v_true, data_name="Linear Velocity", est_name=["Vest Linear Velocity"], x_name="Frames", y_name="Velocity")
    show_results(est_list=[w_vest, w_ekf], true=w_true, data_name="Angular Velocity", est_name=["Vest Angular Velocity", "EKF Angular Velocity"], x_name="Frames", y_name="Velocity")
    plt.savefig(path_to_data+"angular_velocity.png")
    #show_results(est_list=[v_ekf], true=v_true, data_name="Linear Velocity", est_name=["EKF Linear Velocity"], x_name="Frames", y_name="Velocity")
    
    show_results(est_list=[v_vest, v_ekf], true=v_true, data_name="Linear Velocity", est_name=["Vest Linear Velocity", "EKF Linear Velocity"], x_name="Frames", y_name="Velocity")
    plt.savefig(path_to_data+"linear_velocity.png")
    
    show_results(est_list=[t_quest, t_vest, t_ekf], true=t_true, data_name="Linear Transform", est_name=["Quest Linear Transform", "Vest QP Linear Transform", "EKF Linear Transform"], x_name="Frames", y_name="Length")
    plt.savefig(path_to_data+"linear_transform.png")
    
    show_results(est_list=[q_quest, q_ekf], true=q_true, data_name="Quaternion", est_name=["Quest Quaternion", "EKF Linear Transform"], x_name="Frames", y_name="Quaternion", size=4)
    plt.savefig(path_to_data+"quaternion.png")
    
    plt.show()



def main():
    #args are (number of images, path to where to store the results, path to the ground truth if using kitti dataset)
<<<<<<< Updated upstream
    #run_analysis(447, "/home/developer/Documents/vbme_results/", "/home/developer/Desktop/2011_09_26/2011_09_26_drive_0009_sync/oxts/data/")
    # ~ run_analysis(153, "/home/developer/Documents/vbme_results/", None, False)
    
    run_analysis(20, "/tmp/", "/home/codyl/Desktop/2011_09_26_drive_0009_sync/2011_09_26/2011_09_26_drive_0009_sync/oxts/data/")
	
=======
    # ~ run_analysis(447, "/home/developer/Documents/vbme_results/", "/home/developer/Desktop/2011_09_26/2011_09_26_drive_0009_sync/oxts/data/")
    run_analysis(20, "/tmp/", "/home/codyl/Desktop/2011_09_26_drive_0009_sync/2011_09_26/2011_09_26_drive_0009_sync/oxts/data/")
>>>>>>> Stashed changes
    


if __name__ == "__main__":
    main()
