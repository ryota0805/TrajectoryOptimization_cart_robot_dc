from param import Parameter as p
import numpy as np
import util



def objective_function(x, *args):
    #matrixに変換
    trajectory_matrix = x.reshape(p.M, p.N)
    x1, y, theta, omega, v = trajectory_matrix[0], trajectory_matrix[1], trajectory_matrix[2], trajectory_matrix[3], trajectory_matrix[4]
    
    #phiの二乗和を目的関数とする。
    sum = 0
    for i in range(p.N):
        sum += (omega[i] ** 2 / p.omega_max ** 2) + (v[i] ** 2 / p.v_max ** 2)
    
    return sum / p.N

def check_objective_function(x, *args):
    #matrixに変換
    trajectory_matrix = x.reshape(p.M, p.N)
    
    #phiの二乗和を目的関数とする。
    sum1, sum2 = 0, 0
    for i in range(p.N):
        sum1 += (trajectory_matrix[3, i] ** 2 / p.phi_max ** 2)


    for i in range(p.N):
        sum2 += (trajectory_matrix[4, i] ** 2 / p.v_max ** 2) 
    
    return sum1 / p.N, sum2 / p.N, (sum1 + sum2) / p.N
    

def jac_of_objective_function(x, *args):
    #matrixに変換
    trajectory_matrix = x.reshape(p.M, p.N)
    theta, theta1, theta2, omega1, omega2, v1, v2 = trajectory_matrix[2], trajectory_matrix[3], trajectory_matrix[4], trajectory_matrix[5], trajectory_matrix[6], trajectory_matrix[7], trajectory_matrix[8]
    
    jac_f = np.zeros((p.M, p.N))

    for i in range(p.N):
        jac_f[2, i] = 10*2*(theta[i]/p.theta_max - theta1[i]/p.theta1_max)/p.theta_max + 10*2*(theta[i]/p.theta_max - theta2[i]/p.theta2_max)/p.theta_max 
        jac_f[3, i] = -10*2*(theta[i]/p.theta_max - theta1[i]/p.theta1_max)/p.theta1_max
        jac_f[4, i] = -10*2*(theta[i]/p.theta_max - theta2[i]/p.theta1_max)/p.theta2_max
        
        #phiの微分
        jac_f[5, i] = (omega1[i] * 2) / ((p.omega1_max ** 2))  
        
        jac_f[6, i] = (omega2[i] * 2) / ((p.omega2_max ** 2)) 
    
        #vの微分
        jac_f[7, i] = (v1[i] * 2) / ((p.v1_max ** 2)) 
        
        jac_f[8, i] = (v2[i] * 2) / ((p.v2_max ** 2)) 

    #ベクトルに直す
    jac_f = jac_f.flatten()
    
    return jac_f/p.N

def sigmoid(x, a = 100):
    return 1 / (1 + np.exp(-a*x))


def grad_sigmoid(x, a = 100):
    return a*np.exp(-a*x) / (1 + np.exp(-a*x))**2


def objective_function2(x, *args):
    w1, w2 = args[0], args[1]
    trajectory_matrix = x.reshape(p.M, p.N)
    x1, y, theta, omega, v = trajectory_matrix[0], trajectory_matrix[1], trajectory_matrix[2], trajectory_matrix[3], trajectory_matrix[4]
    

    sum = 0
    for i in range(p.N):
        sum += (omega[i] ** 2 / p.phi_max ** 2) + w1*sigmoid(v[i])*(v[i] ** 2 / p.v_max ** 2) + w2*sigmoid(-v[i])*(v[i] ** 2 / p.v_max ** 2)
    
    return sum / p.N


def jac_of_objective_function2(x, *args):
    w1, w2 = args[0], args[1]
    #matrixに変換
    trajectory_matrix = x.reshape(p.M, p.N)
    
    jac_f = np.zeros((p.M, p.N))

    for i in range(p.N):
        #phiの微分
        jac_f[3, i] = (trajectory_matrix[3, i] * 2) / (p.N * (p.phi_max ** 2))  
    
        #vの微分
        jac_f[4, i] = (trajectory_matrix[4, i] * 2) / (p.N * (p.v_max ** 2))  + w1*(grad_sigmoid(trajectory_matrix[4, i])*(trajectory_matrix[4, i] ** 2 / p.v_max ** 2) + sigmoid(trajectory_matrix[4, i])*(trajectory_matrix[4, i] * 2) / (p.N * (p.v_max ** 2))) + w2*(-grad_sigmoid(-trajectory_matrix[4, i])*(trajectory_matrix[4, i] ** 2 / p.v_max ** 2) + sigmoid(-trajectory_matrix[4, i])*(trajectory_matrix[4, i] * 2) / (p.N * (p.v_max ** 2)))

    #ベクトルに直す
    jac_f = jac_f.flatten()
    
    return jac_f


def check_objective_function2(x, *args):
    w1, w2 = args[0], args[1]
    #matrixに変換
    trajectory_matrix = x.reshape(p.M, p.N)
    
    #phiの二乗和を目的関数とする。
    sum1, sum2, sum3, sum4 = 0, 0, 0, 0
    for i in range(p.N):
        sum1 += (trajectory_matrix[3, i] ** 2 / p.phi_max ** 2)

    for i in range(p.N):
        sum2 += (trajectory_matrix[4, i] ** 2 / p.v_max ** 2) 
    
    for i in range(p.N):
        sum3 += sigmoid(trajectory_matrix[4, i])*(trajectory_matrix[4, i] ** 2 / p.v_max ** 2) 
        
    for i in range(p.N):
        sum4 += sigmoid(-trajectory_matrix[4, i])*(trajectory_matrix[4, i] ** 2 / p.v_max ** 2) 
    
    return sum1 / p.N, sum2 / p.N, sum3 / p.N, sum4 / p.N