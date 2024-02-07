#不等式制約、等式制約を定義する
from param import Parameter as p
import util
import numpy as np
import env


#制約条件を計算するための運動学モデルの定式化
#dotx = v*cos(theta)
def kinematics_x(theta, v):
    return v*np.cos(theta)

#doty = v*cos(theta)
def kinematics_y(theta, v):
    return v*np.sin(theta)

#dot(theta) = omega
def kinematics_theta(omega):
    return omega

#xのcollocation point返す関数
def collocation_x(xs, thetas, vs):
    return 1/2*(xs[0] + xs[1]) + p.dt/8*(kinematics_x(thetas[0], vs[0]) - kinematics_x(thetas[1], vs[1]))

#yのcollocation point返す関数
def collocation_x(ys, thetas, vs):
    return 1/2*(ys[0] + ys[1]) + p.dt/8*(kinematics_y(thetas[0], vs[0]) - kinematics_y(thetas[1], vs[1]))

#thetaのcollocation point返す関数
def collocation_theta(thetas, omegas):
    return 1/2*(thetas[0] + thetas[1]) + p.dt/8*(kinematics_theta(omegas[0]) - kinematics_theta(omegas[1]))
    
    
def constraint(x, *args):
    env_data = env.Env()
    obs_rectangle = env_data.obs_rectangle
    obs_circle = env_data.obs_circle
    
    trajectory_matrix = x.reshape(p.M, p.N)
    x, y, theta, omega, v = trajectory_matrix[0], trajectory_matrix[1], trajectory_matrix[2], trajectory_matrix[3], trajectory_matrix[4]
    
    if args[0] == 'model':
        i = args[1][1]
        if args[1][0] == 'x':
            xs, thetas, omegas, vs = [x[i], x[i+1]], [theta[i], theta[i+1]], [omega[i], omega[i+1]], [v[i], v[i+1]]
            thetac = collocation_theta(thetas, omegas)
            vc = (vs[0] + vs[1])/2
            value = (xs[0] - xs[1]) + p.dt/6*(kinematics_x(thetas[0], vs[0]) + 4*kinematics_x(thetac, vc) + kinematics_x(thetas[1], vs[1])) 
        
        elif args[1][0] == 'y':
            ys, thetas, omegas, vs = [y[i], y[i+1]], [theta[i], theta[i+1]], [omega[i], omega[i+1]], [v[i], v[i+1]]
            thetac = collocation_theta(thetas, omegas)
            vc = (vs[0] + vs[1])/2
            value = (ys[0] - ys[1]) + p.dt/6*(kinematics_y(thetas[0], vs[0]) + 4*kinematics_y(thetac, vc) + kinematics_y(thetas[1], vs[1])) 
            
        elif args[1][0] == 'theta':
            thetas, omegas = [theta[i], theta[i+1]], [omega[i], omega[i+1]]
            omegac = (omegas[0] + omegas[1])/2
            value = (thetas[0] - thetas[1]) + p.dt/6*(kinematics_theta(omegas[0]) + 4*kinematics_theta(omegac) + kinematics_theta(omegas[1])) 
        
        else:
            return 'Error'

        return value


    elif args[0] == 'avoid_obstacle':
        if args[1][0] == 'rectangle':
            k, i = args[1][1], args[1][2]
            
            value = (((2*0.8/obs_rectangle[k][2]) ** 10) * (x[i] - (obs_rectangle[k][0] + obs_rectangle[k][2]/2)) ** 10 + ((2*0.8/obs_rectangle[k][3]) ** 10) * (y[i] - (obs_rectangle[k][1] + obs_rectangle[k][3]/2)) ** 10) - 1
            
            return value
        
        elif args[1][0] == 'circle':
            k, i = args[1][1], args[1][2]
            
            value = ((x[i] - obs_circle[k][0]) ** 2 + (y[i] - obs_circle[k][1]) ** 2) - (obs_circle[k][2] + p.robot_size) ** 2

            return value 
    
    elif args[0] == 'boundary':
        variable, ini_ter = args[1][0], args[1][1]
        
        if variable == 'x':
            if ini_ter == 'ini':
                value = x[0] - p.initial_x
            
            elif ini_ter == 'ter':
                value = x[-1] - p.terminal_x
                
        elif variable == 'y':
            if ini_ter == 'ini':
                value = y[0] - p.initial_y
            
            elif ini_ter == 'ter':
                value = y[-1] - p.terminal_y
                
        elif variable == 'theta':
            if ini_ter == 'ini':
                value = theta[0] - p.initial_theta
            
            elif ini_ter == 'ter':
                value = theta[-1] - p.terminal_theta
                
        elif variable == 'omega':
            if ini_ter == 'ini':
                value = omega[0] - p.initial_omega
            
            elif ini_ter == 'ter':
                value = omega[-1] - p.terminal_omega
                
        elif variable == 'v':
            if ini_ter == 'ini':
                value = v[0] - p.initial_v
            
            elif ini_ter == 'ter':
                value = v[-1] - p.terminal_v
                
        return value
    
    
    #不等式制約として与えるomega,vの初期値
    elif args[0] == 'ini':
        variable, i = args[1][0], args[1][1]
        if variable == 'omega':
            value = p.error_omega**2 - (omega[0] - p.initial_omega)**2
            
        elif variable == 'v':
            value = p.error_v**2 - (v[0] - p.initial_v)**2
            
        return value
    
    
def generate_cons():
    env_data = env.Env()
    obs_rectangle = env_data.obs_rectangle
    obs_circle = env_data.obs_circle
    
    cons = ()
    
    #障害物回避のための不等式制約を追加する
    #矩形
    for k in range(len(obs_rectangle)):
        for i in range(p.N):
            args = ['avoid_obstacle', ['rectangle', k, i]]
            cons = cons + ({'type':'ineq', 'fun': constraint, 'args': args},)
            
            
    #円形
    for k in range(len(obs_circle)):
        for i in range(p.N):
            args = ['avoid_obstacle', ['circle', k, i]]
            cons = cons + ({'type':'ineq', 'fun': constraint, 'args': args},)
            
            
    #運動学モデルの制約からなる等式制約を追加する
    #x
    for i in range(p.N-1):
        args = ['model', ['x', i]]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #y
    for i in range(p.N-1):
        args = ['model', ['y', i]]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
    
    #theta
    for i in range(p.N-1):
        args = ['model', ['theta', i]]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
        
    #境界値条件の等式制約を追加
    #x初期条件
    if p.set_cons['initial_x'] == False:
        pass
    else:
        args = ['boundary', ['x', 'ini']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #x終端条件
    if p.set_cons['terminal_x'] == False:
        pass
    else:
        args = ['boundary', ['x', 'ter']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)

    #y初期条件
    if p.set_cons['initial_y'] == False:
        pass
    else:
        args = ['boundary', ['y', 'ini']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #y終端条件
    if p.set_cons['terminal_y'] == False:
        pass
    else:
        args = ['boundary', ['y', 'ter']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #thet1初期条件
    if p.set_cons['initial_theta'] == False:
        pass
    else:
        args = ['boundary', ['theta', 'ini']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #theta終端条件
    if p.set_cons['terminal_theta'] == False:
        pass
    else:
        args = ['boundary', ['theta', 'ter']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
           
    #omega初期条件
    if p.set_cons['initial_omega'] == False:
        pass
    else:
        args = ['boundary', ['omega', 'ini']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #omega終端条件
    if p.set_cons['terminal_omega'] == False:
        pass
    else:
        args = ['boundary', ['omega', 'ter']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #v初期条件
    if p.set_cons['initial_v'] == False:
        pass
    else:
        args = ['boundary', ['v', 'ini']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
    #v終端条件
    if p.set_cons['terminal_v'] == False:
        pass
    else:
        args = ['boundary', ['v', 'ter']]
        cons = cons + ({'type':'eq', 'fun': constraint, 'args': args},)
        
        
    #不等式制約としてomega,vの初期値を与える
    args = ['ini', ['omega', 0]]
    cons = cons + ({'type':'ineq', 'fun': constraint, 'args': args},)
    
    args = ['ini', ['v', 0]]
    cons = cons + ({'type':'ineq', 'fun': constraint, 'args': args},)
    
    return cons


#変数の数だけタプルのリストとして返す関数
def generate_bounds():
    
    #boundsのリストを生成
    bounds = []
    
    #xの範囲
    for i in range(p.N):
        bounds.append((p.x_min, p.x_max))
        
    #yの範囲
    for i in range(p.N):
        bounds.append((p.y_min, p.y_max))
        
    #thetaの範囲
    for i in range(p.N):
        bounds.append((p.theta_min, p.theta_max))
        
    #omegaの範囲
    for i in range(p.N):
        bounds.append((p.omega_min, p.omega_max))
        
    #vの範囲
    for i in range(p.N):
        bounds.append((p.v_min, p.v_max))
        
    return bounds