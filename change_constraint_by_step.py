from param import Parameter as p
import GenerateInitialPath
import util
import constraints
import matplotlib.pyplot as plt
import numpy as np
import scipy.optimize as optimize
import objective_function 
import plot

#WayPointから設計変数の初期値を計算する
cubicX, cubicY = GenerateInitialPath.cubic_spline()
x, y, theta, phi, v = GenerateInitialPath.generate_initialpath(cubicX, cubicY)
trajectory_matrix = np.array([x, y, theta, phi, v])
trajectory_vector = util.matrix_to_vector(trajectory_matrix)

#目的関数の設定
func = objective_function.objective_function

#制約条件の設定
cons = constraints.generate_constraints()

#変数の範囲の設定
bounds = constraints.generate_bounds()

#オプションの設定
options = {'maxiter':1000}

x_list = []
constraint1_list = []
constraint2_list = []

for i in range(p.N):
    constraint1_list.append([])
    constraint2_list.append([])
constraint_list = [constraint1_list, constraint2_list]    

constraint_number_list=[]
rate = [-0.0073 * x ** 2 + 400 for x in range(300)]
#callback
def callback(xk, threshold=3):
    x_list.append(xk)
    constraint_number=0
    counter = 0
    for k in range(len(p.obstacle_list)):
        for i in range(p.N):
            constraint_value = cons[counter]['fun'](xk)
            constraint_list[k][i].append(constraint_value)
            counter = counter + 1
            
            #threshold以上なら制約から除外
            if constraint_value <= rate[callback.iteration]:
                constraint_number = constraint_number + 1
            else:
                pass
            
    callback.iteration+=1    
    constraint_number_list.append(constraint_number)
callback.iteration = 0

#最適化を実行
result = optimize.minimize(func, trajectory_vector, method='SLSQP', constraints=cons, bounds=bounds, options=options, callback=callback)

plot.vis_constraint_values(constraint_list)
plot.vis_constraint_number(constraint_number_list)
#最適化結果の表示
#print(np.shape(x_list))
#print(len(cons))
print(result)
#plot.vis_env()
#plot.vis_path(trajectory_vector)
#plot.compare_path(trajectory_vector, result.x)
#plot.compare_history_theta(trajectory_vector, result.x, range_flag = True)
#plot.compare_history_phi(trajectory_vector, result.x, range_flag = True)
#plot.compare_history_v(trajectory_vector, result.x, range_flag = True)
#plot.vis_history_theta(result.x, range_flag=True)
#plot.vis_history_phi(result.x, range_flag=True)
#plot.vis_history_v(result.x, range_flag = True)
#plot.compare_path_rec(trajectory_vector, result.x)
#plot.path_by_iteration(x_list)
#plot.function_by_iteration(x_list)




