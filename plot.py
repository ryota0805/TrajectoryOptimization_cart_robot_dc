import matplotlib.pyplot as plt
import matplotlib.patches as patches
from param import Parameter as p
import util
import numpy as np
import GenerateInitialPath
import matplotlib.cm as cm
import objective_function

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from param import Parameter as p
import util
import numpy as np
import env


########
#壁と障害物の配置し表示する関数
########
def vis_env():
    fig, ax = plt.subplots()
    
    env_data = env.Env()
    wall_list = env_data.obs_boundary
    obs_rectangle = env_data.obs_rectangle
    obs_circle = env_data.obs_circle
    
    #wallを配置
    for k in range(len(wall_list)):
        wall = patches.Rectangle((wall_list[k][0], wall_list[k][1]), wall_list[k][2], wall_list[k][3], linewidth=1, edgecolor='black', facecolor='black')
        ax.add_patch(wall)
    
    #障害物を配置
    for k in range(len(obs_rectangle)):
        x0, y0, w, h = obs_rectangle[k][0], obs_rectangle[k][1], obs_rectangle[k][2], obs_rectangle[k][3]
        rectangle_obstacle = patches.Rectangle((x0, y0), w, h, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rectangle_obstacle)
        
    for k in range(len(obs_circle)):
        x_o, y_o, r_o = obs_circle[k][0], obs_circle[k][1], obs_circle[k][2],
        circle_obstacle = patches.Circle((x_o, y_o), radius=r_o, edgecolor='black', facecolor='gray')
        ax.add_patch(circle_obstacle)
    
    """
    #startとgoalを配置
    ax.scatter([p.initial_x], [p.initial_y], marker='v', color='green', label='start')
    ax.scatter([p.terminal_x], [p.terminal_y], marker='^', color='green', label='goal')
    """
    
    ax.set_xlabel(r'$x$[m]')
    ax.set_ylabel(r'$y$[m]')
    ax.set_xlim([p.x_min - p.margin, p.x_max + p.margin])
    ax.set_ylim([p.y_min - p.margin, p.y_max + p.margin])
    
    ax.set_aspect('equal')
    plt.show()
    
    return None
    
    
########    
#経路を環境に表示する関数
########
def vis_path(x, y):
    fig, ax = plt.subplots()
    
    
    ax.plot(x, y, marker='x', color='red')
    
    env_data = env.Env()
    wall_list = env_data.obs_boundary
    obs_rectangle = env_data.obs_rectangle
    obs_circle = env_data.obs_circle
    
    #wallを配置
    for k in range(len(wall_list)):
        wall = patches.Rectangle((wall_list[k][0], wall_list[k][1]), wall_list[k][2], wall_list[k][3], linewidth=1, edgecolor='black', facecolor='black')
        ax.add_patch(wall)
    
    #障害物を配置
    for k in range(len(obs_rectangle)):
        x0, y0, w, h = obs_rectangle[k][0], obs_rectangle[k][1], obs_rectangle[k][2], obs_rectangle[k][3]
        rectangle_obstacle = patches.Rectangle((x0, y0), w, h, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rectangle_obstacle)
        
    for k in range(len(obs_circle)):
        x_o, y_o, r_o = obs_circle[k][0], obs_circle[k][1], obs_circle[k][2],
        circle_obstacle = patches.Circle((x_o, y_o), radius=r_o, edgecolor='black', facecolor='gray')
        ax.add_patch(circle_obstacle)
    
    #startとgoalを配置
    ax.scatter([x[0]], [y[0]], marker='v', color='green', label='start')
    ax.scatter([x[-1]], [y[-1]], marker='^', color='green', label='goal')
    
    ax.set_xlabel(r'$x$[m]')
    ax.set_ylabel(r'$y$[m]')
    ax.set_xlim([p.x_min - p.margin, p.x_max + p.margin])
    ax.set_ylim([p.y_min - p.margin, p.y_max + p.margin])
    
    ax.set_aspect('equal')
    ax.legend(loc="best")
    plt.show()
    
    return None

########
#2本のpathを比較する関数
########
def compare_path(x1, y1, x2, y2):
    fig, ax = plt.subplots()
    
    #2本のpathを配置
    ax.plot(x1, y1, marker='', color='red',label='robot1')
    ax.plot(x2, y2, marker='', color='blue',label='robot2')
    
    env_data = env.Env()
    wall_list = env_data.obs_boundary
    obs_rectangle = env_data.obs_rectangle
    obs_circle = env_data.obs_circle
    
    #wallを配置
    for k in range(len(wall_list)):
        wall = patches.Rectangle((wall_list[k][0], wall_list[k][1]), wall_list[k][2], wall_list[k][3], linewidth=1, edgecolor='black', facecolor='black')
        ax.add_patch(wall)
    
    #障害物を配置
    for k in range(len(obs_rectangle)):
        x0, y0, w, h = obs_rectangle[k][0], obs_rectangle[k][1], obs_rectangle[k][2], obs_rectangle[k][3]
        rectangle_obstacle = patches.Rectangle((x0, y0), w, h, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rectangle_obstacle)
        
    for k in range(len(obs_circle)):
        x_o, y_o, r_o = obs_circle[k][0], obs_circle[k][1], obs_circle[k][2],
        circle_obstacle = patches.Circle((x_o, y_o), radius=r_o, edgecolor='black', facecolor='gray')
        ax.add_patch(circle_obstacle)
    """
    #startとgoalを配置
    ax.scatter([x1[0]], [y1[0]], marker='v', color='green', label='start')
    ax.scatter([x1[-1]], [y1[-1]], marker='^', color='green', label='goal')
    """
    ax.set_xlabel(r'$x$[m]')
    ax.set_ylabel(r'$y$[m]')
    ax.set_xlim([p.x_min - p.margin, p.x_max + p.margin])
    ax.set_ylim([p.y_min - p.margin, p.y_max + p.margin])

    ax.set_aspect('equal')
    ax.legend(loc="best")
    plt.show()
    
    return None
    
########
#姿勢thetaのグラフを生成
########
def vis_history_theta(theta, range_flag = False):
    fig, ax = plt.subplots()
    
    ax.plot(theta, color='blue', label=r'$\theta$[rad]')
    ax.set_xlabel(r'$t$[s]')
    ax.set_ylabel(r'$\theta$[rad]')
    ax.legend(loc='upper right')
    
    #thetaの範囲を追加
    if range_flag:
        theta_max_list = [p.theta_max for i in range(p.N)]
        theta_min_list = [p.theta_min for i in range(p.N)]
        ax.plot(theta_max_list, color='green', linestyle='-.')
        ax.plot(theta_min_list, color='green', linestyle='-.')
    else:
        pass
    
    plt.show()
    

########
#姿勢の比較
########
def history_robot_theta(theta1, theta2, range_flag = False):
    fig, ax = plt.subplots()
    
    ax.plot(theta1,  color='red',  label='theta1')
    
    ax.plot(theta2,  color='blue',  label='theta2')
    
    ax.set_xlabel(r'$t$[s]')
    ax.set_ylabel(r'$\theta$[rad]')
    ax.legend(loc='upper right')
    
    #thetaの範囲を追加
    if range_flag:
        theta_max_list = [p.theta1_max for i in range(p.N)]
        theta_min_list = [p.theta1_min for i in range(p.N)]
        ax.plot(theta_max_list, color='green', linestyle='-.')
        ax.plot(theta_min_list, color='green', linestyle='-.')
    else:
        pass
    
    plt.show()
    
 
########
#ステアリング角phiのグラフを生成
########
def vis_history_phi(trajectory_vector, range_flag = False):
    fig, ax = plt.subplots()
    
    trajectory_matrix = util.vector_to_matrix(trajectory_vector)
    
    phi = trajectory_matrix[3]
    
    ax.plot(phi, color='blue', label=r'$\phi$[rad]')
    ax.set_xlabel(r'$t$[s]')
    ax.set_ylabel(r'$\phi$[rad]')
    ax.legend(loc='upper right')
    
    #thetaの範囲を追加
    if range_flag:
        phi_max_list = [p.phi_max for i in range(p.N)]
        phi_min_list = [p.phi_min for i in range(p.N)]
        ax.plot(phi_max_list, color='green', linestyle='-.')
        ax.plot(phi_min_list, color='green', linestyle='-.')
    else:
        pass
    
    plt.show()
    

########
#ステアリング角の比較
########
def history_robot_phi(phi1, phi2, range_flag = False):
    fig, ax = plt.subplots()
    
    ax.plot(phi1,  color='red',  label='phi1')
    ax.plot(phi2,  color='blue',  label='phi2')
    
    ax.set_xlabel(r'$t$[s]')
    ax.set_ylabel(r'$\phi$[rad]')
    ax.legend(loc='upper right')
    
    #thetaの範囲を追加
    if range_flag:
        phi_max_list = [p.phi_max for i in range(p.N)]
        phi_min_list = [-p.phi_max for i in range(p.N)]
        ax.plot(phi_max_list, color='green', linestyle='-.')
        ax.plot(phi_min_list, color='green', linestyle='-.')
    else:
        pass
    
    plt.show()
    

########
#速さvのグラフを生成
########
def vis_history_v(trajectory_vector, range_flag = False):
    fig, ax = plt.subplots()
    
    trajectory_matrix = util.vector_to_matrix(trajectory_vector)
    
    v = trajectory_matrix[4]
    
    ax.plot(v, color='blue', label=r'$v$[m/s]')
    ax.set_xlabel(r'$t$[s]')
    ax.set_ylabel(r'$v$[m/s]')
    ax.legend(loc='upper right')
    
    #thetaの範囲を追加
    if range_flag:
        v_max_list = [p.v_max for i in range(p.N)]
        v_min_list = [p.v_min for i in range(p.N)]
        ax.plot(v_max_list, color='green', linestyle='-.')
        ax.plot(v_min_list, color='green', linestyle='-.')
    else:
        pass
    
    plt.show()
    
########
#角速度の比較
########
def history_robot_omega(omega1, omega2, range_flag = False):
    fig, ax = plt.subplots()
    

    ax.plot(omega1,  color='red',  label='omega1')
    ax.plot(omega2,  color='blue',  label='omega2')
    
    ax.set_xlabel(r'$t$[s]')
    ax.set_ylabel(r'$\omega$[rad/s]')
    ax.legend(loc='upper right')
    
    #thetaの範囲を追加
    if range_flag:
        v_max_list = [p.omega_max for i in range(p.N)]
        v_min_list = [p.omega_min for i in range(p.N)]
        ax.plot(v_max_list, color='green', linestyle='-.')
        ax.plot(v_min_list, color='green', linestyle='-.')
    else:
        pass
    
    plt.show()
    

########
#速さの比較
########
def history_robot_v(v1, v2, range_flag = False):
    fig, ax = plt.subplots()
    

    ax.plot(v1,  color='red',  label='v1')
    ax.plot(v2,  color='blue',  label='v2')
    
    ax.set_xlabel(r'$t$[s]')
    ax.set_ylabel(r'$v$[m/s]')
    ax.legend(loc='upper right')
    
    #thetaの範囲を追加
    if range_flag:
        v_max_list = [p.v_max for i in range(p.N)]
        v_min_list = [p.v_min for i in range(p.N)]
        ax.plot(v_max_list, color='green', linestyle='-.')
        ax.plot(v_min_list, color='green', linestyle='-.')
    else:
        pass
    
    plt.show()

#iteratoinごとの経路を表示する
def path_by_iteration(x_list):
    fig, ax = plt.subplots()

    for i in range(np.shape(x_list)[0]):
        x = x_list[i]
        #vectorをmatrixに変換
        trajectory_matrix = util.vector_to_matrix(x)
        x, y = trajectory_matrix[0], trajectory_matrix[1]
        
        #change color by path
        ax.scatter(x, y, marker='x', color=cm.Reds(i/np.shape(x_list)[0]), s=5)
    
    #wallを配置
    #左側
    leftside_wall = patches.Rectangle((p.x_min - p.wall_thick, p.y_min), p.wall_thick, p.y_max - p.y_min, linewidth=1, edgecolor='black', facecolor='black')
    ax.add_patch(leftside_wall)
    #右側
    rightside_wall = patches.Rectangle((p.x_max, p.y_min), p.wall_thick, p.y_max - p.y_min, linewidth=1, edgecolor='black', facecolor='black')
    ax.add_patch(rightside_wall)
    #下側
    downside_wall = patches.Rectangle((p.x_min - p.wall_thick, p.y_min - p.wall_thick), 2 * p.wall_thick + p.x_max - p.x_min, p.wall_thick, linewidth=1, edgecolor='black', facecolor='black')
    ax.add_patch(downside_wall)
    #上側
    upside_wall = patches.Rectangle((p.x_min - p.wall_thick, p.y_max), 2 * p.wall_thick + p.x_max - p.x_min, p.wall_thick, linewidth=1, edgecolor='black', facecolor='black')
    ax.add_patch(upside_wall)
    
    #障害物を配置
    for k in range(len(p.obstacle_list)):
        x_o, y_o, r_o = p.obstacle_list[k][0], p.obstacle_list[k][1], p.obstacle_list[k][2],
        circle_obstacle = patches.Circle((x_o, y_o), radius=r_o, edgecolor='black', facecolor='gray')
        ax.add_patch(circle_obstacle)
    
    #startとgoalを配置
    ax.scatter([p.initial_x], [p.initial_y], marker='v', color='green', label='start')
    ax.scatter([p.terminal_x], [p.terminal_y], marker='^', color='green', label='goal')
    
    ax.set_xlabel(r'$x$[m]')
    ax.set_ylabel(r'$y$[m]')
    ax.set_xlim([p.x_min - p.margin, p.x_max + p.margin])
    ax.set_ylim([p.y_min - p.margin, p.y_max + p.margin])
    
    ax.set_aspect('equal')
    ax.legend(loc="best")
    plt.show()
    
    return None


def function_by_iteration(x_list):
    func_list = []
    for i in range(np.shape(x_list)[0]):
        func_value = objective_function.objective_function(x_list[i])
        func_list.append(func_value)
    plt.plot(func_list)
    plt.xlabel(r'$Iteration$')
    plt.ylabel(r'$Objective Function$')
    plt.show()
    
    
#各waypointのconstraintのvalueを表示
def vis_constraint_values(constraint_list):
    for i in range(p.N):
        const1 = constraint_list[0][i]
        const2 = constraint_list[1][i]
        plt.plot(const1, label='obstacle1')
        plt.plot(const2, label='obstacle2')
        plt.xlabel(r'$Iteration$')
        plt.ylabel(r'$Value$')
        plt.title("Way Point {}".format(i + 1))
        plt.legend()
        plt.savefig("constraint_fig/" + "{}.png".format(i + 1))
        plt.clf()
        
#constraint_numberの履歴
def vis_constraint_number(constraint_number_list):
    plt.plot(constraint_number_list)
    plt.xlabel(r'$Iteration$')
    plt.ylabel(r'$Number$')
    plt.show()

        
    