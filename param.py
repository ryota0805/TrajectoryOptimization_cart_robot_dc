#パラメータ管理class
import numpy as np
import env

class Parameter:
    env_data = env.Env()
    N = 30                                                      #系列データの長さ
    M = 5                                                       #設計変数の種類の個数
    
    #初期状態と終端状態
    set_cons = {'initial_x'     :True,                          #境界条件をセットするかどうか
                'terminal_x'    :True, 
                'initial_y'     :True, 
                'terminal_y'    :True, 
                'initial_theta' :True,
                'terminal_theta':True,
                'initial_omega' :True,
                'terminal_omega':True,
                'initial_v'     :True, 
                'terminal_v'    :True,
                }
    
    dt = 1                                                      #刻み幅[s]                                             
    d1 = 0
    d2 = 0.6+0.25*2
    
    #ロボットシステムの中心座標と姿勢からx1,y1,x2,y2の境界条件を計算する
    initial_x = 0
    terminal_x = 20
    
    initial_y = 0
    terminal_y = 7
    
    initial_theta = 0
    terminal_theta = 0
    
    initial_omega = 0
    terminal_omega = 0
    
    initial_v = 0                                               #v[m/s]
    terminal_v = 0                                              #v[m/s]
    
    
    #変数の範囲
    x_min = env_data.x_range[0]                                                  #x[m]
    x_max = env_data.x_range[1]                                                  #x[m]
    y_min = env_data.y_range[0]                                                 #y[m]
    y_max = env_data.y_range[1]                                                  #y[m]
    theta_min = -np.pi * 180/ 180                                         #theta[rad]
    theta_max = np.pi *180/180                                          #tehta[rad]
    omega_min = -np.pi/6
    omega_max = np.pi/6
    v_min = -2                                                   #v[m/s]
    v_max = 2                                                   #v[m/s]

    
    #ロボットの把持部分の最大値
    phi_max = np.pi/6
    WayPoint = np.array([[initial_x, initial_y],                #初期パス　[x, y] 
                        [3, 3],
                        [5, 3],
                        [terminal_x, terminal_y]])     

    #障害物のパラメータ 
    #　(x, y, r)
    #　x　: 円の中心座標
    #　y　: 円の中心座標
    #　r　: 半径 
    obstacle_list = [(10, -1, 3), (20, 1, 3)]                   #障害物のパラメータが格納されたリスト
    
    
    #wallのパラメータ
    wall_thick = 1                                #wallの厚さ
    margin = 2
    
    #robot size
    robot_size = 1
    
    #v, omegaの初期値との誤差
    error_omega = 0.001
    error_v = 0.001
