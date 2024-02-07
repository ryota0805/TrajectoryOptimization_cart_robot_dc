#初期パスを生成するファイル

import numpy as np
from scipy import interpolate
from param import Parameter as p
import random
import copy

########
#WayPointから3次スプライン関数を生成し、状態量をサンプリングする
########

#3次スプライン関数の生成
def cubic_spline(x, y):   
        
    tck,u = interpolate.splprep([x,y], k=3, s=0) 
    u = np.linspace(0, 1, num=100, endpoint=True)
    spline = interpolate.splev(u, tck)
    cubicX = spline[0]
    cubicY = spline[1]
    return cubicX, cubicY

#3次スプライン関数の生成(経路が関数の引数として与えられる場合)
def cubic_spline_by_waypoint(waypoint):   
    x, y = [], []
    for i in range(len(waypoint)):
        x.append(waypoint[i][0])
        y.append(waypoint[i][1])
        
    tck,u = interpolate.splprep([x,y], k=3, s=0) 
    u = np.linspace(0, 1, num=p.N, endpoint=True)
    spline = interpolate.splev(u, tck)
    cubicX = spline[0]
    cubicY = spline[1]
    return cubicX, cubicY

#x, yからΘとφを生成する
def generate_initialpath(cubicX, cubicY):
    #nd.arrayに変換
    x = np.array(cubicX)
    y = np.array(cubicY)
    
    #x, yの差分を計算
    deltax = np.diff(x)
    deltay = np.diff(y)
    
    #x, y の差分からthetaを計算
    #theta[0]を初期値に置き換え、配列の最後に終端状態を追加
    theta = np.arctan(deltay / deltax)
    theta[0] = p.initial_theta
    theta = np.append(theta, p.terminal_theta)
    
    #thetaの差分からphiを計算
    #phi[0]を初期値に置き換え配列の最後に終端状態を追加
    deltatheta = np.diff(theta)
    phi = deltatheta / p.dt
    phi[0] = p.initial_phi
    phi = np.append(phi, p.terminal_phi)
    
    #x,yの差分からvを計算
    #phi[0]を初期値に置き換え配列の最後に終端状態を追加
    v = np.sqrt((deltax ** 2 + deltay ** 2) / p.dt)
    v[0] = p.initial_v
    v = np.append(v, p.terminal_v)
    return x, y, theta, phi, v


#x, yからΘとφを生成する
def generate_initialpath2(cubicX, cubicY):
    t = np.linspace(0, p.N, p.N)
    
    fx = interpolate.Akima1DInterpolator(t, cubicX)
    fy = interpolate.Akima1DInterpolator(t, cubicY)
    
    dfx_dt = fx.derivative()
    dfy_dt = fy.derivative()
    
    #nd.arrayに変換
    x = fx(t)
    y = fy(t)
    
    #x, yの差分を計算
    dx_dt = dfx_dt(t)
    dy_dt = dfy_dt(t)
    
    x1, y1 = x.copy(), y.copy()
    x2, y2 = x.copy(), y.copy()
    
    #x, y の差分からthetaを計算
    #theta[0]を初期値に置き換え、配列の最後に終端状態を追加
    theta = np.arctan(dy_dt / dx_dt)
    
    
    #x,yの差分からvを計算
    #phi[0]を初期値に置き換え配列の最後に終端状態を追加
    v = np.sqrt((dx_dt ** 2 + dy_dt ** 2))
    v2 = v.copy()
    
    theta1 = theta.copy()
    theta2 = theta.copy()
    
    omega, omega2 = 0, 0
    return x, y, theta, omega, v


#theta, phi, vの初期値をランダムに生成
def generate_initialpath_randomly(cubicX, cubicY):
    t = np.linspace(0, p.N, p.N)
    
    fx = interpolate.Akima1DInterpolator(t, cubicX)
    fy = interpolate.Akima1DInterpolator(t, cubicY)
    
    #nd.arrayに変換
    x = fx(t)
    y = fy(t)
    theta = 0
    omega1 = np.array([random.uniform(p.omega1_min, p.omega1_max) for i in range(p.N)])
    omega2 = np.array([random.uniform(p.omega2_min, p.omega2_max) for i in range(p.N)])
    v1 = np.array([random.uniform(p.v1_min, p.v1_max) for i in range(p.N)])
    v2 = np.array([random.uniform(p.v2_min, p.v2_max) for i in range(p.N)])
    
    
    return x, y, x, y, theta, theta, omega1, omega2, v1, v2


def initial_zero(a):
    return np.array([a for i in range(p.N)]), np.array([a for i in range(p.N)]), np.array([a for i in range(p.N)]), np.array([a for i in range(p.N)]), np.array([a for i in range(p.N)])

