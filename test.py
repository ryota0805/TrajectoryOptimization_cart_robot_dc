import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# バイクモデルのシステムダイナミクス
def bicycle_model(x, u, dt):
    """
    バイクモデルの状態方程式
    x: [x, y, theta, v] - 車両のx座標, y座標, 方向角, 速度
    u: [delta, a] - ステアリング角, アクセル
    dt: サンプリング時間
    """
    L = 2.5  # 車軸間の距離

    x_dot = x[3] * np.cos(x[2])
    y_dot = x[3] * np.sin(x[2])
    print(x,u)
    theta_dot = (x[3] / L) * np.tan(u[0])
    v_dot = u[1]

    x_new = x + np.array([x_dot, y_dot, theta_dot, v_dot]) * dt

    return x_new

# モデル予測制御の目的関数
def objective_function(u, x, x_ref, horizon):
    """
    目的関数 - 予測誤差の二乗和
    """
    dt = 0.1
    x_pred = np.zeros_like(x)
    print(x)
    for t in range(horizon):
        x = bicycle_model(x, u[t], dt)
        x_pred[t] = x

    return np.sum((x_pred[:, :2] - x_ref[:, :2])**2)

# モデル予測制御のシミュレーション
def simulate_mpc(initial_state, x_ref, horizon):
    """
    モデル予測制御のシミュレーション
    """
    u_init = np.zeros((horizon, 2))  # 初期入力
    result = minimize(objective_function, u_init.flatten(), args=(initial_state, x_ref, horizon),
                      method='SLSQP', bounds=[(-0.5, 0.5), (0, 5)] * horizon)

    u_optimal = result.x.reshape((horizon, 2))
    return u_optimal

# シミュレーションのパラメータ
initial_state = np.array([0, 0, 0, 0])  # 初期状態 [x, y, theta, v]
x_ref = np.array([[5, 5, 0, 0]] * 10)  # 目標軌道
horizon = 3  # 予測ホリゾン

# モデル予測制御のシミュレーション実行
u_optimal = simulate_mpc(initial_state, x_ref, horizon)

# 結果のプロット
plt.figure(figsize=(10, 6))
plt.plot(x_ref[:, 0], x_ref[:, 1], label='Reference Path', marker='o')
plt.title('Model Predictive Control for Bicycle Model')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.grid(True)
plt.show()