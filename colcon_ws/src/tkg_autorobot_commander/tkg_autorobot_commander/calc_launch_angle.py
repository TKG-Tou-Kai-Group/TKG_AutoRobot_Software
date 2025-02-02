#!/usr/bin/env python3
import os
import math
import pickle
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

ROLLER_RADIUS = 0.02

# ===== グローバル変数 =====
# 初速 (v) の最小値、最大値、刻み幅
V_MIN = 3.0
V_MAX = 12.6
V_STEP = 0.2

# 目標距離 (l) の最小値、最大値、刻み幅
L_MIN = 0.1
L_MAX = 15.0
L_STEP = 0.1

# 目標高さ (h) の最小値、最大値、刻み幅
H_MIN = -0.1
H_MAX = 0.5
H_STEP = 0.05

# ルックアップテーブルのファイル名
LOOKUP_FILENAME = "lookup_table.pkl"

def calc_launch_angles(v, l, h, g=9.8):
    """
    初速 v [m/s] で斜方投射したとき、水平距離 l [m]、高さ h [m] の位置に到達するための
    射出角 θ（ラジアン）と到達時刻 t を計算する関数です。
    
    パラメータ:
      v: 初速 [m/s]
      l: 目標までの水平距離 [m]
      h: 目標の高さ [m]
      g: 重力加速度 [m/s^2]（デフォルトは 9.8）
      
    戻り値:
      解のリスト。各解は辞書 { 'launch_angle': θ, 'flight_time': t } を含みます。
    """
    # 斜方投射の運動方程式から、目標 (l, h) に到達する条件は
    # h = l tanθ - (g l^2)/(2 v^2 cos²θ)
    # となります。ここで u = tanθ とおくと、以下の2次方程式が得られます：
    # (g l^2) u² - (2 v² l) u + (g l² + 2 v² h) = 0
    a = g * l**2
    b = -2 * v**2 * l
    c = g * l**2 + 2 * v**2 * h

    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        raise ValueError("与えられた条件では解が存在しません（目標に届かないか、初速不足です）。")
    
    solutions = []
    # 2 つの解（場合によっては重解となる）を求めます
    for sign in [+1, -1]:
        u = (-b + sign * np.sqrt(discriminant)) / (2 * a)  # u = tanθ
        theta = np.arctan(u)  # 射出角 [rad]
        cos_theta = np.cos(theta)
        if np.abs(cos_theta) < 1e-8:
            continue  # 実用的でない解は除外
        # x方向は等速運動なので、到達時刻は t = l / (v cosθ)
        t = l / (v * cos_theta)
        solutions.append({
            'launch_angle': theta,
            'flight_time': t,
        })

    # 得られた解の中から、到達時刻が最も早い（tが最小の）解を選択
    best_solution = min(solutions, key=lambda sol: sol['flight_time'])

    return best_solution['launch_angle']


# 定数・パラメータの設定
g = 9.8             # 重力加速度 [m/s²]
rho = 1.225         # 空気密度 [kg/m³]
mass = 0.03         # 質量 [kg]
diameter = 0.07     # 直径 [m]
A = np.pi * (diameter/2)**2  # 断面積 [m²]
C_D = 0.47          # 抗力係数（球体の場合）

def projectile_with_drag(t, state):
    """状態変数 state = [x, y, vx, vy] に対する常微分方程式（空気抵抗あり）"""
    x, y, vx, vy = state
    v = np.hypot(vx, vy)
    # 空気抵抗による加速度
    drag = 0.5 * rho * C_D * A * v / mass
    ax = -drag * vx
    ay = -g - drag * vy
    return [vx, vy, ax, ay]

def event_x(t, state, target_x):
    """x 座標が target_x に達したときのイベント関数"""
    return state[0] - target_x

# イベントで統合を終了するための設定
event_x.terminal = True
event_x.direction = 1  # x が target_x を上向きに通過したとき

def simulate_trajectory(theta, v0, target_x):
    """
    射出角 theta [rad] で初速 v0 [m/s] で投射した場合の
    目標 x 座標 (target_x) における y 座標を返す。
    もし x=target_x に到達しなかった場合は None を返す。
    """
    vx0 = v0 * np.cos(theta)
    vy0 = v0 * np.sin(theta)
    state0 = [0, 0, vx0, vy0]
    # t_span は十分長い時間を設定
    sol = solve_ivp(projectile_with_drag, [0, 2.0], state0,
                    events=lambda t, s: event_x(t, s, target_x),
                    max_step=0.01)
    
    if sol.t_events[0].size > 0:
        # x = target_x に達した時刻での y 座標（イベントで終了しているので、最後の状態が目標近傍）
        y_at_target = sol.y_events[0][0][1]
        return y_at_target
    else:
        # x = target_x に到達しなかった場合
        return None

def f(theta, v0, target_x, target_y):
    """
    目的関数: 射出角 theta でのシミュレーション結果 y(target_x) と目標高さ target_y の差
    f(theta)= y(target_x) - target_y
    """
    y_at_target = simulate_trajectory(theta, v0, target_x)
    if y_at_target is None:
        # 到達しなかった場合、誤差を大きな値として返す
        return -1e3
    return y_at_target - target_y

def calc_launch_angles_with_drag(v, l, h):

    # 入力パラメータ
    v0 = v      # 初速 [m/s]
    target_x = l # 目標の水平距離 [m]
    target_y = h # 目標の高さ [m]
    
    # 射出角の探索区間 [rad]
    theta_min = np.radians(0)      # 極小角
    theta_max = np.radians(28)     # 極大角
    
    # ルート探索の前に、区間の両端で f(theta) の符号が逆転しているか確認
    f_min = f(theta_min, v0, target_x, target_y)
    f_max = f(theta_max, v0, target_x, target_y)
    
    if f_min * f_max > 0:
        raise ValueError("探索区間内で符号の逆転が見つかりません。条件に合う解が存在しない可能性があります。")
    
    # ルート探索（ブレント法）で f(theta)=0 となる theta を求める
    return brentq(f, theta_min, theta_max, args=(v0, target_x, target_y))
    
# ===== ルックアップテーブル生成関数 =====
def generate_lookup_table():
    """
    グローバル変数で定義した範囲・刻み幅に基づいてルックアップテーブルを生成し、
    辞書形式で返す。
    キーは (v, l, h) のタプル（小数点以下の丸めをしてキーの誤差を抑える）、
    値は calc_launch_angles_with_drag の計算結果（射出角：ラジアン）。
    """
    lookup_table = {}
    v_values = np.arange(V_MIN, V_MAX + V_STEP, V_STEP)
    l_values = np.arange(L_MIN, L_MAX + L_STEP, L_STEP)
    h_values = np.arange(H_MIN, H_MAX + H_STEP, H_STEP)

    total = len(v_values) * len(l_values) * len(h_values)
    count = 0

    for v in v_values:
        for l in l_values:
            for h in h_values:
                try:
                    angle = calc_launch_angles_with_drag(v, l, h)
                except ValueError as e:
                    angle = -1.0
                # キーは丸めた値で保存（浮動小数点誤差対策）
                key = (round(v, 5), round(l, 5), round(h, 5))
                lookup_table[key] = angle
                count += 1
                # 進捗表示（必要に応じてコメントアウト）
                if count % 10000 == 0:
                    print(f"{count} / {total} entries computed...")
    return lookup_table


# ===== ルックアップテーブルのロード or 生成 =====
if os.path.exists(LOOKUP_FILENAME):
    # 既存ファイルからルックアップテーブルを読み込む
    with open(LOOKUP_FILENAME, "rb") as f:
        lookup_table = pickle.load(f)
    print("ルックアップテーブルをファイルからロードしました。")
else:
    # ファイルがなければ生成し、保存する
    print("ルックアップテーブルを生成中...")
    lookup_table = generate_lookup_table()
    with open(LOOKUP_FILENAME, "wb") as f:
        pickle.dump(lookup_table, f)
    print("ルックアップテーブルの生成と保存が完了しました。")


# ===== ルックアップテーブルを利用して射出角を取得する関数 =====
def lookup_launch_angles_with_drag(v, l, h):
    """
    入力された初速 v、目標距離 l、目標高さ h に対して、
    ルックアップテーブルから射出角（ラジアン）を返す。
    テーブルにない場合は、重い計算を直接実行する。
    """
    # 入力値をグローバルな刻み幅に合わせて丸める
    v_key = round(round((v - V_MIN) / V_STEP) * V_STEP + V_MIN, 5)
    l_key = round(round((l - L_MIN) / L_STEP) * L_STEP + L_MIN, 5)
    h_key = round(round((h - H_MIN) / H_STEP) * H_STEP + H_MIN, 5)

    key = (v_key, l_key, h_key)
    if key in lookup_table:
        return lookup_table[key]
    else:
        # ルックアップテーブルに該当する値がない場合は直接計算
        print("ルックアップテーブルにない組み合わせです。直接計算します。")
        return calc_launch_angles_with_drag(v, l, h)

def calc_best_launch_angle(rpm, l, h):
    v = 2.0 * math.pi * ROLLER_RADIUS * rpm / 60.0
    try:
        #launch_angle = calc_launch_angles(v, l, h)
        launch_angle = lookup_launch_angles_with_drag(v, l, h)
    except ValueError as e:
        #print(e)
        return -1.0

    return launch_angle
