#!/usr/bin/env python3
import math
import numpy as np

ROLLER_RADIUS = 0.02

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
    return solutions

def calc_best_launch_angle(rpm, l, h):
    v = 2.0 * math.pi * ROLLER_RADIUS * rpm / 60.0
    try:
        launch_solutions = calc_launch_angles(v, l, h)
    except ValueError as e:
        print(e)
        return -1.0

    # 得られた解の中から、到達時刻が最も早い（tが最小の）解を選択
    best_solution = min(launch_solutions, key=lambda sol: sol['flight_time'])
    
    # 結果表示（角度は度に変換）
    return best_solution['launch_angle']
