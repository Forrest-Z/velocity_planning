#! /usr/bin/python3
#! -*- coding:utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import rospy
import roslib
import os

# 中值滤波, window_size为奇数
def medianFilter(data, window_size):
    half_size = int(window_size / 2);
    for i in range(0, len(data)):
        # 找出对应点的窗
        window_data = []
        for j in range(i - half_size, i + half_size):
            window_data.append(data[min(len(data) - 1, max(j, 0))])
        # 找到window_data中间值
        median_data = np.median(window_data)
        data[i] = median_data

# 解析文件
def parseFile(file_path):
    position_x, position_y = [], []
    arc_lengths = []
    yaws = []
    curvatures = []
    file = open(file_path)
    data = file.readlines()
    arc_length = 0.0
    for i, line in enumerate(data):
        line = line.strip().split(',')
        position_x.append(float(line[0]))
        position_y.append(float(line[1]))
        yaws.append(float(line[2]))
        curvatures.append(float(line[3]))
        if i != 0:
            arc_length += np.sqrt((position_x[i] - position_x[i - 1]) ** 2 + (position_y[i] - position_y[i - 1]) ** 2)
        arc_lengths.append(arc_length)
    return position_x, position_y, yaws, curvatures, arc_lengths


if __name__ == "__main__":
    # 读取文件路径
    direction = roslib.packages.get_pkg_dir("motion_planning") + "/curve_record/"
    total_file_names = os.listdir(direction)
    # 对文件名按时间进行排序
    total_file_names.sort(key = lambda k: int(k.split('.')[0]))
    # 创建可视化窗口
    fig = plt.figure(figsize=(14, 14))
    ax1 = fig.add_subplot(4, 1, 1)
    ax2 = fig.add_subplot(4, 1, 2)
    ax3 = fig.add_subplot(4, 1, 3)
    ax4 = fig.add_subplot(4, 1, 4)
    for file_name in total_file_names:
        print(file_name)
        # 进行可视化
        # 解析文件
        position_x, position_y, yaws, curvatures, arc_lengths = parseFile(direction + file_name)
        # 计算曲率变化率
        curvatures_change_rates = []
        for i,_ in enumerate(curvatures):
            if i < len(curvatures) - 1:
                curvatures_change_rate = (curvatures[i + 1] - curvatures[i]) / (arc_lengths[i + 1] - arc_lengths[i])
                curvatures_change_rates.append(curvatures_change_rate)
            else:
                curvatures_change_rate = 0
                curvatures_change_rates.append(curvatures_change_rate)
        # 进行中值滤波
        medianFilter(curvatures_change_rates, 5)
        # 清空之前的可视化
        ax1.cla()
        ax2.cla()
        ax3.cla()
        ax4.cla()
        # 图1
        ax1.grid(b=True,which='major',axis='both',alpha= 0.5,color='skyblue',linestyle='--',linewidth=2)
        ax1.axis('equal')
        ax1.plot(position_x, position_y)
        # 图2
        ax2.grid(b=True,which='major',axis='both',alpha= 0.5,color='skyblue',linestyle='--',linewidth=2)
        ax2.plot(arc_lengths, yaws)
        # 图3
        ax3.grid(b=True,which='major',axis='both',alpha= 0.5,color='skyblue',linestyle='--',linewidth=2)
        ax3.plot(arc_lengths, curvatures)
        # 图4
        ax4.grid(b=True,which='major',axis='both',alpha= 0.5,color='skyblue',linestyle='--',linewidth=2)
        ax4.plot(arc_lengths, curvatures_change_rates)
        # 进行显示
        plt.pause(1)
