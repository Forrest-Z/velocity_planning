#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import os
import re

# 要读取的日志文件
user_home_path = os.environ['HOME'] + "/"
log_file_path = user_home_path + "motion_planning_log/motion_planning_node.INFO"
log_file = open(log_file_path, 'r')

# 要保存的日志文件
sorted_log_file_path = user_home_path + "motion_planning_log/sorted_motion_planning_node.txt"
if os.path.exists(sorted_log_file_path):
    os.remove(sorted_log_file_path)
sorted_log_file = open(sorted_log_file_path, 'w')

log = log_file.readlines()
# 用空格替换双空格
for i in range(0, len(log)):
    log[i] = log[i].replace('  ', ' ')
# 删除前三行没有用的信息
log_header = log[:4]
log = log[3:]
for log_header_info in log_header:
    sorted_log_file.write(log_header_info)
# 获取主线程号
main_thread_id = log[0].split(' ')[2]
# 是否开始将子线程输出进行排序的标志位
state_flag = False
need_sort_infos = []

for log_info in log:
    thread_id = log_info.split(' ')[2]
    if thread_id == main_thread_id and not state_flag:
        sorted_log_file.write(log_info)
    elif thread_id == main_thread_id and state_flag:
        state_flag = False
        # 先处理左转状态打印信息时间从1-3秒
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "TURNLEFT":
                if len(match_time) <= 8:
                    sorted_log_file.write(need_sort_info)
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "TURNLEFT":
                if len(match_time) > 8 and match_time[8] == '0':
                    sorted_log_file.write(need_sort_info)        
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', need_sort_info)
            if match_state == "TURNLEFT":
                print(check_info)
                print(match_time)
                if len(match_time) > 8 and match_time[8] == '1':
                    sorted_log_file.write(need_sort_info)
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "TURNLEFT":
                if len(match_time) > 8 and match_time[8] == '2':
                    sorted_log_file.write(need_sort_info)
        # 再处理直行状态打印信息时间从1-3秒
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "FORWARD":
                if len(match_time) <= 8:
                    sorted_log_file.write(need_sort_info)
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "FORWARD":
                if len(match_time) > 8 and match_time[8] == '0':
                    sorted_log_file.write(need_sort_info)        
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', need_sort_info)
            if match_state == "FORWARD":
                print(check_info)
                print(match_time)
                if len(match_time) > 8 and match_time[8] == '1':
                    sorted_log_file.write(need_sort_info)
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "FORWARD":
                if len(match_time) > 8 and match_time[8] == '2':
                    sorted_log_file.write(need_sort_info)
        # 再处理右转状态打印信息时间从1-3秒
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "TURNRIGHT":
                if len(match_time) <= 8:
                    sorted_log_file.write(need_sort_info)
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "TURNRIGHT":
                if len(match_time) > 8 and match_time[8] == '0':
                    sorted_log_file.write(need_sort_info)        
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', need_sort_info)
            if match_state == "TURNRIGHT":
                print(check_info)
                print(match_time)
                if len(match_time) > 8 and match_time[8] == '1':
                    sorted_log_file.write(need_sort_info)
        for need_sort_info in need_sort_infos:
            check_info = need_sort_info.split(']')[1]
            match_state = re.sub(u"[^\u0041-\u005a]+", "", check_info)
            match_time = re.findall('\d+', check_info)
            if match_state == "TURNRIGHT":
                if len(match_time) > 8 and match_time[8] == '2':
                    sorted_log_file.write(need_sort_info)                
        need_sort_infos = []
    else:
        state_flag = True
        need_sort_infos.append(log_info)
