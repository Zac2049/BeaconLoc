import json
import os
import time
import sys
sys.path.append('..')
from src.solve_distance import BeaconLocalization
import numpy as np
from abc import ABC, abstractmethod
from filterpy.kalman import KalmanFilter
import yaml
from collections import deque

def remove_outliers(data, threshold=0.5):
    mean = np.mean(data)
    std = np.std(data)
    return [x if abs(x - mean) <= threshold * std else mean for x in data]

def remove_max_min(data):
    sorted_data = sorted(data)
    max_data = sorted_data[-2:]
    min_data = sorted_data[:2]
    mean = np.mean(sorted_data)
    return [x if x not in max_data and x not in min_data else mean for x in data]

class BeaconState:
    def __init__(self, window_size):
        self.filter = MeanFilter(window_size)
        self.latest_rssi = None
        self.latest_time = None

class MedianFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = deque(maxlen=window_size)
    
    def update(self, value):
        self.data.append(value)
        return self.get_median()
    
    def get_median(self):
        if len(self.data) == 0:
            return None
        self.data = remove_max_min(self.data)
        return np.median(self.data)

class MeanFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = deque(maxlen=window_size)
    
    def update(self, value):
        self.data.append(value)
        return self.get_mean()
    
    def get_mean(self):
        if len(self.data) == 0:
            return None
        self.data = remove_max_min(self.data)
        return np.mean(self.data)

def load_config(config_file):
    with open(config_file, 'r') as file:
        return yaml.safe_load(file)

class KalmanFilter:
    def __init__(self, config_file='../config/beacon_config.yaml'):
        config = load_config(config_file)
        self.kf = self.initialize_kalman_filter(config)
        self.last_update_time = time.time()
    
    def initialize_kalman_filter(self, config):
        kf = KalmanFilter(dim_x=1, dim_z=1)
        # kf.x = np.array([data[0]])
        kf.P *= 1000.
        kf.R = 5
        kf.Q = 0.1
        kf.H = np.array([[1.]])
        kf.F = np.array([[1.]])
        return kf
    
    def update(self, position):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        self.kf.F[0, 3] = self.kf.F[1, 4] = self.kf.F[2, 5] = dt
        self.kf.predict()
        self.kf.update(position)
        
        return self.kf.x[:3]


def read_beacon_coords(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    data = json.loads(content)
    return data

def read_beacon_data(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    data = json.loads(content)
    return data 

def load_config(config_file):
    with open(config_file, 'r') as file:
        return yaml.safe_load(file)

def process_beacon_data_for_strongest(beacon_data, laterate_num):
    '''
    Process beacon data to find the three strongest signals.
    '''
    # latest_signals = {}
    
    # for beacon_group in beacon_data:
    #     for beacon in beacon_group:
    #         if isinstance(beacon, dict) and 'uuid' in beacon and 'rssi' in beacon and 'time' in beacon:
    #             uuid = beacon['uuid']
    #             rssi = beacon['rssi']
    #             timestamp = beacon['time']
                
    #             if uuid not in latest_signals or timestamp > latest_signals[uuid]['time']:
    #                 latest_signals[uuid] = {'rssi': rssi, 'time': timestamp}
    
    # strongest_uuids = sorted(latest_signals, key=lambda x: latest_signals[x]['rssi'], reverse=True)[:3]
    # signal_strengths = {uuid: data['rssi'] for uuid, data in latest_signals.items()}
    # return strongest_uuids, signal_strengths
    window_size = 5
    state = {}
    for beacon_group in beacon_data:
        for beacon in beacon_group:
            if isinstance(beacon, dict) and 'uuid' in beacon and 'rssi' in beacon and 'time' in beacon:
                uuid = beacon['uuid']
                rssi = beacon['rssi']
                timestamp = beacon['time']
                
                if uuid not in state:
                    state[uuid] = BeaconState(window_size)
                
                median_rssi = state[uuid].filter.update(rssi)
                
                if median_rssi is not None:
                    if state[uuid].latest_time is None or timestamp > state[uuid].latest_time:
                        state[uuid].latest_rssi = median_rssi
                        state[uuid].latest_time = timestamp
    
    latest_signals = {uuid: {'rssi': state[uuid].latest_rssi, 'time': state[uuid].latest_time} for uuid in state if state[uuid].latest_rssi is not None}
    strongest_uuids = sorted(latest_signals, key=lambda x: latest_signals[x]['rssi'], reverse=True)[:laterate_num]
    signal_strengths = {uuid: data['rssi'] for uuid, data in latest_signals.items()}
    
    return strongest_uuids, signal_strengths



def process_beacon_data_for_pos(beacon_coords_file, beacon_data_file, config_file, update_frequency=10, laterate_num=3):
    '''
    Main function to process beacon data and determine position
    '''
    args = argparse.Namespace(beacon_coords_file=beacon_coords_file, beacon_data_file=beacon_data_file, config_file=config_file, update_frequency=update_frequency, laterate_num=laterate_num)
    print(args.beacon_coords_file) 
    if not os.path.exists(args.beacon_coords_file):
        print(f"Error: Beacon coordinates file not found: {args.beacon_coords_file}")
        return
    
    if not os.path.exists(args.beacon_data_file):
        print(f"Error: Beacon data file not found: {args.beacon_data_file}")
        return

    beacon_data = read_beacon_data(args.beacon_data_file)
    update_interval = 1.0 / args.update_frequency
    
    config = load_config(args.config_file)

    # print("beacon_data",beacon_data)
    localization = BeaconLocalization(args)

    positions = []
    
    for beacon_group in beacon_data:
        strongest_uuids, signal_strengths = process_beacon_data_for_strongest([beacon_group], laterate_num)
        if len(strongest_uuids) < 3:
            print("Error: At least 3 beacons are required to determine position")
            positions.append(None)
            continue
        position = localization.update_position(strongest_uuids, signal_strengths)
        positions.append(position)
        if position is not None:
            try:
                print(f"Current position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")
            except:
                print("position",position)
        else:
            print("Unable to determine position")

        time.sleep(update_interval)
    
    return positions

        
def aggregate_continuous_points(coordinates, n):
    """
    对每n个连续的坐标点进行聚合降频，并剔除离群点后得到统计代表点。
    
    coordinates: List of tuples/lists where each element is a (x, y) coordinate.
    n: Number of continuous points to group together.
    
    Returns:
    List of aggregated points (x, y).
    """
    aggregated_points = []

    for i in range(0, len(coordinates), n):
        # 提取每组n个连续的点
        group = [coord for coord in coordinates[i:i+n] if coord is not None]
        group = np.array(group)
        # 计算中位数和标准差
        median_x = np.median(group[:, 0])
        median_y = np.median(group[:, 1])
        std_x = np.std(group[:, 0])
        std_y = np.std(group[:, 1])

        # 设定阈值来剔除离群点
        threshold = 0.5
        filtered_group = []
        for coord in group:
            if (abs(coord[0] - median_x) < threshold * std_x) and (abs(coord[1] - median_y) < threshold * std_y):
                filtered_group.append(coord)

        filtered_group = np.array(filtered_group)

        # 如果过滤后有点，计算代表点
        if len(filtered_group) > 0:
            representative_point = np.mean(filtered_group, axis=0)
        else:
            # 如果所有点都被剔除了，可以使用中位数或者其他方式处理
            representative_point = np.mean(group, axis=0)
        
        aggregated_points.append(representative_point)

    return aggregated_points

def aggregate_continuous_points_sliding(coordinates, n):
    """
    对每个坐标点，使用前后n-1个点的窗口进行滤波，并剔除离群点后得到统计代表点。
    
    coordinates: List of tuples/lists where each element is a (x, y) coordinate.
    n: Window size, the number of points to consider around each point.
    
    Returns:
    List of aggregated points (x, y) with the same length as input coordinates.
    """
    aggregated_points = []
    half_window = n // 2

    for i in range(len(coordinates)):
        # 提取当前点及其前后n-1个点组成的窗口
        start_idx = max(0, i - half_window)
        end_idx = min(len(coordinates), i + half_window + 1)
        window = [coord for coord in coordinates[start_idx:end_idx] if coord is not None]
        window = np.array(window)

        # 计算中位数和标准差
        median_x = np.median(window[:, 0])
        median_y = np.median(window[:, 1])
        std_x = np.std(window[:, 0])
        std_y = np.std(window[:, 1])

        # 设定阈值来剔除离群点
        threshold = 0.2
        filtered_window = []
        for coord in window:
            if (abs(coord[0] - median_x) < threshold * std_x) and (abs(coord[1] - median_y) < threshold * std_y):
                filtered_window.append(coord)

        filtered_window = np.array(filtered_window)

        # 如果过滤后有点，计算代表点
        if len(filtered_window) > 0:
            representative_point = np.mean(filtered_window, axis=0)
        else:
            # 如果所有点都被剔除了，可以使用中位数或者其他方式处理
            representative_point = np.mean(window, axis=0)

        aggregated_points.append(representative_point)

    return aggregated_points
