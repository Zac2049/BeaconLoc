import numpy as np
from scipy.optimize import minimize
from filterpy.kalman import KalmanFilter
import time
import yaml
from abc import ABC, abstractmethod
import pdb

class PositionFilter(ABC):
    @abstractmethod
    def update(self, position):
        pass

class KalmanPositionFilter(PositionFilter):
    def __init__(self, config):
        self.kf = self.initialize_kalman_filter(config)
        self.last_update_time = time.time()

    def initialize_kalman_filter(self, config):
        kf_config = config['kalman_filter']
        kf = KalmanFilter(dim_x=kf_config['dim_x'], dim_z=kf_config['dim_z'])
        kf.F = np.array([[1, 0, 0, 0.1, 0, 0],
                         [0, 1, 0, 0, 0.1, 0],
                         [0, 0, 1, 0, 0, 0.1],
                         [0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 1]])
        kf.H = np.array([[1, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0]])
        kf.R *= kf_config['measurement_noise']
        kf.Q *= kf_config['process_noise']
        return kf

    def update(self, position):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        self.kf.F[0, 3] = self.kf.F[1, 4] = self.kf.F[2, 5] = dt
        self.kf.predict()
        self.kf.update(position)

        return self.kf.x[:3]

class MedianPositionFilter(PositionFilter):
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.positions = []

    def update(self, position):
        self.positions.append(position)
        if len(self.positions) > self.window_size:
            self.positions.pop(0)
        
        return np.median(self.positions, axis=0)

class AveragePositionFilter(PositionFilter):
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.positions = []

    def update(self, position):
        self.positions.append(position)
        if len(self.positions) > self.window_size:
            self.positions.pop(0)
        
        return np.mean(self.positions, axis=0)

class BeaconLocalization:
    def __init__(self, beacon_coords, config_file='../config/beacon_config.yaml'):
        self.beacon_coords = beacon_coords
        self.config = self.load_config(config_file)
        self.default_beacon_height = self.config['beacon_localization']['default_beacon_height']
        self.default_human_height = self.config['beacon_localization']['default_human_height']
        self.filter = self.initialize_filter()

    def load_config(self, config_file):
        with open(config_file, 'r') as file:
            return yaml.safe_load(file)

    def initialize_filter(self):
        filter_type = self.config['beacon_localization']['filter_type']
        if filter_type == 'kalman':
            return KalmanPositionFilter(self.config['beacon_localization'])
        elif filter_type == 'median':
            return MedianPositionFilter(self.config['beacon_localization']['median_window_size'])
        elif filter_type == 'average':
            return AveragePositionFilter(self.config['beacon_localization']['average_window_size'])
        else:
            raise ValueError(f"Unsupported filter type: {filter_type}")

    def match_beacons(self, strongest_uuids, signal_strengths):
        matched_beacons = []
        for uuid in strongest_uuids:
            if uuid in self.beacon_coords:
                coord_dict = self.beacon_coords[uuid]
                x, y, z = coord_dict['x'], coord_dict['y'], coord_dict['z']
                # z = self.default_beacon_height
                distance = self.rssi_to_distance(signal_strengths[uuid])
                matched_beacons.append((x, y, z, distance))
        return matched_beacons

    def rssi_to_distance(self, rssi):
        rssi_config = self.config['beacon_localization']['rssi_to_distance']
        txPower = rssi_config['tx_power']
        n = rssi_config['path_loss_exponent']
        return 10 ** ((txPower - rssi) / (10 * n))

    def trilaterate(self, beacons):
        def error(point, beacons):
            return sum((np.linalg.norm(np.array(point) - np.array(b[:3])) - b[3])**2 for b in beacons)
        initial_guess = np.mean(beacons, axis=0)[:3]
        initial_guess[2] = self.default_beacon_height
        bounds = [(None, None), (None, None), (0.9, 1.1)]
        result = minimize(error, initial_guess, args=(beacons,), method=self.config['trilateration']['method'], bounds=bounds)
        # result = minimize(error, initial_guess, args=(beacons,), method=self.config['trilateration']['method'])
        return result.x

    def update_position(self, strongest_uuids, signal_strengths):
        matched_beacons = self.match_beacons(strongest_uuids, signal_strengths)
        if len(matched_beacons) < self.config['position_update']['min_beacons']:
            return None  # Not enough beacons for trilateration

        estimated_position = self.trilaterate(matched_beacons)

        # Apply filter
        filtered_position = self.filter.update(estimated_position)

        return filtered_position

def solve_distance(beacon_coords, strongest_uuids, signal_strengths):
    """
    Solve the distance and determine the position based on beacon data.

    Parameters:
    beacon_coords (dict): A dictionary mapping beacon UUIDs to their (x, y) coordinates.
    strongest_uuids (list): A list of UUIDs of the strongest detected beacons.
    signal_strengths (dict): A dictionary mapping beacon UUIDs to their signal strengths (RSSI values).

    Returns:
    tuple or str: A tuple (x, y, z) representing the estimated position, or a string message if position cannot be determined.
    """
    localization = BeaconLocalization(beacon_coords)
    position = localization.update_position(strongest_uuids, signal_strengths)
    return position if position is not None else "Unable to determine position"