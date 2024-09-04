from flask import Flask, jsonify, request
from utils.common import read_beacon_coords, process_beacon_data_for_strongthree
from src.solve_distance import BeaconLocalization

app = Flask(__name__)

# Global variables
beacon_coords = None
localization = None

def init_app(beacon_coords_file):
    global beacon_coords, localization
    beacon_coords = read_beacon_coords(beacon_coords_file)
    localization = BeaconLocalization(beacon_coords)

# state = BeaconState(window_size=5)
@app.route('/upload_distance', methods=['POST'])
def real_time_processing():
    data = request.json
    if isinstance(data, list):
        # Handle multiple beacon groups
        positions = []
        for beacon_group in data:
            strongest_uuids, signal_strengths = process_beacon_data_for_strongthree([beacon_group])
            position = localization.update_position(strongest_uuids, signal_strengths)
            positions.append(position)
        return jsonify({'positions': positions})
    else:
        # Handle single beacon group
        strongest_uuids, signal_strengths = process_beacon_data_for_strongthree([data])
        position = localization.update_position(strongest_uuids, signal_strengths)
        return jsonify({'position': position})

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Beacon Localization Server")
    parser.add_argument("--beacon_coords_file", type=str, default="data/beacon_coords.txt", help="Path to the file containing beacon coordinates")
    parser.add_argument("--port", type=int, default=5000, help="Port to run the server on")
    args = parser.parse_args()

    init_app(args.beacon_coords_file)
    app.run(debug=True, host='0.0.0.0', port=args.port)