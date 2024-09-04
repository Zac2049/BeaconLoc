import argparse
import sys
sys.path.append('..')
from utils.common import process_beacon_data_for_pos

def parse_arguments():
    parser = argparse.ArgumentParser(description="Beacon Localization")
    parser.add_argument("--beacon_coords_file", type=str, default="../data0827a/beacon_coords.txt", required=True, help="Path to the file containing beacon coordinates")
    parser.add_argument("--beacon_data_file", type=str, required=True, default="../data0827a/bluetoothData(58).txt", help="Path to the file containing beacon data")
    parser.add_argument("--update_frequency", type=float, default=1.0, help="Update frequency in Hz")
    return parser.parse_args()


def main():
    args = parse_arguments()
    positions = process_beacon_data_for_pos(args)
    print(positions)

if __name__ == "__main__":
   main()
