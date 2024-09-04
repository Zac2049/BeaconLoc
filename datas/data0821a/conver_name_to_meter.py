path = "/data/data870/lph/beaconloc/data0821a"

# name(5) to 1.2m
import os


for file in os.listdir(path):
    if file.endswith(".txt"):
        file_name_index = file.split("(")[1][0]
        file_name_index = int(file_name_index)
        meter_name = f"bluetoothData_{(file_name_index-5)*0.4+1.2}m.txt"
        os.rename(os.path.join(path, file), os.path.join(path, meter_name))
