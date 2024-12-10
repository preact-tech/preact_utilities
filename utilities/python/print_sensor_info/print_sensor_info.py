'''
print_sensor_info.py

Read and data from the sensor and print values.
'''


import argparse
import pytofcore


parser = argparse.ArgumentParser()
parser.add_argument('-p','--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")

args = parser.parse_args()

s = pytofcore.Sensor(port_name=args.port_name)

sensor_info = s.get_sensor_info()
print("SENSOR INFO:")
for name in sensor_info._fields:
    print(f"    {name}: {getattr(sensor_info, name)}")

vled_info = s.get_vled_setting_and_limits()
print(f"    vledSetting: {vled_info[0]}")
print(f"    vledMin: {vled_info[1]}")
print(f"    vledMax: {vled_info[2]}")
