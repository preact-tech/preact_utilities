'''
print_sensor_info.py

Read and data from the sensor and print values.
'''


import argparse
import pytofcore


parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('-p','--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")

args = parser.parse_args()

s = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

sensor_info = s.get_sensor_info()
print("SENSOR INFO:")
for name in sensor_info._fields:
    print(f"{name}: {getattr(sensor_info, name)}")


