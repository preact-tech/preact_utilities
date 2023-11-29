'''
simple_streamer.py

Sample code connect to sensor, capture 20 frames, then display the amplitude image, distance image, and point cloud.
'''

import numpy as np
import signal
import time
import argparse
import numpy as np
import pytofcore
import open3d as o3d
import cv2


def create_cloud_from_distance_image(distance: np.array, rays: np.array, horizontal_binning: bool, vertical_binning: bool):
    # map the distance image (m x n) to a point cloud (m x n x 3)
    slicing = [1, 1]
    if horizontal_binning:
        slicing[0] = 2
    if vertical_binning:
        slicing[1] = 2
    # take subset of rays based on binning configuration
    rays_small = rays[::slicing[1], ::slicing[0], :]
    cloud = distance[..., np.newaxis] * rays_small
    return cloud

def measurement_callback(data):
  if data.data_type == pytofcore.Measurement.DataType.DISTANCE_AMPLITUDE:
    #measurement_data.append(data)
    distance = np.array(data.distance_data)
    amplitude = np.array(data.amplitude_data)
    cv2.imshow("Distance Image",distance*5)
    cv2.imshow("Amplitude Image",amplitude*100)
    cv2.waitKey(1)
    cloud = create_cloud_from_distance_image(distance / 1000.0, rays,False,False)
    x = cloud[:,:,0][amplitude>args.min_amplitude]
    y = cloud[:,:,1][amplitude>args.min_amplitude]
    z = cloud[:,:,2][amplitude>args.min_amplitude]
    amplitude = amplitude[amplitude>args.min_amplitude]
    pcd = o3d.geometry.PointCloud()
    xyz=np.concatenate((z.reshape((-1,1)),x.reshape((-1,1)),y.reshape((-1,1))),axis=1)
    pcd.points = o3d.utility.Vector3dVector(xyz)
    vis.clear_geometries()

    # add pcd to visualizer
    vis.add_geometry(pcd)
    vis.get_view_control().rotate(523.6 * 3, 523.6 * 3, 0, 0)
    vis.get_view_control().set_zoom(0.35)
    vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.YCoordinate

    vis.poll_events()
    vis.update_renderer()

def handler(signum, frame):
    print("Stopping stream...")
    sensor.stop_stream()
    exit(1)


vis = o3d.visualization.Visualizer()
vis.create_window()

parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('-p','--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('-m','--min_amplitude', default=0, type=int, help="minimum amplitude to display")

args = parser.parse_args()

sensor = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

rays = np.array(sensor.pixel_rays).reshape(
        (3, 240, 320)).transpose((1, 2, 0))
sensor.subscribe_measurement(measurement_callback)
sensor.set_integration_times(4000,0,0)
sensor.stream_distance_amplitude()
signal.signal(signal.SIGINT, handler)

while True:
    time.sleep(1)


