'''
simple_streamer.py

Sample code connect to sensor, capture 20 frames, then display the amplitude image, distance image, and point cloud.
'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import time
import argparse
import numpy as np
import pytofcore

measurement_data=[]

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
    measurement_data.append(data)
      #Distance data is in millimeters
      #Do filtering on this distance array
    
    #surf.remove()

    measurement_callback.counter += 1


parser = argparse.ArgumentParser()
parser.add_argument('--protocol-version', default=pytofcore.Sensor.DEFAULT_PROTOCOL_VERSION, type=int, help="Protocol version to use 0 or 1")
parser.add_argument('-p','--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('-m','--min_amplitude', default=0, type=int, help="minimum amplitude to display")

args = parser.parse_args()

sensor = pytofcore.Sensor(protocol_version=args.protocol_version, port_name=args.port_name)

rays = np.array(sensor.pixel_rays).reshape(
        (3, 240, 320)).transpose((1, 2, 0))
sensor.subscribe_measurement(measurement_callback)
measurement_callback.counter=0
sensor.stream_distance_amplitude()
while measurement_callback.counter < 20:
    time.sleep(0.1)
sensor.stop_stream()

plt.ion()
fig = plt.figure()
ax=[]
ax.append(fig.add_subplot(131))
ax.append(fig.add_subplot(132))
ax.append(fig.add_subplot(133, projection='3d'))
ax[2].view_init(elev=0, azim=180)
ax[0].set_title("Amplitude Image")
ax[1].set_title("Distance Image")
ax[2].set_title("Point Cloud")


for data in measurement_data:
    distance = np.array(data.distance_data)
    amplitude = np.array(data.amplitude_data)
    cloud = create_cloud_from_distance_image(distance / 1000.0, rays,False,False)
    a = ax[0].imshow(amplitude)
    d = ax[1].imshow(distance)
    x = cloud[:,:,0][amplitude>args.min_amplitude]
    y = cloud[:,:,1][amplitude>args.min_amplitude]
    z = cloud[:,:,2][amplitude>args.min_amplitude]
    amplitude = amplitude[amplitude>args.min_amplitude]
    surf = ax[2].scatter( x,y,z,c=amplitude, cmap=cm.jet,
                        linewidth=0, antialiased=False)
    fig.canvas.draw()
    fig.canvas.flush_events()
    surf.remove()
    d.remove()
    a.remove()

plt.show()








