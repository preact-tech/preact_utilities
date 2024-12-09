'''
yolo_streamer.py

Sample code connect to sensor, then display the amplitude image, distance image, and point cloud.
'''

import numpy as np
import signal
import time
import argparse
import numpy as np
import pytofcore
import open3d as o3d
import cv2
from ultralytics import YOLO, settings
from ultralytics.engine.results import Results
import torch


def load_model():
    model_load_start = time.time()
    # unload existing model from gpu memory

    print('Loading YOLOv8 Model...')
    model = YOLO('yolov8n-seg.pt')  # load a pretrained model
    if torch.cuda.is_available():            
        model.to("cuda")
        device = 'cuda'
    else:
        model.to("cpu")
        device = 'cpu'
    
    model.fuse()

    # first prediction takes a really long time, do a dummy one as part of loading model
    _ = model.predict(
            source=np.zeros((240, 320, 3), dtype=np.uint8), classes=0,  # 0 => people
            device=device,
            conf=0.6,
            verbose=False,
            imgsz=320
    )
    model_load_end = time.time()
    model_load_elapsed = model_load_end - model_load_start
    print(f"Model loaded. Elapsed time: {model_load_elapsed}")
    return [model, device]

def normalize_array(arr: np.array, use_sigmoid: bool = False, scale: float = 0.01) -> np.ndarray:
    # Preprocessing numpy array image for yolov8 model use.
    img64 = arr.astype(np.float64)
    if use_sigmoid:
        img64 = 255 * 2 * (1.0 / (1.0 + np.exp(-img64 * scale)) - 0.5)
    else:
        img64 /= img64.max()
        img64 *= 255
    img8 = img64.astype(np.uint8)
    imgbgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
    return imgbgr

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


    ampbgr = normalize_array(amplitude, True, 0.005)
                
    with torch.no_grad():
        results = model.predict(
            source=ampbgr, classes=[0],  # 0 => people  
            device=device,
            conf=0.6,
            verbose=False,
            imgsz=320
        )  # predict on an image

    obj_label = -np.ones(amplitude.shape, dtype=np.int8)
    obj_score = np.zeros(amplitude.shape, dtype=np.int8)
    if results[0].masks is not None:
        masks = results[0].masks.xy
        classes = results[0].boxes.cls.cpu().numpy()
        scores = results[0].boxes.conf.cpu().numpy()
        for msk, cls, conf in zip(masks, classes, scores):
            msk = msk.astype(np.int32)
            msk = msk.reshape((-1, 1, 2))
            _ = cv2.drawContours(obj_label, [msk], -1, int(cls), cv2.FILLED)
            _ = cv2.drawContours(obj_score, [msk], -1, int(conf * 100), cv2.FILLED)


    cv2.imshow("Distance Image",distance*5)
    cv2.imshow("Amplitude Image",results[0].plot())
    cv2.waitKey(1)
    cloud = create_cloud_from_distance_image(distance / 1000.0, rays,False,False)
    x = cloud[:,:,0][amplitude>args.min_amplitude]
    y = cloud[:,:,1][amplitude>args.min_amplitude]
    z = cloud[:,:,2][amplitude>args.min_amplitude]
    amplitude = amplitude[amplitude>args.min_amplitude]
    pcd = o3d.geometry.PointCloud()
    xyz=np.concatenate((z.reshape((-1,1)),x.reshape((-1,1)),y.reshape((-1,1))),axis=1)
    pcd.points = o3d.utility.Vector3dVector(xyz)
    # color people in point cloud blue, everything else gray
    colors = np.zeros_like(xyz)
    labels = obj_label.reshape(x.shape)
    colors[:, 0] = 0.5 # Red channel 
    colors[:, 1] = 0.5  # Green channel 
    colors[:, 2] = 0.5  # Blue channel 
    colors[labels == 0, 0] = 0
    colors[labels == 0, 1] = 0 
    colors[labels == 0, 2] = 1 

    # Add the colors to the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors)

    vis.clear_geometries()

    # add pcd to visualizer
    vis.add_geometry(pcd)
    vis.get_view_control().rotate(523.6 * 3, 523.6 * 3, 0, 0)
    vis.get_view_control().set_zoom(0.35)

    vis.poll_events()
    vis.update_renderer()

def handler(signum, frame):
    print("Stopping stream...")
    sensor.stop_stream()
    exit(1)


vis = o3d.visualization.Visualizer()
vis.create_window()

parser = argparse.ArgumentParser()
parser.add_argument('-p','--port-name', default=pytofcore.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('-m','--min_amplitude', default=0, type=int, help="minimum amplitude to display")

args = parser.parse_args()

#Load model
[model, device] = load_model()

sensor = pytofcore.Sensor(port_name=args.port_name)

rays = np.array(sensor.pixel_rays).reshape(
        (3, 240, 320)).transpose((1, 2, 0))
sensor.subscribe_measurement(measurement_callback)
sensor.set_integration_time(1000)
sensor.stream_distance_amplitude()
signal.signal(signal.SIGINT, handler)

while True:
    time.sleep(1)


