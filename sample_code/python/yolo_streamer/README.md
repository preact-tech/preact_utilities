# Yolo Streamer
Sample python code to stream and display data from a PreAct ToF sensor.  Yolov8 segmentaiton model will be run on the amplitude image to detect people.  People will be colored blue in amplitude image and point cloud.

## Prerequisites 
Install required packages
```
pip install -r requirements.txt
```

## Arguments


- '-p','--port-name'  (optional): Port to connect to device over. Default: /dev/ttyACM0

- “-m”, “--min-amplitude” (optional): Minimum amplitude requreid for a point to be displayed.  Default: 0

## Usage

```
python3 yolo_streamer.py
```
