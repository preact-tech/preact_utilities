# Simple Streamer
Sample python code to stream and display data from a PreAct ToF sensor.

## Prerequisites 
Install required packages
```
pip install -r requirements.txt
```

## Arguments


- '-p','--port-name'  (optional): Port to connect to device over. Default: /dev/ttyACM0

- '--protocol-version' (optional): Protocol version to connect with. Default: 1

- “-m”, “--min-amplitude” (optional): Minimum amplitude requreid for a point to be displayed.  Default: 0

## Usage

```
python3 simple_streamer.py
```
