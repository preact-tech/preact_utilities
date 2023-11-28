# Simple Streamer
Sample c++ code to stream and display data from a PreAct ToF sensor.

## Prerequisites 
Install the following libraries:

- libtofcore
- Open3d
- Opencv
- Eigen

## Building
```
mkdir build
cd build
cmake ..
make 
```

## Arguments

- '-p','--port-name'  (optional): Port to connect to device over. Default: /dev/ttyACM0

- '--protocol-version' (optional): Protocol version to connect with. Default: 1


## Usage

```
./simple_streamer
```
