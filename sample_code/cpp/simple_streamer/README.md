# Simple Streamer
Sample c++ code to stream and display data from a PreAct ToF sensor.

## Prerequisites 
Install the following libraries:

- libtofcore
- Open3d (http://www.open3d.org/docs/release/compilation.html)
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

- '-p','--device-uri'   (optional): Port to connect to device over. Default: /dev/ttyACM0

- '--protocol-version' (optional): Protocol version to connect with. Default: 1

- '-m', '--min-amplitude' (optional): Minimum amplitude requreid for a point to be displayed.  Default: 0


## Usage

```
./simple_streamer -p /dev/ttyACM0
```
