# HDF5 Parser
Sample python code to read and display data from a PreAct Viewer HDF5 recording file.

## Prerequisites 
Install required packages
```
pip install -r requirements.txt
```

## Arguments

- “-p”, “--path”  (required): Path to hdf5 file captured with the preact viewer.

- “-m”, “--min-amplitude” (optional): Minimum amplitude requreid for a point to be displayed.  Default: 0

## Usage

```
python3 visualize_h5.py -p /home/user/PreAct/Viewer_recording_20231023_115012.hdf5 -m 50
```
