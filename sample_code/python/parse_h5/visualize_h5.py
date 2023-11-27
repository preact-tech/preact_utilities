import os, sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import cv2
from matplotlib import cm
import h5py
import argparse
import numpy as np
import math as m
import time    



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

def main():
   
    parser = argparse.ArgumentParser()
    parser.add_argument('-p','--path', default="/tmp", type=str, help="Path to h5 file", required=True)
    parser.add_argument('-m','--min_amplitude', default=0, type=int, help="minimum amplitude to display")
    args = parser.parse_args()
    plt.ion()
    fig, ax = plt.subplots(1,1,subplot_kw={"projection": "3d"})
    ax.view_init(elev=0, azim=0)


    h5=h5py.File(args.path, "r")
    rays=h5['recording_information']['rays'][()]

    for key in h5['frames'].keys():
        print("showing frame: ",key)
        distance=h5['frames'][key]['Distance'][()]
        amplitude=h5['frames'][key]['Amplitude'][()]

        cloud=create_cloud_from_distance_image(distance, rays,False,False)
        x = cloud[:,:,0][amplitude>args.min_amplitude]
        y = cloud[:,:,1][amplitude>args.min_amplitude]
        z = cloud[:,:,2][amplitude>args.min_amplitude]
        amplitude = amplitude[amplitude>args.min_amplitude]



        surf = ax.scatter( x,y,z,c=amplitude, cmap=cm.jet,
                            linewidth=0, antialiased=False)
        fig.canvas.draw()
        fig.canvas.flush_events()
 
        time.sleep(0.1)

    plt.show()

    h5.close()


# Driver Code
if __name__ == '__main__':
     
    # Calling main() function
    main()


