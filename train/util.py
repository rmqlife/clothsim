import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

def get_filelist(home_dir,ext):
    filelist = []
    for dirpath, dirnames, filenames in os.walk(home_dir):
        for filename in filenames:
            if filename.endswith(ext):
                fn = os.path.join(dirpath,filename)
                filelist.append(fn)
    filelist = sorted(filelist)
    return filelist




