import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

def get_filelist(home_dir, ext=['png','jpg']):
    filelist = []
    for dirpath, dirnames, filenames in os.walk(home_dir):
        for filename in filenames:
            if filename[-3:] in ext:
                fn = os.path.join(dirpath,filename)
                filelist.append(fn)
    filelist = sorted(filelist)
    # filelist = sorted(filelist,key=lambda x: int(os.path.basename(x)[:-4]))
    return filelist




def stack_vector(mat,vec):
    mat = np.vstack((mat,vec)) if mat.size else vec
    return mat