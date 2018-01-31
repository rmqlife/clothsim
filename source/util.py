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


def flt2img(name, depth_scale = 0.000124986647279):

    f=open(name,'rb')
    width=struct.unpack('<I',f.read(4))[0]
    height=struct.unpack('<I',f.read(4))[0]
    import numpy as np
    d = np.zeros([width*height])
    bg = 0
    for i in range(width*height):
        val = struct.unpack('d',f.read(8))[0]
        bg = max(bg,val)
        d[i]=val
        pass
        #print(struct.unpack('d',f.read(8))[0])
    for i in range(width*height):
        if d[i]>=bg:
            d[i]=0
            
    d=d/depth_scale
    d=d.reshape((height,width))
    d=d.astype('uint16')
    d=np.flip(d,0)
    return d


import struct
import cv2
import matplotlib.pyplot as plt

from util import *

def flt2img(name, depth_scale = 0.000124986647279):
    f=open(name,'rb')
    width=struct.unpack('<I',f.read(4))[0]
    height=struct.unpack('<I',f.read(4))[0]
    import numpy as np
    d = np.zeros([width*height])
    bg = 0
    for i in range(width*height):
        val = struct.unpack('d',f.read(8))[0]
        bg = max(bg,val)
        d[i]=val
        pass
        #print(struct.unpack('d',f.read(8))[0])
    for i in range(width*height):
        if d[i]>=bg:
            d[i]=0
            
    d=d/depth_scale
    d=d.reshape((height,width))
    d=d.astype('uint16')
    d=np.flip(d,0)
    return d