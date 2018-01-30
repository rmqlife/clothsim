import time
from skimage.feature import hog
from skimage import data, color, exposure
import os, cv2
import matplotlib.pyplot as plt
import numpy as np

def gabor_feat(gray, num_theta = 4, grid = 80):
    ks = 101 # kernel size
    avg = np.zeros(gray.shape)
    count = 0
    rows, cols = gray.shape
    hist = []
    for i in range(num_theta):
        g_kernel1 = cv2.getGaborKernel((ks, ks), sigma = 6.0, theta = i*np.pi/num_theta, lambd = 10.0, gamma=0.5, psi=0, ktype=cv2.CV_32F)
        filtered = cv2.filter2D(gray, cv2.CV_8UC3, g_kernel1)
        count = count+1
        avg = avg + filtered
        count = 0
        for col in np.arange(0, cols, grid):
            for row in np.arange(0, rows, grid):
                block = filtered[row:row+grid,col:col+grid]
                val =  float(np.sum(block))
                hist.append(val)
    avg = avg/float(num_theta)
    avg = np.uint8(avg)
    return avg, hist

def hoghist(gray):
    hist = hog(gray, orientations=8, pixels_per_cell=(80,   80), cells_per_block=(1,1))
    return hist

def grayhist(gray):
    hist = cv2.calcHist([gray],[0],None,[256],[0,256])
    hist = np.reshape(hist,(-1,1))
    hist = hist.ravel()
    return hist

def howhist(gray):
    gabor, hist = gabor_feat(gray, num_theta = 4)
    return np.array(hist)


def xhist(im):
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    #start_time = time.time()
    i_gray = grayhist(gray)
    i_gray = i_gray/sum(i_gray)
    #print sum(i_gray), start_time - time.time()
    
    start_time = time.time()
    i_how = howhist(gray)
    i_how = i_how/sum(i_how)
    #print sum(i_how), start_time -  time.time()
    
    start_time = time.time()
    i_hog = hoghist(gray)
    i_hog = i_hog/sum(i_hog)
    #print sum(i_hog), start_time - time.time()
    
    i_feat = i_hog
    i_feat = np.hstack((i_hog,i_gray,i_how))
    return i_feat