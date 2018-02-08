import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree


def vis_depth(vec):
    target_size = (64,64)
    plt.imshow(vec.reshape(target_size)),plt.show()

    
def nearest_predict(vec,mat,pos):
    vec = vec.astype(int)
    mat = mat.astype(int)
    res = np.zeros(mat.shape[0])
    for i in range(mat.shape[0]):
        vec2= mat[i,:]
        dt = np.sum(abs(vec-vec2))
        res[i]=dt
    # find top answer's indices in mat
    ans = np.argsort(res)[:10]
#     for i in ans:
#         vis_depth(mat[i,:])
    return np.mean(pos[ans], axis=0), mat[ans[0]]

def draw_comparison(y_pred, y):
    for i in range(y_pred.shape[1]):
        axes = plt.gca()
        axes.set_xlabel('time index')
        axes.set_ylabel('controller velocity')
        #axes.set_ylim([-0.3,0.3])
        plt.plot(range(y_pred.shape[0]),y_pred[:,i], label = 'predicted')
        plt.plot(range(y.shape[0]),y[:,i], label = 'ground truth')
        plt.show()

class Nearest:
    def __init__(self, arr):
        self.tree=cKDTree(arr,leafsize=1)
    def findNeigh(self, dat):
        ret=self.tree.query(dat,k=1,distance_upper_bound=1.0e15)
        return ret[0],ret[1]


def exchange_hands(pos):
    return np.hstack((pos[:,-3:],pos[:,:3]))
