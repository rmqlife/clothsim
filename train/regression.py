import os, cv2
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans


def same_size(X,y):
    minlength = min(y.shape[0],X.shape[0])
    print minlength
    y = y[:minlength]
    X = X[:minlength]
    return X,y

#data = np.load('cloth2/controller/data.npz')
def load_model_name(data_name):
    data = np.load(data_name)
    pos = data['pos']
    feat = data['feat']
    return load_model_data(pos,feat)
    
    
def load_model_data(pos, feat, num_samples, alpha):
    inds = np.random.choice(a=min(len(feat),len(pos)),size=int(num_samples*float(len(feat))))
    if len(inds)%2 == 1:
        inds=inds[:-1]
    nd = len(inds)/2 # number of deltas
    X_train = feat[inds[:nd]] - feat[inds[nd:]]
    y_train = pos[inds[:nd]] - pos[inds[nd:]]
    X_train, y_train = same_size(X_train, y_train)
    print X_train.shape, y_train.shape

    # begin training
    from sklearn import linear_model
    model = linear_model.Lasso(alpha = alpha)
    model.fit(X_train, y_train)
    return model

def combine_feature(data_name1, data_name2):
    data1 = np.load(data_name1)
    pos = data1['pos']
    feat1 = data1['feat']
    
    data2 = np.load(data_name2)
    feat2 = data2['feat']

    feat = np.hstack((feat1,feat2))
    return feat, pos

def dist2(p1):
    return np.sum(p1**2)**0.5

def find_goals(pos,target,n_goals):
    kmeans = KMeans(n_clusters=n_goals, random_state=0).fit(pos)
    centers = kmeans.cluster_centers_
    centers_dist = np.float16(np.ones(centers.shape[0]))
    centers_id = np.int32(np.zeros(centers.shape[0]))

    for j in range(len(centers)):
        for i in range(len(pos)):
            d = dist2(pos[i,:]-centers[j,:])
            if d<centers_dist[j]:
                centers_dist[j] = d
                centers_id[j] = i
    # sort the distance to final target
    for i in range(len(centers_id)):
        centers_dist[i] = dist2(pos[centers_id[i],:]-pos[target,:])

    # sort the centers
    centers = np.vstack((centers_dist,centers_id)).T
    centers = centers[centers[:,0].argsort()]
    centers_dist = centers[:,0]
    centers_id = np.int32(centers[:,1])
#     rgblist = get_filelist('/home/rmqlife/work/data/2017-09-09-11-06-56/rgb')
#     for i in centers_id:
#         plt.imshow(cv2.imread(rgblist[i],0),cmap='gray')
#         plt.show()
    
    return centers_id.tolist()

