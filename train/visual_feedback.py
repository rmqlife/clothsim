import cv2,os
import wrinkle2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

#visual feedback dictionary
class vfdic():
    debug_fag = False   
    def same_size(self,X,y):
        minlength = min(y.shape[0],X.shape[0])
        print minlength
        y = y[:minlength]
        X = X[:minlength]
        return X,y
    
    def __init__(self, tt_pos, tt_feat, num_samples=3, alpha = 1e-5):
        inds = np.random.choice(a=min(len(tt_feat),len(tt_pos)),size=int(num_samples*float(len(tt_feat))))
        if len(inds)%2 == 1:
            inds=inds[:-1]
        nd = len(inds)/2 # number of deltas
        X_train = tt_feat[inds[:nd]]
        y_train = tt_pos[inds[:nd]]
        X_train, y_train = self.same_size(X_train, y_train)
        print X_train.shape, y_train.shape

        # begin training
        from sklearn import linear_model
        self.model = linear_model.Lasso(alpha = alpha)
        self.model.fit(X_train, y_train)
    
    def feat_to_pos(self,feat):
        return self.model.predict(feat.reshape(1,-1)).ravel()
        
    def dist2(self,p1):
        return np.sum(p1**2)**0.5

if __name__=="__main__":
    data = np.load('./1110-2020/data.npz')
    tt_feat = data['tt_feat']
    tt_pos = data['tt_pos']
    tt_handles = data['tt_handles']

    test_size = 3000;
    dic = vfdic(tt_pos=tt_pos[:test_size,:], tt_feat = tt_feat[:test_size,:], num_samples = 2)
    points = np.array([])
    for ind in range(3000,4001):
        feat = tt_feat[ind,:]
        pos_true = tt_pos[ind,:]
        pos_sparse = dic.feat_to_pos(feat=feat)
        point = np.array((pos_true,pos_sparse)).ravel()
        points = np.vstack((points,point)) if points.size else point

    print points.shape
    # for each controlling parameters:
    for i in range(tt_pos.shape[1]):
        x = range(len(points))
        plt.plot(x,points[:,i])
        plt.plot(x,points[:,i+6])
        #plt.plot(x,points[:,i+12])
        plt.show()