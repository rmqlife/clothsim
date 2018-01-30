import cv2
import wrinkle2
import numpy as np
import matplotlib.pyplot as plt

class random_trees():
    debug_fag = False    
    def __init__(self,tt_handles,tt_pos,tt_feat,num_class=100, num_trees=1):
        self.tt_handles = tt_handles
        self.tt_pos = tt_pos
        self.tt_feat = tt_feat
        # build the initial classifier
        # cluster the pos position:
        # give labels
        from sklearn.cluster import KMeans


        kmeans = KMeans(n_clusters=num_class, random_state=0).fit(tt_handles)
        labels = kmeans.labels_

        from sklearn.ensemble import RandomForestClassifier
        # using labels to construct the random trees
        self.clf = RandomForestClassifier(n_estimators=num_trees)
        # n_estimators : integer, optional (default=10) The number of trees in the forest.
        self.clf.fit(X=tt_feat, y=labels)

        # Apply trees in the forest to X, return leaf indices in each estimator(decision tree)
        self.tt_leaf = self.clf.apply(X=tt_feat)
        self.build_leaf_index()
        
        # construct the (HH)Hxa=a* 
    def build_leaf_index(self):
        tree = self.tt_leaf
        leaf_num = np.zeros(tree.shape[1])
        for i in range(tree.shape[1]):
            leaf_num[i] = len(np.unique(tree[:,i]))
        leaf_amount = 0
        leaf_id = np.zeros((tree.shape[1], int(np.max(tree)+1)))
        actual_id = np.array([])
        # build the map from tree_leaves to a 1d leaf
        for i in range(tree.shape[1]):
            leaves = np.unique(tree[:,i])
            for j in leaves:
                leaf_id[i][j]=leaf_amount
                leaf_amount = leaf_amount + 1
                this_id = np.array([i,j])
                actual_id = np.vstack((actual_id,this_id)) if actual_id.size else this_id

        self.actual_id = actual_id.astype(int)
        self.leaf_id = leaf_id.astype(int)
        
        print "actural leaf amount", np.sum(leaf_num)
        print "leaf_amount",leaf_amount
        print "actual_id length", len(self.actual_id)
        print "max leaf_id", np.max(self.leaf_id)
        self.get_leaf_action()
        return

    def leaf_to_index(self, tree, leaf):
        return self.leaf_id[tree][leaf]
    def index_to_leaf(self, ind):
        return self.actual_id[ind]
    
    
    def get_leaf_action(self):
        # using the self.tt_feat self.tt_pos
        # self.tt_tree
        # map the actions of training samples on each leaf of the tree
        tree = self.tt_leaf
        H = np.zeros((len(self.tt_pos),len(self.actual_id)))
        for i in range(tree.shape[0]):
            for j in range(tree.shape[1]):
                # j is the tree id
                # i is the sample id
                ind = self.leaf_to_index(j,tree[i][j])
                H[i,ind]=1
        # normalize
        # H = 1.0/tree.shape[1]*H
        print "H shape", H.shape
        
        A = np.dot(H.T,H)
        A = A + np.diag(np.ones(A.shape[0]))*1e-13
        invA = np.linalg.inv(A)
        B = np.dot(invA, H.T)
        act_set = np.dot(B,self.tt_pos)
        # print act_set.shape
        self.leaf_act = np.zeros((self.leaf_id.shape[0], self.leaf_id.shape[1], self.tt_pos.shape[1]))
        for i in range(len(act_set)):
            # i is an leaf ind
            tree_id, leaf_id = self.index_to_leaf(i)
            self.leaf_act[tree_id,leaf_id,:]=act_set[i,:]
        return
    
    def feat_to_act(self,feat):
        leaf = self.feat_to_leaf(feat)
        tt_act = np.zeros(self.leaf_act.shape[2])
        for i in range(len(leaf)):
            act = self.leaf_act[i,leaf[i]]
            tt_act = tt_act + act            
        return tt_act.ravel()
    
    def feat_to_leaf(self,feat):
        leaf = self.clf.apply(X=feat.reshape(1,-1))
        return leaf.ravel()
    
    def leaf_to_pos(self,leaf):
        pos_sum = np.zeros(6)
        pos_num = 0
        for i in range(len(leaf)):
            # in i-th tree
            leaf_set = self.tt_leaf[:,i]
            # indices that in the same leaf as the sample
            leaf_loc = np.where(leaf_set==leaf[i])
            # average all the sums
            pos_sum = pos_sum + np.sum(self.tt_pos[leaf_loc,:],axis=1).ravel()
            pos_num = pos_num + len(leaf_loc[0])
        return pos_sum/pos_num
    
    def feat_to_pos(self,feat):
        leaf = self.feat_to_leaf(feat)
        return self.leaf_to_pos(leaf)
