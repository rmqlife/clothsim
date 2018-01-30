from expert2 import *
import wrinkle2
import util
import cv2
import matplotlib.pyplot as plt

class Dagger:
    frame_id = 0
    def __init__(self):
        import time
        path = "./"+time.strftime("%m%d-%H%M")
        if os.path.exists(path):
            shutil.rmtree(path)
        os.mkdir(path)

        self.tt_expert_pos = np.array([])
        self.tt_pred_pos = np.array([])
        self.tt_handles = np.array([])
        self.tt_feat = np.array([])
        self.tt_log = np.array([])
        self.path = path
        pass

    def update(self,handles, pred_pos, expert_pos, feat, this_log):
        handles = np.array(handles).ravel()
        expert_pos = np.array(expert_pos).ravel()
        pred_pos = np.array(pred_pos).ravel()
        

        self.tt_pred_pos = np.vstack((self.tt_pred_pos,pred_pos)) if self.tt_pred_pos.size else pred_pos
        self.tt_feat = np.vstack((self.tt_feat,feat)) if self.tt_feat.size else feat
        self.tt_handles = np.vstack((self.tt_handles, handles)) if self.tt_handles.size else handles
        self.tt_expert_pos = np.vstack((self.tt_expert_pos, expert_pos)) if self.tt_expert_pos.size else expert_pos
        self.tt_log = np.vstack((self.tt_log, this_log)) if self.tt_log.size else this_log
        
        np.savez(self.path+'/data',tt_expert_pos = self.tt_expert_pos,
                 tt_pred_pos = self.tt_pred_pos, tt_feat = self.tt_feat, 
                 tt_handles = self.tt_handles, tt_log = self.tt_log)
        pass
    
    def frame_path(self):
        self.frame_id = self.frame_id + 1
        return self.path+"/%04i"%(self.frame_id-1)
    
    def saved_frame_path(self):
        return self.path+"/%04i"%(self.frame_id-1)+'.png'
    
