import numpy as np
class CameraTransform:
    def __init__(self,R,t):
        # model view matrix 
        MV = np.hstack((R,t))
        self.model_view = np.vstack((MV,(0,0,0,1)))
        self.camera_position = -np.dot(R.T,t).reshape((-1))
        self.camera_orientation = np.dot(R.T,(0,0,1))
        pass
    def world_to_cam(self, point_in_world):
        p = np.hstack((point_in_world,1))
        p = np.dot(self.model_view, p)
        p = p[:3]/p[3]
        return p
    def cam_to_world(self, point_in_camera):
        p = np.hstack((point_in_camera,1))
        from numpy.linalg import inv
        p = np.dot(inv(self.model_view),p)
        p = p[:3]/p[3]
        return p
