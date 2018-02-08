# add background
from merge_dataset import *
from util import *
import matplotlib.pyplot as plt
from wrinkle2 import *

def add_bg(im,bg):
    mask = (im==0).astype('bool')
    #res = cv2.bitwise_and(bg,bg,mask=mask)
    return bg*mask+im

def add_bg_folder(home,home2,bg_path):
    tt_depth = np.array([])
    if not os.path.exists(home2):
        os.mkdir(home2)
    bg = cv2.imread(bg_path, cv2.IMREAD_ANYDEPTH)
    pics = get_filelist(home,['png'])
    for i in range(0,len(pics)):
        d = cv2.imread(pics[i],cv2.IMREAD_ANYDEPTH)
        d = add_bg(d,bg)
        new_path = os.path.join(home2,pics[i].split(os.sep)[-1])
        cv2.imwrite(new_path, d)
        print(new_path)


if __name__=='__main__':
    import sys
    add_bg_folder(sys.argv[1], sys.argv[2], sys.argv[3])

