from util import *
import sys
import shutil
if __name__=="__main__":
    src_list = get_filelist(sys.argv[1],ext=["out"])
    dst_dir = "./out"
    if not os.path.exists(dst_dir):
        os.mkdir(dst_dir)
    for src in src_list:
        dst_name = "-".join(src.split(os.sep)[-3:-1])+".out"
        dst_name = os.path.join(dst_dir,dst_name)
        print(dst_name)
        shutil.copyfile(src,dst_name)
