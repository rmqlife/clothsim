import os,sys,shutil,random
from subprocess import call
import ctypes as ct
import numpy as np
from vapory import (Scene,POVRayElement,Camera,Background,
                    LightSource,Texture,Pigment,Finish,format_if_necessary)
from vtk import (vtkPolyData, vtkPoints, vtkTriangle, vtkCellArray, vtkPolyDataMapper, vtkActor,
                 vtkRenderer, vtkRenderWindow, vtkCamera,
                 vtkWindowToImageFilter, vtkBMPWriter,
                 vtkImageShiftScale)

class ClothMesh(POVRayElement):
    def __init__(self,vss,iss,nss,*args):
        self.vss=vss
        self.iss=iss
        self.nss=nss
        self.args=list(args)
    def __str__(self):
        nrV = len(self.vss)
        nrI = len(self.iss)
        ret="mesh2{\n"
        ret=ret+"vertex_vectors{%d,\n"%nrV
        for i in xrange(nrV):
            ret=ret+("<%f,%f,%f>%s")%(self.vss[i][0],self.vss[i][1],self.vss[i][2],("," if i<nrV-1 else ""))
        ret=ret+"}\n"
        if len(self.nss) == nrV:
            ret=ret+"normal_vectors{%d,\n"%nrV
            for i in xrange(nrV):
                ret=ret+("<%f,%f,%f>%s")%(self.nss[i][0],self.nss[i][1],self.nss[i][2],("," if i<nrV-1 else ""))
            ret=ret+"}\n"
        ret=ret+"face_indices{%d,\n"%nrI
        for i in xrange(nrI):
            ret=ret+("<%d,%d,%d>%s")%(self.iss[i][0],self.iss[i][1],self.iss[i][2],("," if i<nrI-1 else ""))
        ret=ret+"}\n"
        ret_additional="".join([str(format_if_necessary(e)) for e in self.args])
        ret=ret+ret_additional
        ret=ret+"}"
        return ret

class arcsim_expert:
    def __init__(self):
        #load lib
        self.expert = ct.cdll.LoadLibrary("./libarcsim_expert.so")
        #register simulation functions
        self.expert.create_sheet_example_c.argtypes =   \
            [ct.c_double,ct.c_int,ct.c_double,ct.c_int,ct.c_char_p,
             ct.POINTER(ct.c_double),ct.POINTER(ct.c_double),
             ct.POINTER(ct.c_double),ct.POINTER(ct.c_double),
             ct.POINTER(ct.c_double),ct.c_int,
             ct.c_bool,ct.c_bool]
        self.expert.setup_begin_c.argtypes = [ct.c_char_p]
        self.expert.setup_end_c.argtypes = []
        self.expert.set_obs_motion_c.argtypes = [ct.c_int,ct.POINTER(ct.c_double),ct.c_int]
        self.expert.set_handle_c.argtypes = [ct.POINTER(ct.c_double),ct.c_int]
        self.expert.save_frame_vtk_c.argtypes = [ct.c_char_p,ct.c_int]
        self.expert.get_meshv_c.argtypes = [ct.POINTER(ct.c_int),ct.c_int]
        self.expert.get_meshv_c.restype = ct.POINTER(ct.c_double)
        self.expert.get_meshn_c.argtypes = [ct.POINTER(ct.c_int),ct.c_int]
        self.expert.get_meshn_c.restype = ct.POINTER(ct.c_double)
        self.expert.get_meshi_c.argtypes = [ct.POINTER(ct.c_int),ct.c_int]
        self.expert.get_meshi_c.restype = ct.POINTER(ct.c_int)
        self.expert.advance_c.argtypes = []
        self.expert.illustrate_c.argtypes = [ct.c_char_p,ct.c_int]
        #genDepth(GLdouble eye[3],GLdouble ctr[3],GLdouble up[3],
        #         GLdouble fovy,GLdouble aspect,GLdouble zNear,GLdouble zFar,
        #         int w,int h,const char* path,const char* pathDepth)
        self.expert.gen_depth_c.argtypes = [ct.POINTER(ct.c_double),ct.POINTER(ct.c_double),ct.POINTER(ct.c_double),ct.c_double,ct.c_double,
                                            ct.c_int,ct.c_int,ct.c_char_p,ct.c_char_p]
        #register expert functions
        self.expert.expert_flat_close_c.argtypes = [ct.POINTER(ct.c_double),ct.c_double,ct.c_double]
        self.expert.expert_flat_close_c.restype = ct.POINTER(ct.c_double)
        self.expert.expert_arc_close_c.argtypes = [ct.POINTER(ct.c_double),ct.c_double,ct.c_double]
        self.expert.expert_arc_close_c.restype = ct.POINTER(ct.c_double)
        self.expert.expert_twist_close_c.argtypes = [ct.POINTER(ct.c_double),ct.c_double,ct.c_double]
        self.expert.expert_twist_close_c.restype = ct.POINTER(ct.c_double)
        self.expert.expert_flat_c.argtypes = [ct.POINTER(ct.c_double),ct.c_double,ct.c_double]
        self.expert.expert_flat_c.restype = ct.POINTER(ct.c_double)
        self.expert.expert_arc_c.argtypes = [ct.POINTER(ct.c_double),ct.c_double,ct.c_double]
        self.expert.expert_arc_c.restype = ct.POINTER(ct.c_double)
        self.expert.expert_twist_c.argtypes = [ct.POINTER(ct.c_double),ct.c_double,ct.c_double]
        self.expert.expert_twist_c.restype = ct.POINTER(ct.c_double)
        self.expert.apply_expert_c.argtypes = [ct.POINTER(ct.c_double),ct.POINTER(ct.c_double),ct.c_double]
        self.expert.apply_hand_c.argtypes = [ct.POINTER(ct.c_double),ct.POINTER(ct.c_double),ct.POINTER(ct.c_double),ct.c_double]
        self.expert.free_vec_c.argtypes = [ct.POINTER(ct.c_double)]
        self.expert.free_veci_c.argtypes = [ct.POINTER(ct.c_int)]
        #image output size
        self.w=1024
        self.h=768
        self.fovy=90
        self.zNear=0.1
        self.zFar=5
        #background color
        self.bk_r=1
        self.bk_g=1
        self.bk_b=1
        #cloth color
        self.cc_r=0.7
        self.cc_g=0.7
        self.cc_b=0.7
        self.specular=0.4
        #cloth color
        self.cco_r=0.7
        self.cco_g=0.2
        self.cco_b=0.2
        self.specularo=0.4
        #camera pos
        self.cx=2.0
        self.cy=2.0
        self.cz=2.0
        #focal point
        self.fx=0.5
        self.fy=1.0
        self.fz=0.0
        #view up
        self.ux=0.0
        self.uy=0.0
        self.uz=1.0
        #lights
        self.lights=[]
        self.lights=self.lights+[LightSource([1,0,2],'color',[1,1,1])]
    #simulation functionality
    def create_sheet(self, x, resX, y, resY,    \
                     path, g, w, wden, wdrag, 
                     obs, nrObs,
                     oneside,remesh):
        g_c =(ct.c_double * 3)(*g)
        w_c =(ct.c_double * 3)(*w)
        wDen_c =(ct.c_double)(*[wden])
        wDrag_c =(ct.c_double)(*[wdrag])
        obs_c =(ct.c_double * len(obs))(*obs)
        self.expert.create_sheet_example_c(x,resX,y,resY,path.encode(),  \
                                           g_c,w_c,wDen_c,wDrag_c,  \
                                           obs_c, nrObs,  \
                                           oneside,remesh)
    def setup(self, path, motions):
        self.expert.setup_begin_c(path.encode())
        for i in xrange(0,self.expert.nr_obstacles_c()):
            motion_c =(ct.c_double * len(motions[i]))(*(motions[i]))
            self.expert.set_obs_motion_c(i,motion_c,len(motions[i])//7)
        self.expert.setup_end_c()
    def set_handle(self, handles):
        ptr = []
        for i in xrange(len(handles)):
            ptr = ptr+[handles[i][0],handles[i][1],handles[i][2]]
        ptr_c =(ct.c_double*len(ptr))(*ptr)
        self.expert.set_handle_c(ptr_c,len(handles))
    def save_frame_mesh(self, idx):
        #read
        nrv_c=(ct.c_int)()
        nrn_c=(ct.c_int)()
        nri_c=(ct.c_int)()
        vss_c = self.expert.get_meshv_c(nrv_c,idx)
        nss_c = self.expert.get_meshn_c(nrn_c,idx)
        iss_c = self.expert.get_meshi_c(nri_c,idx)
        vss = [[0,0,0]]*nrv_c.value
        nss = [[0,0,0]]*nrn_c.value
        iss = [[0,0,0]]*nri_c.value
        assert nrv_c.value == nrn_c.value
        for i in xrange(nrv_c.value):
            vss[i]=[vss_c[i*3+0],vss_c[i*3+1],vss_c[i*3+2]]
            nss[i]=[nss_c[i*3+0],nss_c[i*3+1],nss_c[i*3+2]]
        for i in xrange(nri_c.value):
            iss[i]=[iss_c[i*3+0],iss_c[i*3+1],iss_c[i*3+2]]
        self.expert.free_vec_c(vss_c)
        self.expert.free_vec_c(nss_c)
        self.expert.free_veci_c(iss_c)
        return vss, nss, iss
    def save_frame_vtk(self, path, f):
        self.expert.save_frame_vtk_c(path.encode(),f)
    def save_frame_image(self, path):
        meshes=[Background("color",[self.bk_r,self.bk_g,self.bk_b])]
        tex = Texture(Pigment('color',[self.cc_r,self.cc_g,self.cc_b]),
                      Finish('specular',self.specular))
        texo = Texture(Pigment('color',[self.cco_r,self.cco_g,self.cco_b]),
                       Finish('specular',self.specularo))
        for i in xrange(-1,self.expert.nr_obstacles_c()):
            vss, nss, iss = self.save_frame_mesh(i)
            meshes+=[ClothMesh(vss,iss,nss,tex if i == -1 else texo)]

        #render
        scene = Scene(Camera('location',[self.cx,self.cy,self.cz],
                             'sky',[self.ux,self.uy,self.uz],
                             'look_at',[self.fx,self.fy,self.fz],
                             'right',[1,0,0],'up',[0,-1,0],'angle',self.fovy),
                      objects=self.lights+meshes)
        scene.render(path,width=self.w,height=self.h,
                     antialiasing=self.aa if hasattr(self,"aa") else 0.0)
    def save_frame_depth(self, path):

        #setup camera
        camera=vtkCamera()
        camera.SetPosition(self.cx,self.cy,self.cz)
        camera.SetFocalPoint(self.fx,self.fy,self.fz)
        camera.SetViewUp(self.ux,self.uy,self.uz)
        camera.SetViewAngle(self.fovy)
        #setup renderer
        renderer=vtkRenderer()
        renderWindow=vtkRenderWindow()
        renderWindow.SetOffScreenRendering(1)
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.w,self.h)
        for i in xrange(-1,self.expert.nr_obstacles_c()):
            vss, nss, iss = self.save_frame_mesh(i)
            #geometry
            pts=vtkPoints()
            for i in xrange(len(vss)):
                pts.InsertNextPoint(vss[i][0],vss[i][1],vss[i][2])
                tris=vtkCellArray()
            for i in xrange(len(iss)):
                triangle=vtkTriangle()
                triangle.GetPointIds().SetId(0,iss[i][0])
                triangle.GetPointIds().SetId(1,iss[i][1])
                triangle.GetPointIds().SetId(2,iss[i][2])
                tris.InsertNextCell(triangle)
            poly=vtkPolyData()
            poly.SetPoints(pts)
            poly.SetPolys(tris)
            #actor
            mapper=vtkPolyDataMapper()
            mapper.SetInput(poly)
            actor=vtkActor()
            actor.SetMapper(mapper)
            renderer.AddActor(actor)
        renderer.SetBackground(self.bk_r,self.bk_g,self.bk_b)
        renderer.SetActiveCamera(camera)
        renderWindow.Render()
        #output options
        windowToImageFilter=vtkWindowToImageFilter()
        windowToImageFilter.SetInput(renderWindow)
        windowToImageFilter.SetMagnification(1);
        windowToImageFilter.SetInputBufferTypeToZBuffer(); #Extract z buffer value
        windowToImageFilter.Update()
        #scale and shift
        scale=vtkImageShiftScale()
        scale.SetOutputScalarTypeToUnsignedChar()
        scale.SetInputConnection(windowToImageFilter.GetOutputPort())
        scale.SetShift(0)
        scale.SetScale(-255)
        #output
        writer=vtkBMPWriter()
        writer.SetFileName(path)
        writer.SetInputConnection(scale.GetOutputPort())
        writer.Write()
    def save_frame_depth_osmesa(self, path):
        vss, nss, iss = self.save_frame_mesh(-1)
        f = open('mesh.ascii','w')
        #vss
        f.write(str(len(vss)*3)+' ')
        for i in xrange(len(vss)):
            f.write(str(vss[i][0])+' '+str(vss[i][1])+' '+str(vss[i][2])+' ')
        f.write('\n')
        #nss
        f.write(str(len(nss)*3)+' ')
        for i in xrange(len(nss)):
            f.write(str(nss[i][0])+' '+str(nss[i][1])+' '+str(nss[i][2])+' ')
        f.write('\n')
        #iss
        f.write(str(len(iss)*3)+' ')
        for i in xrange(len(iss)):
            f.write(str(iss[i][0])+' '+str(iss[i][1])+' '+str(iss[i][2])+' ')
        f.write('\n')
        f.close()
        #run program
        args=['./mainDepth']
        args+=[str(self.cx),str(self.cy),str(self.cz)]
        args+=[str(self.fx),str(self.fy),str(self.fz)]
        args+=[str(self.ux),str(self.uy),str(self.uz)]
        args+=[str(float(self.fovy)*float(self.h)/float(self.w)),str(float(self.w)/float(self.h))]
        args+=[str(self.w),str(self.h),'',path,'mesh.ascii']
        call(args)
    def save_frame(self, path, f, image=True, depth=False, vtk=False):
        if image:
            self.save_frame_image(path+'/%04i.png'%f)
        if depth:
            self.save_frame_depth_osmesa(path+"/%04i.flt"%f)
            from util import flt2img
            img = flt2img(path+"/%04i.flt"%f)
            import cv2
            cv2.imwrite(path+"/depth%04i.png"%f, img)
            os.remove(path+"/%04i.flt"%f)
            #self.save_frame_depth(path+'/'+str(f)+".bmp")
        if vtk:
            self.save_frame_vtk(path,f)
    def advance(self):
        self.expert.advance_c()
    def illustrate(self, path, nr_frame):
        self.expert.illustrate_c(path.encode(),nr_frame)
    def illustrate_python(self, path, nr_frame, image):
        if os.path.exists(path):
            shutil.rmtree(path)
        os.mkdir(path)
        for i in xrange(nr_frame):
            self.advance()
            self.save_frame(path,i,image)
    #expert functionality
    def expert_tpl(self, handles, x, y, func_ptr):
        assert len(handles) == 4
        ptr = []
        for i in xrange(len(handles)):
            ptr = ptr+[handles[i][0],handles[i][1],handles[i][2]]
        ptr_c = (ct.c_double*len(ptr))(*ptr)
        dss_c=func_ptr(ptr_c,x,y)
        dss_ret=[[dss_c[0],dss_c[1],dss_c[2]],
                 [dss_c[3],dss_c[4],dss_c[5]]]
        self.expert.free_vec_c(dss_c)
        return dss_ret
    def expert_flat_close(self, handles, x, y):
        return self.expert_tpl(handles,x,y,self.expert.expert_flat_close_c)
    def expert_arc_close(self, handles, x, y):
        return self.expert_tpl(handles,x,y,self.expert.expert_arc_close_c)
    def expert_twist_close(self, handles, x, y):
        return self.expert_tpl(handles,x,y,self.expert.expert_twist_close_c)
    def expert_flat(self, handles, x, y):
        return self.expert_tpl(handles,x,y,self.expert.expert_flat_c)
    def expert_arc(self, handles, x, y):
        return self.expert_tpl(handles,x,y,self.expert.expert_arc_c)
    def expert_twist(self, handles, x, y):
        return self.expert_tpl(handles,x,y,self.expert.expert_twist_c)
    def apply_expert(self, handles, robot, delta):
        assert len(handles) == 4 and len(robot) == 2
        xss = []
        for i in xrange(len(handles)):
            xss = xss+[handles[i][0],handles[i][1],handles[i][2]]
        xss_c = (ct.c_double*len(xss))(*xss)

        dss = []
        for i in xrange(len(robot)):
            dss = dss+[robot[i][0],robot[i][1],robot[i][2]]
        dss_c = (ct.c_double * len(dss))(*dss)

        self.expert.apply_expert_c(xss_c,dss_c,delta)
        return [[xss_c[0],xss_c[1],xss_c[2]],
                [xss_c[3],xss_c[4],xss_c[5]],
                [xss_c[6],xss_c[7],xss_c[8]],
                [xss_c[9],xss_c[10],xss_c[11]]]
    def apply_hand(self, handles, hand, delta):
        assert len(handles) == 4 and len(hand) == 2
        xss = []
        for i in xrange(len(handles)):
            xss = xss+[handles[i][0],handles[i][1],handles[i][2]]
        xss_c = (ct.c_double*len(xss))(*xss)
        dss0_c = (ct.c_double * 3)(*(hand[0]))
        dss1_c = (ct.c_double * 3)(*(hand[1]))

        self.expert.apply_hand_c(xss_c,dss0_c,dss1_c,delta)
        return [[xss_c[0],xss_c[1],xss_c[2]],
                [xss_c[3],xss_c[4],xss_c[5]],
                [xss_c[6],xss_c[7],xss_c[8]],
                [xss_c[9],xss_c[10],xss_c[11]]]
    def illustrate_expert_python(self, x, y, delta, \
                                 path, nr_frame, nr_pass, expert,   \
                                 image, depth, vtk):
        if os.path.exists(path):
            shutil.rmtree(path)
        os.mkdir(path)
        handles=[[0,0,0],[x,0,0],[0,y,0],[x,y,0]]
        hands=[[0,0,0],[0,0,0]]
        j=0
        for p in xrange(nr_pass):
            #pick hand location
            while True:
                hands[0] = [random.uniform(-x/4,x/4),random.uniform(-x/4,x/4),random.uniform(-x/4,x/4)]
                hands[1] = [random.uniform(-x/4,x/4)+x,random.uniform(-x/4,x/4),random.uniform(-x/4,x/4)]
                if np.linalg.norm(np.subtract(hands[0],hands[1])) < x:
                    break
            #test
            for i in xrange(nr_frame):
                self.advance()
                handles=self.apply_hand(handles,hands,delta)
                handles =self.apply_expert(handles,expert(handles,x,y),delta)
                self.set_handle(handles)
                self.save_frame(path,j,image,depth,vtk)
                j=j+1

if __name__== "__main__":
    expert = arcsim_expert()
    obs =   \
    [ 
        1,  #this is a box
        10.0,10.0,0.3,  #x,y,z extent
        0,  #this is a capsule
        0.1,0.5     #y length, radius
    ]
    expert.create_sheet(1,5,2,10,   \
                        "./python_sheet",[0,0,-9.81],[0,0,0],0,0,   \
                        obs,2,   \
                        oneside=True,remesh=True)
    motion0=    \
    [
        0 , #this is frame 0 at 0sec
        0,1,-1, #translation
        0,0,0,  #rotation
        
        10, #this is frame 1 at 10sec
        0,1,-2, #translation
        0,0,0,  #rotation
    ]
    motion1=    \
    [
        0 , #this is frame 0 at 0sec
        1,1,-1, #translation
        0,0,0,  #rotation

        10, #this is frame 1 at 10sec
        1,1,-2, #translation
        0,0,0,  #rotation
    ]
    motions=[motion0,motion1]
    expert.setup("./python_sheet/sheet.json", motions)
    #test simulation functionality
    #expert.illustrate_python("./python_sheet_output",100,True)
    #test expert functionality
    expert.illustrate_expert_python(1,2,0.01,"./python_sheet_expert_output",100,5,expert.expert_flat_close,  \
                                    True,True,True)
