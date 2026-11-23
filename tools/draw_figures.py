"""Publication figures from the existing architecture and official E6 meshes.
No robot/network I/O. All illustrated poses are graphics, not measurements.
"""
from pathlib import Path
import json, struct, math, shutil, os, subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle, Circle, Arc
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'assets'
MODEL=Path(os.environ.get('E6_MODEL_DIR', str(ROOT/'sdk/app/web/robot-assets/e6')))
INK='#172331'; MUTED='#586779'; BORDER='#b7c5d4'; BLUE='#2563eb'; ORANGE='#d97706'; PURPLE='#7c3aed'; RED='#bc3344'; POWER='#7b8491'
plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','font.size':11,'axes.unicode_minus':False})

def transform(xyz,rpy):
    x,y,z=rpy
    rx=np.array([[1,0,0],[0,math.cos(x),-math.sin(x)],[0,math.sin(x),math.cos(x)]])
    ry=np.array([[math.cos(y),0,math.sin(y)],[0,1,0],[-math.sin(y),0,math.cos(y)]])
    rz=np.array([[math.cos(z),-math.sin(z),0],[math.sin(z),math.cos(z),0],[0,0,1]])
    t=np.eye(4);t[:3,:3]=rz@ry@rx;t[:3,3]=xyz;return t

def render_e6():
    output=ASSETS/'e6_model.png'
    if not (MODEL/'model.json').is_file():
        raise FileNotFoundError('Keep the supplied assets/e6_model.png, or set E6_MODEL_DIR to the SDK visual-model directory containing model.json, meshes and UPSTREAM-LICENSE.')
    model=json.loads((MODEL/'model.json').read_text());matrices={model['root_link']:np.eye(4)}
    q=np.radians([90,-25,-70,5,90,0]);index=0
    for joint in model['joints']:
        t=transform(joint['xyz'],joint['rpy'])
        if joint['type']=='revolute':
            a=q[index];index+=1;t=t@transform([0,0,0],[0,0,a])
        matrices[joint['child']]=matrices[joint['parent']]@t
    fig=plt.figure(figsize=(4,4),dpi=180);ax=fig.add_subplot(projection='3d',computed_zorder=False)
    all_points=[]
    for link in model['links']:
        for visual in link['visuals']:
            data=(MODEL/visual['mesh']).read_bytes();count=struct.unpack_from('<I',data,80)[0]
            records=np.frombuffer(data,offset=84,count=count,dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')]))
            vertices=records['vertices'].astype(float)*np.array(visual['scale'])
            t=matrices[link['name']]@transform(visual['xyz'],visual['rpy'])
            vertices=vertices@t[:3,:3].T+t[:3,3];all_points.append(vertices.reshape(-1,3))
            normals=np.cross(vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-10)
            light=np.array([.4,-.6,.7]);light/=np.linalg.norm(light)
            shade=.60+.35*np.maximum(0,normals@light)
            colors=np.column_stack([shade*.91,shade*.94,shade,np.ones(len(shade))])
            mesh=Poly3DCollection(vertices,facecolors=colors,edgecolor='none',linewidth=0,rasterized=True)
            ax.add_collection3d(mesh)
    # Preserve the original robot pose and use the catalog mount for the real tool.
    subprocess.run(['node',str(ROOT/'tools/export_gripper_figure.mjs')],check=True)
    tool=np.fromfile(ROOT/'runtime/gripper-figure-triangles.bin',dtype='<f4').reshape(-1,3,3).astype(float)
    flange=matrices['Link6'];tool=tool@flange[:3,:3].T+flange[:3,3]
    normals=np.cross(tool[:,1]-tool[:,0],tool[:,2]-tool[:,0])
    normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-10)
    shade=.5+.4*np.maximum(0,normals@light)
    colors=np.column_stack([shade*.47,shade*.56,shade*.64,np.ones(len(shade))])
    ax.add_collection3d(Poly3DCollection(tool,facecolors=colors,edgecolor='none',linewidth=0,rasterized=True))
    all_points.append(tool.reshape(-1,3))
    points=np.concatenate(all_points);low=points.min(0);high=points.max(0);center=(low+high)/2;span=max(high-low)*.56
    ax.set(xlim=(center[0]-span,center[0]+span),ylim=(center[1]-span,center[1]+span),zlim=(center[2]-span,center[2]+span))
    ax.set_box_aspect((1,1,1));ax.view_init(elev=18,azim=-65);ax.set_axis_off();fig.subplots_adjust(0,0,1,1)
    fig.savefig(output,transparent=True,bbox_inches='tight',pad_inches=0);plt.close(fig)
    shutil.copy2(MODEL/'UPSTREAM-LICENSE',ASSETS/'E6_MODEL_LICENSE')

def canvas(w,h):
    fig,ax=plt.subplots(figsize=(w,h),dpi=180);fig.patch.set_facecolor('white');ax.set(xlim=(0,w),ylim=(h,0));ax.axis('off');fig.subplots_adjust(0,0,1,1);return fig,ax

def txt(ax,x,y,s,size=11,color=INK,ha='center',weight='normal',va='center'):
    ax.text(x,y,s,fontsize=size,color=color,ha=ha,va=va,weight=weight,linespacing=1.55)

def box(ax,x,y,w,h,title,body='',accent=None):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.02,rounding_size=0.09',facecolor='white',edgecolor=BORDER,linewidth=1.1))
    if accent:ax.plot([x+.12,x+w-.12],[y+.08,y+.08],color=accent,lw=2.5)
    txt(ax,x+w/2,y+.29,title,12,weight='bold')
    if body:txt(ax,x+w/2,y+.75,body,10,MUTED)

def arrow(ax,points,color=BLUE,dashed=False,both=False,lw=1.6):
    for a,b in zip(points[:-2],points[1:-1]):ax.plot([a[0],b[0]],[a[1],b[1]],color=color,lw=lw,ls='--' if dashed else '-')
    a,b=points[-2:];ax.add_patch(FancyArrowPatch(a,b,arrowstyle='<->' if both else '-|>',mutation_scale=12,color=color,lw=lw,linestyle='--' if dashed else '-'))

def export(fig,name):
    svg=ASSETS/(name+'.svg')
    fig.savefig(svg,facecolor='white');fig.savefig(ASSETS/(name+'.png'),facecolor='white',dpi=190);plt.close(fig)
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text(encoding='utf-8').splitlines())+'\n',encoding='utf-8')

def hardware():
    fig,ax=canvas(15,10.7)
    txt(ax,.5,.38,'SYSTEM SETUP',12,MUTED,ha='left',weight='bold')
    txt(ax,.5,.8,'Quest 3, host computer and dual robot interfaces',17,ha='left',weight='bold')
    box(ax,.55,1.65,2.55,3.65,'Meta Quest 3','',BLUE)
    # Deliberately schematic headset/controller symbols, not fabricated CAD.
    ax.add_patch(FancyBboxPatch((1.13,2.37),1.4,.58,boxstyle='round,pad=0.03,rounding_size=0.2',fc='#edf2f7',ec=INK,lw=1.6))
    for x in [1.44,1.8,2.16]:ax.add_patch(FancyBboxPatch((x,2.51),.10,.26,boxstyle='round,pad=0.01,rounding_size=0.05',fc=INK,ec='none'))
    for x in [1.2,2.3]:
        ax.add_patch(Circle((x,3.41),.17,fc='#edf2f7',ec=INK,lw=1.2));ax.plot([x,x-.05],[3.55,3.89],color=INK,lw=6,solid_capstyle='round')
    txt(ax,1.82,4.28,'Left / right controllers\nWebXR pose + buttons',10)
    txt(ax,1.82,4.95,'Internal batteries',10,POWER)
    box(ax,3.85,2.63,1.65,1.45,'Wi-Fi AP','Local network')
    arrow(ax,[(3.11,3.2),(3.83,3.2)],both=True);txt(ax,3.47,2.98,'Wi-Fi',9)
    box(ax,6.25,1.65,3.1,4.35,'Windows PC','',BLUE)
    ax.add_patch(Rectangle((6.94,2.42),1.72,.94,ec=INK,fc='#f3f6fa',lw=1.5));ax.plot([7.8,7.8],[3.36,3.62],color=INK,lw=1.5);ax.plot([7.35,8.25],[3.62,3.62],color=INK,lw=1.5)
    txt(ax,7.8,4.08,'Python control service\nLocal recording + monitoring',10)
    txt(ax,7.8,5.3,'Source-bound Ethernet NICs\nUSB serial to each gripper Uno',9.2)
    arrow(ax,[(5.51,3.2),(6.23,3.2)],both=True);txt(ax,5.87,2.98,'LAN',9)
    txt(ax,4.67,4.72,'HTTPS / WSS :8443\nInput to PC; status to Quest',9.5,MUTED)
    for y,label,nic in [(1.65,'Left DOBOT E6','NIC L'),(4.48,'Right DOBOT E6','NIC R')]:
        box(ax,11.25,y,2.45,2.3,label)
        im=plt.imread(ASSETS/'e6_model.png')
        ys,xs=np.where(im[:,:,3]>.01)
        im=im[max(0,ys.min()-10):ys.max()+11,max(0,xs.min()-10):xs.max()+11]
        ax.imshow(im,extent=(11.55,13.4,y+1.82,y+.55),aspect='auto',zorder=3)
        txt(ax,12.48,y+2.05,'E6 + prototype gripper',9.5,MUTED)
        yy=y+1.35
        arrow(ax,[(9.37,yy),(11.23,yy)],both=True)
        txt(ax,10.3,yy-.26,nic+'  /  Ethernet',9.5)
        txt(ax,10.3,yy+.28,'29999 command\n30004 feedback',8.8,MUTED)
        ax.add_patch(Circle((14.35,y+.93),.22,ec=RED,fc='#fce9eb',lw=1.5));txt(ax,14.35,y+.93,'STOP',7.8,RED,weight='bold')
        txt(ax,14.35,y+.43,'E-stop',10,RED)
        arrow(ax,[(14.12,y+.93),(13.72,y+.93)],color=RED)
    # Power routing is a topology, not a terminal-level wiring diagram.
    box(ax,.55,6.25,2.55,.85,'AC mains')
    box(ax,4.0,6.25,2.5,.85,'PC power adapter')
    arrow(ax,[(3.12,6.67),(3.98,6.67)],POWER,True)
    arrow(ax,[(6.52,6.67),(7.8,6.67),(7.8,6.02)],POWER,True)
    box(ax,9.25,7.0,4.45,.85,'OEM robot power supplies (x2)')
    arrow(ax,[(1.82,7.12),(1.82,7.43),(9.23,7.43)],POWER,True)
    arrow(ax,[(13.72,7.43),(14.9,7.43),(14.9,4.18),(12.5,4.18),(12.5,3.97)],POWER,True)
    arrow(ax,[(13.72,7.43),(14.9,7.43),(14.9,6.95),(12.5,6.95),(12.5,6.8)],POWER,True)
    txt(ax,.55,5.72,'Independent arm networks; one Uno per equipped gripper.',10,MUTED,ha='left')
    box(ax,.55,8.65,3.0,1.05,'PC / USB','Gripper target + ACK')
    box(ax,4.1,8.65,2.8,1.05,'Arduino Uno','USB powered · one per claw')
    box(ax,8.05,8.65,2.9,1.05,'Gripper servo','PWM from Uno D2')
    box(ax,11.65,8.65,2.8,1.05,'Servo supply','External power')
    arrow(ax,[(3.57,9.25),(4.08,9.25)],both=True)
    arrow(ax,[(6.92,9.25),(8.03,9.25)]);txt(ax,7.47,9.03,'PWM',8.5,BLUE)
    arrow(ax,[(11.63,9.25),(10.97,9.25)],POWER,True)
    txt(ax,7.5,10.05,'Common ground between Uno and servo supply. Robot E-stops act on the robot power path.',9.5,MUTED)
    for x,label,c,d in [(.65,'Data (bidirectional)',BLUE,False),(4.0,'Power (schematic)',POWER,True),(7.2,'Hardware E-stop',RED,False)]:
        ax.plot([x,x+.4],[10.43,10.43],color=c,lw=1.7,ls='--' if d else '-');txt(ax,x+.5,10.43,label,9.4,MUTED,ha='left')
    export(fig,'system_setup')

def architecture():
    fig,ax=canvas(15,12.0)
    txt(ax,.45,.35,'DATA FLOW & CONTROL ARCHITECTURE',12,MUTED,ha='left',weight='bold')
    txt(ax,.45,.8,'Online control pipeline and measured feedback',16,ha='left',weight='bold')
    box(ax,.5,1.55,2.5,1.3,'Quest WebXR client','gripSpace poses + buttons\nsequence / reference epoch')
    box(ax,3.45,1.55,2.5,1.3,'Secure input session','HTTPS / WSS pairing\ncontinuous pose stream')
    box(ax,6.4,1.55,3.05,1.3,'Input admission','tracking / visibility / age\nlatest sample; no stale replay')
    box(ax,10.0,1.55,4.45,1.3,'Frame mapping + clutch','Per-arm yaw calibration; shared forward\nFresh measured anchors at each re-grip')
    for a,b in [(3.02,3.43),(5.97,6.38),(9.47,9.98)]:arrow(ax,[(a,2.2),(b,2.2)])
    arrow(ax,[(12.2,2.87),(12.2,3.35),(2.0,3.35),(2.0,3.78)])
    txt(ax,7.3,3.16,'Per-arm control instance (x2)   |   position in mm, orientation from rotations',10,MUTED)
    box(ax,.5,3.8,3.0,1.35,'Target pose + cone','Grip-anchored position / rotation\nStart posture + downward tilt checks')
    box(ax,4.0,3.8,3.3,1.35,'Solver / reference branch','Current: RangedIK v7 direct\nLegacy: bounded IK, optionally QP',PURPLE)
    box(ax,7.8,3.8,3.15,1.35,'Candidate validation','Controller FK / joint bounds\nfresh feedback / queue / context')
    box(ax,11.45,3.8,3.0,1.35,'Bound DOBOT driver','Explicit source NIC per arm\nServoJ -> controller ACK')
    for a,b in [(3.52,3.98),(7.32,7.78),(10.97,11.43)]:arrow(ax,[(a,4.47),(b,4.47)])
    arrow(ax,[(14.47,4.47),(14.83,4.47),(14.83,6.65),(14.47,6.65)])
    box(ax,11.45,6.0,3.0,1.3,'Left / right E6','Robot-side servo execution\nPhysical robot + controller')
    box(ax,7.8,6.0,3.15,1.3,'Feedback validation','30004: measured joints + TCP\nidentity / age / connection generation')
    arrow(ax,[(11.43,6.65),(10.97,6.65)])
    arrow(ax,[(9.38,5.98),(9.38,5.18)])
    box(ax,4.0,6.0,3.3,1.3,'State + group supervision','Prepare / Arm / independent Grip\nparticipant faults -> group Stop')
    arrow(ax,[(7.78,6.65),(7.32,6.65)])
    arrow(ax,[(5.65,5.98),(5.65,5.18)],color=INK)
    box(ax,.5,6.0,3.0,1.3,'Quest HUD + PC monitor','Measured state / goals / readiness\nexplicit actions; no fabricated feedback')
    arrow(ax,[(3.98,6.65),(3.52,6.65)])
    arrow(ax,[(.48,6.65),(.18,6.65),(.18,2.2),(.48,2.2)])
    txt(ax,1.8,5.62,'HUD feedback over WSS',9,MUTED)
    box(ax,.5,8.0,4.35,1.15,'Local episode recording','Input / targets / commands / measured feedback\nBounded queue; metadata + events + analysis')
    arrow(ax,[(4.35,7.32),(4.35,7.6),(2.0,7.6),(2.0,7.98)],color=INK)
    box(ax,5.35,8.0,4.1,1.15,'Stop + recovery handling','Revoke motion -> Stop\nConfirm using fresh stationary feedback')
    arrow(ax,[(6.45,7.32),(6.45,7.98)],color=INK)
    arrow(ax,[(9.47,8.58),(10.95,8.58),(10.95,5.48),(12.85,5.48),(12.85,5.17)],color=INK)
    box(ax,11.45,8.0,3.0,1.15,'Hardware E-stops','Independent of PC / Wi-Fi',RED)
    arrow(ax,[(12.95,7.98),(12.95,7.32)],color=RED)
    txt(ax,.5,9.6,'INDEPENDENT GRIPPER CHANNEL',10,MUTED,ha='left',weight='bold')
    box(ax,.5,9.9,3.0,1.15,'Index trigger','Continuous value 0–1\nMiddle Grip controls arm following')
    box(ax,4.0,9.9,3.3,1.15,'PC gripper target','Calibrated open / closed pulses\nRelease overrides closing target')
    box(ax,7.8,9.9,3.15,1.15,'USB serial + Uno','115200 baud / session + sequence\nTarget ACK / PWM on D2')
    box(ax,11.45,9.9,3.0,1.15,'Gripper servo','Externally powered positional servo\nActual jaw gap unmeasured')
    for a,b in [(3.52,3.98),(7.32,7.78),(10.97,11.43)]:arrow(ax,[(a,10.5),(b,10.5)])
    txt(ax,.5,11.55,'RangedIK v7 direct drives the arm. Optional gripper PI requires an external position sensor; the demonstrations use direct PWM.',9.5,MUTED,ha='left')
    export(fig,'data_flow')

if __name__=='__main__':
    ASSETS.mkdir(exist_ok=True);render_e6();hardware();architecture();print('Generated system_setup and data_flow as SVG + PNG')
