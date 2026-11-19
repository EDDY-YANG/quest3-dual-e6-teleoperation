"""Tessellate the supplied AP214 occurrence assembly. All STL/GLB dimensions are metres.
Run with cadquery-ocp-novtk 7.9.3.1.1 and numpy 2.2.6. Does not use the video as mesh input.
"""
from pathlib import Path
from collections import Counter
import hashlib,json,math,struct
import numpy as np
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopLoc import TopLoc_Location
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from step_inspect import read_assembly,bounds,describe

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/"cad/gripper_v1.step"
OUT=ROOT/"files/web/robot-assets/gripper-v1"
GROUPS=("fixed","crank","slide_negative","slide_positive","rod_negative","rod_positive")
def role(part,b):
    cx=(b[0]+b[3])/2;cy=(b[1]+b[4])/2
    side="negative" if cx<0 else "positive"
    if part=="Side Linkage":return "rod_negative" if cy>0 else "rod_positive"
    if part in ("Slider","Claw","Flange"):return "slide_"+side
    if part.startswith("Thrust Bearing"):return "crank" if abs(cx)>15 else "slide_"+side
    if part in ("Middle Linkage","Servo arm") or part.startswith("Gear_"):return "crank"
    return "fixed"
def mat(part):
    if part=="Claw":return "finger"
    if part.startswith("MG996RBody"):return "servo"
    if part=="Gripper PCB":return "pcb"
    if part in ("PCB Cover","PCB Lid","PCB Linkage","Rail Linkage","Servo flange","Flange"):return "housing"
    if part in ("Slide rail","Slider") or part.startswith("Thrust Bearing"):return "steel"
    if part.startswith("Gear_"):return "brass"
    return "link"
COLORS={"finger":[.20,.23,.25],"housing":[.83,.84,.78],"servo":[.64,.07,.22],"link":[.51,.55,.57],"steel":[.34,.37,.39],"brass":[.66,.47,.17],"pcb":[.065,.25,.23]}
def mesh(shape,origin):
    BRepMesh_IncrementalMesh(shape,.10,False,.24,True).Perform()
    vertices=[]; faces=[];count=0; ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        face=TopoDS.Face_s(ex.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is not None:
            tr=loc.Transformation()
            v=np.array([list(tri.Node(i).Transformed(tr).Coord()) for i in range(1,tri.NbNodes()+1)])
            vertices.append((v-origin)*.001)
            ids=np.array([list(tri.Triangle(i).Get()) for i in range(1,tri.NbTriangles()+1)],dtype=int)-1
            if face.Orientation()==TopAbs_REVERSED:ids=ids[:,[0,2,1]]
            faces.append(ids+count);count+=len(v)
        ex.Next()
    v=np.vstack(vertices);f=np.vstack(faces);norms=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    valid=np.linalg.norm(norms,axis=1)>1.e-15;f=f[valid];norms=norms[valid]
    normals=np.zeros_like(v)
    for col in range(3):np.add.at(normals,f[:,col],norms)
    l=np.linalg.norm(normals,axis=1);normals[l>0]/=l[l>0,None];normals[l==0]=[0,0,1]
    return v.astype('<f4'),normals.astype('<f4'),f.astype('<u4').ravel()
def pivot(item,sign):
    holes=[f for f in describe(item) if f["type"]=="cylinder" and abs(f["axis"][2])>.9999 and abs(f["r"]-2.15)<.005 and sign*f["p"][0]>0]
    centers=np.array([f["p"][:2] for f in holes]);assert len(holes) and np.ptp(centers,axis=0).max()<.005
    return centers.mean(0)
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    doc,items=read_assembly(SOURCE);assert len(items)==28
    plate=next(shape for part,_,shape in items if part=="PCB Cover");mount_z=bounds(plate)[2]
    # The four bottom fastener holes locate the mating circle independently of the servo center.
    mount_holes={tuple(round(x,5) for x in f["p"][:2]) for f in describe(plate) if f['type']=='cylinder' and abs(f['r']-3.15)<.003 and abs(f['bbox'][2]-mount_z)<.002}
    assert len(mount_holes)==4
    origin=np.array([*np.mean(list(mount_holes),axis=0),mount_z])
    link=next(shape for p,_,shape in items if p=="Side Linkage" and bounds(shape)[1]>0)
    crank=pivot(link,1);slider=pivot(link,-1); radius=float(np.linalg.norm(crank));rod_len=float(np.linalg.norm(crank-slider))
    graph={"asset":{"version":"2.0","generator":"E6 SDK / OCCT AP214 occurrence mesh"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"name":"gripper_v1","children":list(range(1,7))}]+[{"name":g,"children":[]} for g in GROUPS],"meshes":[],"materials":[],"accessors":[],"bufferViews":[]}
    for key,rgb in COLORS.items():graph["materials"].append({"name":key,"doubleSided":False,"pbrMetallicRoughness":{"baseColorFactor":[*rgb,1],"metallicFactor":.42 if key in ("steel","brass") else .08,"roughnessFactor":.50}})
    raw=bytearray()
    def accessor(a,tag,component,target):
        raw.extend(b'\0'*(-len(raw)%4));offset=len(raw);blob=a.tobytes();raw.extend(blob)
        view=len(graph['bufferViews']);graph['bufferViews'].append({'buffer':0,'byteOffset':offset,'byteLength':len(blob),'target':target})
        obj={'bufferView':view,'count':len(a),'componentType':component,'type':tag}
        if tag=='VEC3':obj.update(min=a.min(0).astype(float).tolist(),max=a.max(0).astype(float).tolist())
        graph['accessors'].append(obj);return len(graph['accessors'])-1
    occurrence=[];tip={}
    for n,(part,path,shape) in enumerate(items):
        bb=bounds(shape);g=role(part,bb);verts,normals,indices=mesh(shape,origin)
        if part=="Claw":
            distal=verts[verts[:,2]>=verts[:,2].max()-.030,0]*1000.
            tip[g]=float(distal.max() if g=="slide_negative" else distal.min())
        primitive={'attributes':{'POSITION':accessor(verts,'VEC3',5126,34962),'NORMAL':accessor(normals,'VEC3',5126,34962)},'indices':accessor(indices,'SCALAR',5125,34963),'material':list(COLORS).index(mat(part))}
        graph["meshes"].append({'primitives':[primitive]})
        node_name=f"part_{n:02d}_{part.split('_')[0].replace(' ','_')}"
        graph['nodes'][GROUPS.index(g)+1]['children'].append(len(graph['nodes']))
        graph['nodes'].append({'name':node_name,'mesh':n,'extras':{'cad_occurrence':path[-1],'motion_group':g}})
        occurrence.append({'id':node_name,'product':part,'occurrence':path[-1],'group':g,'triangles':len(indices)//3})
    graph['buffers']=[{'byteLength':len(raw)}];raw.extend(b'\0'*(-len(raw)%4))
    meta=json.dumps(graph,ensure_ascii=True,separators=(',',':')).encode();meta+=b' '*(-len(meta)%4)
    glb=struct.pack('<4sII',b'glTF',2,28+len(meta)+len(raw))+struct.pack('<I4s',len(meta),b'JSON')+meta+struct.pack('<I4s',len(raw),b'BIN\0')+raw
    (OUT/'gripper.glb').write_bytes(glb)
    data={'schema':1,'id':'gripper-v1','units':'m for scene, mm for mechanism','step_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'glb_sha256':hashlib.sha256(glb).hexdigest(),'cad_mount_center_mm':origin.tolist(),'mount_holes_xy_mm':sorted(mount_holes),'crank_pin_rest_mm':crank.tolist(),'slide_pin_rest_mm':slider.tolist(),'rod_length_mm':rod_len,'crank_radius_mm':radius,'closed_tip_clearance_mm':max(0,round(tip['slide_positive']-tip['slide_negative'],4)),'open_crank_delta_deg':75.0,'stroke_basis':'Nominal CAD demonstration arc within rail travel; bench calibration required.','groups':list(GROUPS),'occurrences':occurrence,'servo_hardware':'XINHUI 20KG upgrade / 180-degree positional servo; user-supplied datasheet image','servo_geometry':'STEP contains MG996R body envelope; actual XINHUI spline, fit and clocking require bench check.'}
    (OUT/'mechanism.json').write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print(json.dumps({'parts':len(items),'triangles':sum(x['triangles'] for x in occurrence),'bytes':len(glb),'mount':origin.tolist(),'crank_pin_mm':crank.tolist(),'slide_pin_mm':slider.tolist(),'rod_mm':rod_len,'closed_clearance_mm':data['closed_tip_clearance_mm']}))
if __name__=='__main__':main()
