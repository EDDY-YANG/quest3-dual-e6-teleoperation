from pathlib import Path
import json, sys, math
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane

def labelname(label):
    a=TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(),a):return a.Get().ToExtString()
    return ""
def bounds(shape):
    box=Bnd_Box();BRepBndLib.AddOptimal_s(shape,box,False,False); return list(box.Get())
def xyz(p):return [p.X(),p.Y(),p.Z()]
def read_assembly(path):
    doc=TDocStd_Document(TCollection_ExtendedString("Gripper"));reader=STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path))!=IFSelect_RetDone or not reader.Transfer(doc):raise RuntimeError("Could not read complete STEP")
    t=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); roots=TDF_LabelSequence();t.GetFreeShapes(roots); nodes=[]
    def walk(label,loc,names):
        name=labelname(label);ref=TDF_Label()
        if t.IsReference_s(label):
            assert t.GetReferredShape_s(label,ref); loc=loc.Multiplied(t.GetLocation_s(label))
        else:ref=label
        names=names+[name]; product=labelname(ref);children=TDF_LabelSequence()
        if t.GetComponents_s(ref,children,False) and children.Length():
            for i in range(1,children.Length()+1):walk(children.Value(i),loc,names)
        else:nodes.append((product.split('^')[0],names,t.GetShape_s(ref).Moved(loc)))
    for i in range(1,roots.Length()+1):walk(roots.Value(i),TopLoc_Location(),[])
    return doc,nodes
def describe(shape):
    faces=TopExp_Explorer(shape,TopAbs_FACE);surfaces=[]
    while faces.More():
        face=TopoDS.Face_s(faces.Current()); a=BRepAdaptor_Surface(face,True);typ=a.GetType();f={}
        if typ==GeomAbs_Cylinder:
            cyl=a.Cylinder();f={'type':'cylinder','p':xyz(cyl.Location()),'axis':xyz(cyl.Axis().Direction()),'r':cyl.Radius()}
        elif typ==GeomAbs_Plane:
            plane=a.Plane(); f={'type':'plane','p':xyz(plane.Location()),'n':xyz(plane.Axis().Direction())}
        if f: f['bbox']=bounds(face);surfaces.append(f)
        faces.Next()
    return surfaces
if __name__=="__main__":
    base=Path(__file__).resolve().parents[1];doc,nodes=read_assembly(base/'cad/gripper_v1.step')
    report=[]
    for i,(name,path,shape) in enumerate(nodes):
        bb=bounds(shape);print(i,name,[round(x,2) for x in bb],path[-1])
        report.append({'id':i,'name':name,'path':path,'bounds':bb,'surfaces':describe(shape)})
    (base/'review').mkdir(exist_ok=True)
    (base/'review/cad_analysis.json').write_text(json.dumps(report,indent=1,ensure_ascii=True),encoding='utf-8')
