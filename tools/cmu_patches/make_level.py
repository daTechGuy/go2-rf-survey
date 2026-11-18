import rclpy, numpy as np, math, time, yaml, os
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
rclpy.init(); n=rclpy.create_node('mk'); got=[]
n.create_subscription(PointCloud2,'/utlidar/transformed_cloud',lambda m:got.append(m),20)
t0=time.time()
while len(got)<12 and time.time()-t0<25: rclpy.spin_once(n,timeout_sec=0.2)
rng=np.random.default_rng(1); normals=[]
for m in got:
    p=np.array([[a,b,c] for a,b,c in pc2.read_points(m,field_names=('x','y','z'),skip_nans=True)])
    r=np.hypot(p[:,0],p[:,1]); q=p[(r>0.8)&(r<4.5)]
    best=None
    for _ in range(500):
        i=rng.choice(len(q),3,replace=False); a,b,c=q[i]
        nv=np.cross(b-a,c-a); nn=np.linalg.norm(nv)
        if nn<1e-6: continue
        nv/=nn
        if nv[2]<0: nv=-nv
        inl=(np.abs((q-a)@nv)<0.03).sum()
        if best is None or inl>best[0]: best=(inl,nv,a)
    mask=np.abs((q-best[2])@best[1])<0.03; f=q[mask]
    u,s,vt=np.linalg.svd(f-f.mean(0),full_matrices=False); nf=vt[2]
    if nf[2]<0: nf=-nf
    if mask.sum()/len(q)>0.55: normals.append(nf)
N=np.array(normals); nm=N.mean(0); nm/=np.linalg.norm(nm)
spread=float(np.degrees(np.arccos(np.clip(N@nm,-1,1))).max())
z=np.array([0,0,1.0]); ax=np.cross(nm,z); s=np.linalg.norm(ax); c=nm@z
K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
R=np.eye(3)+K+K@K*((1-c)/(s*s))
print('scans used',len(N),' mean normal',np.round(nm,3),' tilt %.1f deg  max spread %.2f deg'%(math.degrees(math.acos(c)),spread))
print('check R@normal =',np.round(R@nm,4),' det %.3f'%np.linalg.det(R))
_prev={}
try:
    _prev=yaml.safe_load(open(os.path.expanduser('~/Desktop/level_correction.yaml'))) or {}
except Exception:
    pass
out={'floor_normal_body':[float(v) for v in nm],'R_level':[[float(v) for v in row] for row in R],'tilt_deg':float(math.degrees(math.acos(c))),'scans':int(len(N)),'measured':time.strftime('%F %T')}
for _k in ('cloud_yaw_deg','cloud_yaw_note'):
    if _k in _prev: out[_k]=_prev[_k]   # keep the separately measured cloud yaw
yaml.safe_dump(out,open(os.path.expanduser('~/Desktop/level_correction.yaml'),'w'))
print('saved ~/Desktop/level_correction.yaml')
