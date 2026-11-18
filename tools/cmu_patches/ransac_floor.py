import rclpy, numpy as np, math, time, sys
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
topic=sys.argv[1]
rclpy.init(); n=rclpy.create_node('rf'); got=[]
n.create_subscription(PointCloud2,topic,lambda m:got.append(m),5)
t0=time.time()
while len(got)<5 and time.time()-t0<15: rclpy.spin_once(n,timeout_sec=0.2)
rng=np.random.default_rng(0)
for m in got:
    p=np.array([[a,b,c] for a,b,c in pc2.read_points(m,field_names=('x','y','z'),skip_nans=True)])
    r=np.hypot(p[:,0],p[:,1]); q=p[(r>0.8)&(r<4.5)]
    best=None
    for _ in range(400):
        i=rng.choice(len(q),3,replace=False); a,b,c=q[i]
        nv=np.cross(b-a,c-a); nn=np.linalg.norm(nv)
        if nn<1e-6: continue
        nv/=nn
        if nv[2]<0: nv=-nv
        d=np.abs((q-a)@nv); inl=(d<0.03).sum()
        if best is None or inl>best[0]: best=(inl,nv,a)
    inl,nv,a=best
    mask=np.abs((q-a)@nv)<0.03; f=q[mask]
    A=f-f.mean(0); u,s,vt=np.linalg.svd(A,full_matrices=False); nf=vt[2]
    if nf[2]<0: nf=-nf
    tilt=math.degrees(math.acos(np.clip(nf[2],-1,1)))
    pitch=math.degrees(math.atan2(nf[0],nf[2])); roll=math.degrees(math.atan2(nf[1],nf[2]))
    print('%s: pts %d inliers %d (%.0f%%)  tilt %.1f deg  [normal x%+.2f y%+.2f z%+.2f]  pitch-like %+.1f roll-like %+.1f  floor z at centroid %.2f'%(topic.split('/')[-1],len(q),mask.sum(),100*mask.sum()/len(q),tilt,nf[0],nf[1],nf[2],pitch,roll,f[:,2].mean()))
