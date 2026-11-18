import rclpy, time
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
rclpy.init(); n=rclpy.create_node('lag'); last={}
def mk(k):
    def cb(m):
        s=Time.from_msg(m.header.stamp).nanoseconds*1e-9
        last[k]=(time.time()-s, time.time())
    return cb
n.create_subscription(Imu,'/utlidar/imu',mk('raw /utlidar/imu'),50)
n.create_subscription(PointCloud2,'/utlidar/cloud',mk('raw /utlidar/cloud'),5)
n.create_subscription(Imu,'/utlidar/transformed_imu',mk('transformed_imu'),50)
n.create_subscription(PointCloud2,'/utlidar/transformed_cloud',mk('transformed_cloud'),5)
n.create_subscription(Odometry,'/state_estimation',mk('/state_estimation'),50)
n.create_subscription(PointCloud2,'/registered_scan',mk('/registered_scan'),5)
t0=time.time(); nxt=0
while time.time()-t0<21:
    rclpy.spin_once(n,timeout_sec=0.05)
    if time.time()-t0>=nxt:
        nxt+=10; print('t=%2.0fs  '%(time.time()-t0)+'  '.join('%s %+.1fs'%(k.split('/')[-1],v[0]) for k,v in sorted(last.items())),flush=True)
