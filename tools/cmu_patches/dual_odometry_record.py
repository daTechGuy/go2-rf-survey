import rclpy, time, math, json
from nav_msgs.msg import Odometry
rclpy.init(); n=rclpy.create_node('dual_rec'); L={'slam':None,'leg':None}
def mk(k):
    def cb(m):
        q=m.pose.pose.orientation; p=m.pose.pose.position
        L[k]=(p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    return cb
n.create_subscription(Odometry,'/state_estimation',mk('slam'),50)
n.create_subscription(Odometry,'/utlidar/robot_odom',mk('leg'),50)
rows=[]; t0=time.time(); last=0
while time.time()-t0<150:
    rclpy.spin_once(n,timeout_sec=0.02)
    if time.time()-last>=0.1 and L['slam'] and L['leg']:
        rows.append([round(time.time()-t0,2)]+[round(v,4) for v in L['slam']]+[round(v,4) for v in L['leg']]); last=time.time()
json.dump(rows,open('/tmp/dual_rec.json','w')); print('done',len(rows))
