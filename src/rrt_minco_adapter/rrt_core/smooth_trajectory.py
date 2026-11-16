"""Collision-certified quadratic corner blends and continuous arc-length tracking."""
import bisect
import math

# The PX4/Gazebo position loop in this scene is configured conservatively at
# 0.6 m/s so it has room to catch the moving reference before the safety hold.
DEFAULT_SPEED = 0.6
DEFAULT_ACCELERATION = 0.8
DEFAULT_TURN_RESPONSE_TIME = 0.4
DEFAULT_TURN_SPEED_FACTOR = 0.8
TANGENTIAL_ACCELERATION_FRACTION = 0.7
NORMAL_ACCELERATION_FRACTION = 0.65
TURN_TRANSITION_ACCELERATION_FRACTION = 0.35
TRACKING_HEIGHT_SCALE_FLOOR = 0.35


def tracking_speed_scale(horizontal_error,height_error):
    """Return a continuous progress limit while the vehicle catches up.

    Horizontal error controls path progress directly.  Height error still
    slows the route, but keeps a small horizontal crawl while it is inside the
    safety limit; a brief altitude lag should not make the XY route appear to
    stop.  Either outer limit still requests a full hold.
    """
    if not all(math.isfinite(v) and v>=0 for v in (horizontal_error,height_error)):
        return 0.
    def fade(error,begin,stop):
        u=max(0.,min(1.,(error-begin)/(stop-begin)))
        return 1.-u*u*(3.-2.*u)
    if horizontal_error>=.25 or height_error>=.20:
        return 0.
    horizontal=fade(horizontal_error,.08,.25)
    height=fade(height_error,.08,.20)
    return horizontal*(TRACKING_HEIGHT_SCALE_FLOOR+(1.-TRACKING_HEIGHT_SCALE_FLOOR)*height)


def tracking_progress_scale(position, reference, reference_velocity, height_error):
    """按误差方向调节轨迹时钟，避免飞机超前时把参考点越减越慢。

    position/reference 是同一世界坐标系的 XY；reference_velocity 必须是
    尚未乘进度倍率的轨迹速度（暂停后也能据此判断轨迹前进方向）。
    沿切向的正误差表示飞机超前，负误差表示飞机落后。
    软减速只使用侧向偏差和落后距离；超前时让参考点继续靠近飞机，
    PX4 仍通过原有位置误差反馈控制飞机。整个过程不跳转参考点、不倒退时间。
    总水平误差达到 0.25m 时仍硬暂停，高度与无效输入保护也保持有效。
    起终点参考速度接近零时，没有可靠切向，退回原总误差规则。
    """
    values=(*position,*reference,*reference_velocity,height_error)
    if not all(math.isfinite(v) for v in values) or height_error<0:
        return 0.
    ex,ey=position[0]-reference[0],position[1]-reference[1]
    horizontal=math.hypot(ex,ey)
    speed=math.hypot(*reference_velocity)
    if horizontal>=.25 or speed<1e-6:
        return tracking_speed_scale(horizontal,height_error)
    tx,ty=reference_velocity[0]/speed,reference_velocity[1]/speed
    along=ex*tx+ey*ty
    cross=ex*ty-ey*tx
    effective=math.hypot(min(along,0.),cross)
    return tracking_speed_scale(effective,height_error)


def lerp(a,b,t): return tuple(a[k]*(1-t)+b[k]*t for k in range(2))


def bezier(control,u):
    a,b,c=control
    p=lerp(lerp(a,b,u),lerp(b,c,u),u)
    d=tuple(2*((1-u)*(b[k]-a[k])+u*(c[k]-b[k])) for k in range(2))
    norm=math.hypot(*d)
    return p,tuple(v/norm for v in d)


def curve_free(control,scene,depth=0):
    # The curve lies in its control hull. A disjoint bounding box certifies it.
    xs=[p[0] for p in control];ys=[p[1] for p in control]
    xmin,xmax,ymin,ymax=scene.bounds;m=scene.margin
    if not (xmin+m<min(xs) and max(xs)<xmax-m and ymin+m<min(ys) and max(ys)<ymax-m): return False
    overlap=any(not (max(xs)<l or min(xs)>r or max(ys)<b or min(ys)>t) for l,r,b,t in scene.boxes)
    if not overlap: return True
    if depth>=16: return False
    a,b,c=control;ab=lerp(a,b,.5);bc=lerp(b,c,.5);mid=lerp(ab,bc,.5)
    return curve_free((a,ab,mid),scene,depth+1) and curve_free((mid,bc,c),scene,depth+1)


def curvature_bound(control):
    a,b,c=control
    d=tuple(2*(b[k]-a[k]) for k in range(2))
    e=tuple(2*(c[k]-2*b[k]+a[k]) for k in range(2))
    e2=sum(v*v for v in e)
    u=max(0.,min(1.,-sum(d[k]*e[k] for k in range(2))/e2)) if e2>1e-15 else 0.
    norm=math.hypot(*(d[k]+u*e[k] for k in range(2)))
    if norm<1e-10: return math.inf
    return abs(d[0]*e[1]-d[1]*e[0])/norm**3


class SmoothTrajectory:
    def __init__(self,path,scene,speed=DEFAULT_SPEED,acceleration=DEFAULT_ACCELERATION,blend=.3,
                 turn_response_time=DEFAULT_TURN_RESPONSE_TIME,
                 turn_speed_factor=DEFAULT_TURN_SPEED_FACTOR):
        if len(path)<2 or not all(math.isfinite(v) and v>0 for v in (speed,acceleration,blend)):
            raise ValueError('Need a path and positive finite trajectory limits')
        if not math.isfinite(turn_response_time) or turn_response_time<0:
            raise ValueError('turn_response_time must be finite and nonnegative')
        if not math.isfinite(turn_speed_factor) or not 0<turn_speed_factor<=1:
            raise ValueError('turn_speed_factor must be in (0,1]')
        self.curves=[];tail=tuple(path[0]);self.blends=0
        for i in range(1,len(path)-1):
            a,p,b=map(tuple,path[i-1:i+2]);da=math.dist(a,p);db=math.dist(p,b)
            if min(da,db)<1e-8: continue
            incoming=tuple((p[k]-a[k])/da for k in range(2))
            outgoing=tuple((b[k]-p[k])/db for k in range(2))
            if math.dist(incoming,outgoing)<1e-5: continue
            trim=min(blend,.3*da,.3*db);accepted=False
            for _ in range(14):
                entry=tuple(p[k]-trim*incoming[k] for k in range(2))
                exit=tuple(p[k]+trim*outgoing[k] for k in range(2))
                control=(entry,p,exit)
                if curve_free(control,scene) and math.isfinite(curvature_bound(control)):
                    self.add_line(tail,entry);self.curves.append(control);tail=exit;self.blends+=1
                    accepted=True;break
                trim*=.5
            if not accepted: raise ValueError('Corner %d cannot be rounded with obstacle clearance; replan required'%i)
        self.add_line(tail,tuple(path[-1]))
        if not self.curves: raise ValueError('Trajectory requires distinct start and goal')
        if not all(curve_free(control,scene) for control in self.curves):
            raise ValueError('Continuous trajectory collision certificate failed')
        self.s=[0.];self.locations=[(0,0.)];self.points=[tuple(path[0])];limits=[speed]
        self.corner_indices=[];self.turn_ranges=[]
        for index,control in enumerate(self.curves):
            bound=curvature_bound(control)
            cap=min(speed,turn_speed_factor*math.sqrt(acceleration*NORMAL_ACCELERATION_FRACTION/bound)) if bound>1e-10 else speed
            entry_index=len(self.points)-1
            limits[-1]=min(limits[-1],cap)
            count=max(2,int(math.ceil((math.dist(control[0],control[1])+math.dist(control[1],control[2]))/.005)))
            for j in range(1,count+1):
                u=j/count;p,_=bezier(control,u)
                self.s.append(self.s[-1]+math.dist(p,self.points[-1]))
                self.points.append(p);self.locations.append((index,u));limits.append(cap)
            self.corner_indices.append(len(self.points)-1)
            if cap<speed:
                self.turn_ranges.append((entry_index,len(self.points)-1,cap))
        # Slow before the geometric bend even with perfect tracking. Each corner
        # has a speed envelope along arc length: finish braking speed*response_time
        # before entry, hold through the bend, then recover with a gentler budget.
        # Taking the minimum handles adjacent corners without a speed-up between them.
        transition_acceleration=acceleration*TURN_TRANSITION_ACCELERATION_FRACTION
        response_distance=speed*turn_response_time
        for entry,exit,cap in self.turn_ranges:
            for i,distance in enumerate(self.s):
                if distance<self.s[entry]:
                    available=max(0.,self.s[entry]-distance-response_distance)
                else:
                    available=max(0.,distance-self.s[exit])
                limits[i]=min(limits[i],math.sqrt(cap*cap+2*transition_acceleration*available))
        # Orthogonal budgets: sqrt(0.7^2+0.65^2)<1, including during turns.
        # Larger tangential budget shortens the final braking phase.
        acceleration*=TANGENTIAL_ACCELERATION_FRACTION
        self.v=limits;self.v[0]=self.v[-1]=0.
        for i in range(1,len(self.v)):
            self.v[i]=min(self.v[i],math.sqrt(self.v[i-1]**2+2*acceleration*(self.s[i]-self.s[i-1])))
        for i in range(len(self.v)-2,-1,-1):
            self.v[i]=min(self.v[i],math.sqrt(self.v[i+1]**2+2*acceleration*(self.s[i+1]-self.s[i])))
        self.times=[0.]
        for i in range(1,len(self.s)):
            self.times.append(self.times[-1]+2*(self.s[i]-self.s[i-1])/(self.v[i]+self.v[i-1]))
        self.total_time=self.times[-1]

    def add_line(self,a,b):
        if math.dist(a,b)>1e-9: self.curves.append((a,lerp(a,b,.5),b))

    def sample(self,elapsed):
        if elapsed<=0: return self.points[0],[0.,0.]
        if elapsed>=self.total_time: return self.points[-1],[0.,0.]
        i=min(len(self.times)-2,bisect.bisect_right(self.times,elapsed)-1)
        dt=elapsed-self.times[i];T=self.times[i+1]-self.times[i]
        acceleration=(self.v[i+1]-self.v[i])/T
        travel=self.v[i]*dt+.5*acceleration*dt*dt
        fraction=max(0.,min(1.,travel/(self.s[i+1]-self.s[i])))
        curve,u1=self.locations[i+1]
        prev_curve,u0=self.locations[i]
        if prev_curve!=curve: u0=0.
        position,tangent=bezier(self.curves[curve],u0+(u1-u0)*fraction)
        speed=max(0.,self.v[i]+acceleration*dt)
        return position,[speed*v for v in tangent]
