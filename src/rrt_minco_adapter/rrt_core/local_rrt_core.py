"""Bidirectional RRT* port of connect_centerline.m, in metres.
Known static scene benchmark; exact segment/AABB collisions with clearance.
"""
import math
import random
import time
import heapq


def length(path):
    return sum(math.dist(a,b) for a,b in zip(path,path[1:]))


class Scene:
    def __init__(self, data, clearance=None):
        self.data=data
        self.margin=float(data.get('clearance',0.4) if clearance is None else clearance)
        self.bounds=data['bounds']
        if self.margin<=0 or not math.isfinite(self.margin):
            raise ValueError('clearance must be positive')
        self.boxes=[(x-sx/2-self.margin,x+sx/2+self.margin,
                     y-sy/2-self.margin,y+sy/2+self.margin) for x,y,sx,sy in data['boxes']]

    def free(self,a,b):
        xmin,xmax,ymin,ymax=self.bounds
        for p in (a,b):
            if not all(math.isfinite(v) for v in p): return False
            if not (xmin+self.margin<p[0]<xmax-self.margin and ymin+self.margin<p[1]<ymax-self.margin):
                return False
        for box in self.boxes:
            lo,hi=0.,1.
            for axis in range(2):
                v=b[axis]-a[axis]
                left,right=box[2*axis:2*axis+2]
                if abs(v)<1e-12:
                    if not left<=a[axis]<=right:
                        lo,hi=1.,0.; break
                else:
                    t1,t2=(left-a[axis])/v,(right-a[axis])/v
                    lo=max(lo,min(t1,t2)); hi=min(hi,max(t1,t2))
                    if lo>hi: break
            if lo<=hi: return False
        return True


class Tree:
    def __init__(self,root):
        self.points=[tuple(root)]; self.parent=[-1]; self.cost=[0.]; self.children=[set()]

    def chain(self,index):
        result=[]; seen=set()
        while index>=0:
            if index in seen: raise RuntimeError('Tree parent cycle')
            seen.add(index); result.append(self.points[index]); index=self.parent[index]
        return result

    def ancestor(self,ancestor,node):
        while node>=0:
            if ancestor==node: return True
            node=self.parent[node]
        return False

    def reparent(self,node,parent):
        self.children[self.parent[node]].remove(node)
        self.parent[node]=parent; self.children[parent].add(node)
        stack=[node]
        while stack:
            i=stack.pop(); p=self.parent[i]
            self.cost[i]=self.cost[p]+math.dist(self.points[i],self.points[p])
            stack.extend(self.children[i])

    def extend(self,target,scene,step,radius):
        nearest=min(range(len(self.points)),key=lambda i:math.dist(self.points[i],target))
        p=self.points[nearest]; distance=math.dist(p,target)
        if distance<1e-9: return None
        q=tuple(p[k]+(target[k]-p[k])*min(step,distance)/distance for k in range(2))
        if not scene.free(p,q): return None
        near=[i for i in range(len(self.points)) if math.dist(self.points[i],q)<radius]
        parent=nearest; cost=self.cost[parent]+math.dist(p,q)
        for i in near:
            candidate=self.cost[i]+math.dist(self.points[i],q)
            if candidate<cost and scene.free(self.points[i],q): parent,cost=i,candidate
        index=len(self.points)
        self.points.append(q);self.parent.append(parent);self.cost.append(cost);self.children.append(set())
        self.children[parent].add(index)
        for i in near:
            if i==0 or self.ancestor(i,index): continue
            candidate=cost+math.dist(q,self.points[i])
            if candidate<self.cost[i]-1e-9 and scene.free(q,self.points[i]): self.reparent(i,index)
        return index


def shortcut(path,scene):
    # Shortest valid subsequence, rather than greedily skipping to the farthest index.
    costs=[0.]+[math.inf]*(len(path)-1);parents=[-1]*len(path)
    for j in range(1,len(path)):
        for i in range(j):
            cost=costs[i]+math.dist(path[i],path[j])
            if cost<costs[j] and scene.free(path[i],path[j]): costs[j]=cost;parents[j]=i
    result=[];j=len(path)-1
    while j>=0: result.append(path[j]);j=parents[j]
    return list(reversed(result))


def refine(path,scene):
    result=shortcut(path,scene)
    # Additional deterministic local shortening; retains the same homotopy route.
    for _ in range(12):
        changed=False
        for i in range(1,len(result)-1):
            a,p,b=result[i-1:i+2];target=tuple((a[k]+b[k])/2 for k in range(2))
            for fraction in (1.,.5,.25,.125,.0625):
                q=tuple(p[k]+fraction*(target[k]-p[k]) for k in range(2))
                if scene.free(a,q) and scene.free(q,b) and math.dist(a,q)+math.dist(q,b)<math.dist(a,p)+math.dist(p,b)-1e-6:
                    result[i]=q;changed=True;break
        result=shortcut(result,scene)
        if not changed: break
    return result


def visibility_reference(scene,start,goal):
    """Comparison only: shortest visibility graph route with 2mm corner offsets.

    Kept separate from RRT* output; this is a numerical reference, not a claim
    that a finite random search has found a globally optimal path.
    """
    points=[tuple(start),tuple(goal)]
    for left,right,bottom,top in scene.boxes:
        for x in (left-.002,right+.002):
            for y in (bottom-.002,top+.002):
                p=(x,y)
                if scene.free(p,p): points.append(p)
    distance=[math.inf]*len(points);distance[0]=0;queue=[(0.,0)]
    while queue:
        cost,i=heapq.heappop(queue)
        if cost!=distance[i]: continue
        if i==1: return cost
        for j,p in enumerate(points):
            new=cost+math.dist(points[i],p)
            if new<distance[j] and scene.free(points[i],p):
                distance[j]=new;heapq.heappush(queue,(new,j))
    return None


def plan(scene,start,goal,seed=7,iterations=3000,timeout=8.,step=0.3,radius=0.8,connect=0.6,
         optimization_time=None):
    # Preserve the full search budget for difficult scenes; bound refinement separately.
    if optimization_time is not None and (not math.isfinite(optimization_time) or optimization_time<0):
        raise ValueError('optimization_time must be finite and nonnegative')
    start,goal=tuple(start),tuple(goal)
    if not scene.free(start,start) or not scene.free(goal,goal):
        raise ValueError('Start/goal in obstacle clearance or outside scene')
    rng=random.Random(seed); trees=[Tree(start),Tree(goal)]
    best=None; first=None; connections=set(); began=time.monotonic()
    used=0
    for iteration in range(iterations):
        used=iteration+1
        elapsed=time.monotonic()-began
        if elapsed>timeout: break
        if first is not None and optimization_time is not None and elapsed-first>=optimization_time:
            break
        side=(iteration%2) if best is None else (0 if iteration%10<7 else 1)
        tree,other=trees[side],trees[1-side]
        if best is None:
            bias=goal if side==0 else start
            if rng.random()<0.7: target=(rng.gauss(bias[0],0.8),rng.gauss(bias[1],0.8))
            else:
                xmin,xmax,ymin,ymax=scene.bounds;target=(rng.uniform(xmin,xmax),rng.uniform(ymin,ymax))
        elif iteration%100<80:
            if rng.random()<0.8:
                centre=rng.choice(best); r=min(1.,length(best)/5+0.2)*math.sqrt(rng.random())
            else:
                i=rng.randrange(len(best)-1);t=rng.random()
                centre=tuple(best[i][k]*(1-t)+best[i+1][k]*t for k in range(2));r=0.3*math.sqrt(rng.random())
            angle=rng.uniform(0,2*math.pi); target=(centre[0]+r*math.cos(angle),centre[1]+r*math.sin(angle))
        else:
            cost=length(best);d=math.dist(start,goal);angle=math.atan2(goal[1]-start[1],goal[0]-start[0])
            theta=rng.uniform(0,2*math.pi);r=math.sqrt(rng.random())
            x=r*cost/2*math.cos(theta);y=r*math.sqrt(max(0,cost*cost-d*d))/2*math.sin(theta)
            target=((start[0]+goal[0])/2+math.cos(angle)*x-math.sin(angle)*y,
                    (start[1]+goal[1])/2+math.sin(angle)*x+math.cos(angle)*y)
        idx=tree.extend(target,scene,step,radius)
        if idx is None: continue
        q=tree.points[idx]
        candidates=[i for i,p in enumerate(other.points) if math.dist(p,q)<connect*(1.5 if best else 1)]
        for i in candidates:
            if scene.free(q,other.points[i]): connections.add((idx,i) if side==0 else (i,idx))
        if connections:
            a,b=min(connections,key=lambda pair:trees[0].cost[pair[0]]+trees[1].cost[pair[1]]+
                    math.dist(trees[0].points[pair[0]],trees[1].points[pair[1]]))
            raw=list(reversed(trees[0].chain(a)))+trees[1].chain(b)
            candidate=shortcut(raw,scene)
            if best is None or length(candidate)<length(best):
                best=candidate
                if first is None: first=time.monotonic()-began
    if best is None: raise RuntimeError('No feasible path within iteration/time budget')
    before=length(best)
    best=refine(best,scene)
    if not all(scene.free(a,b) for a,b in zip(best,best[1:])): raise RuntimeError('Final collision check failed')
    return best,dict(seed=seed,iterations=used,first_path_time=first,total_time=time.monotonic()-began,
                     length=length(best),before_refinement_length=before,nodes=sum(len(t.points) for t in trees))
