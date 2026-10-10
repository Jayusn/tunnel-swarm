"""Independent raw-point body candidate experiment, unused by ROS defaults.

Every member is an actual current return. Static/motion labels remain evidence,
not point-removal gates. Fixed .08 m density ascent creates disjoint basins;
no basin with six returns falls back to its raw parent. This deliberately
admits raw parents over800 points before body separation, unlike production.
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

REGION_RADIUS = float(np.sqrt(.02))
BODY_RADIUS = .08
MIN_REAL_POINTS = 6

def dbscan_groups(points):
    """Original point-core radius/core3; return unbounded parents for splitting."""
    points=np.asarray(points,dtype=float).reshape(-1,3)
    if not len(points): return []
    tree=cKDTree(points)
    core=tree.query_ball_point(points,REGION_RADIUS,return_length=True,workers=1)>=3
    core_indices=np.flatnonzero(core)
    if not len(core_indices): return []
    lookup=np.full(len(points),-1,dtype=int); lookup[core_indices]=np.arange(len(core_indices))
    pairs=tree.query_pairs(REGION_RADIUS,output_type='ndarray')
    if len(pairs): pairs=pairs[core[pairs[:,0]] & core[pairs[:,1]]]
    graph=coo_matrix((np.ones(len(pairs)),(lookup[pairs[:,0]],lookup[pairs[:,1]])),shape=(len(core_indices),len(core_indices))).tocsr()
    _,component=connected_components(graph,directed=False)
    labels=np.full(len(points),-1,dtype=int); labels[core_indices]=component
    border=np.flatnonzero(~core)
    neighborhoods=tree.query_ball_point(points[border],REGION_RADIUS,workers=1,return_sorted=True)
    for index,neighbors in zip(border,neighborhoods):
        adjacent=[n for n in neighbors if core[n]]
        if adjacent: labels[index]=labels[adjacent[0]]
    order = np.argsort(labels, kind='stable')
    keys, starts, counts = np.unique(labels[order], return_index=True, return_counts=True)
    return [order[start:start+count] for key,start,count in zip(keys,starts,counts) if key >= 0]

def density_data(points,parent):
    local=points[parent]; tree=cKDTree(local)
    neighborhoods=tree.query_ball_point(local,BODY_RADIUS,workers=1,return_sorted=True)
    degree=np.fromiter(map(len,neighborhoods),dtype=int)
    return local,tree,neighborhoods,degree

def density_peaks(points,parent,data=None):
    """Local density ascent using only existing .08m neighborhood support.

    Each point chooses the greatest-degree neighbor, smallest original index
    on ties. Strict total-order ascent creates disjoint basins. Retain the
    original raw parent when no basin contains six real observations.
    """
    _,_,neighbors,degree=density_data(points,parent) if data is None else data
    counts=np.fromiter(map(len,neighbors),dtype=int)
    starts=np.r_[0,np.cumsum(counts)[:-1]]
    samples=np.concatenate(neighbors).astype(int)
    maximum=np.maximum.reduceat(degree[samples],starts)
    query=np.repeat(np.arange(len(parent)),counts)
    best=np.where(degree[samples]==maximum[query],samples,len(parent))
    parent_link=np.minimum.reduceat(best,starts)
    while True:
        compressed=parent_link[parent_link]
        if np.array_equal(compressed,parent_link): break
        parent_link=compressed
    order=np.argsort(parent_link,kind='stable')
    _, starts, counts=np.unique(parent_link[order],return_index=True,return_counts=True)
    parts=[parent[order[start:start+count]] for start,count in zip(starts,counts) if count>=MIN_REAL_POINTS]
    return parts if parts else [parent]

def peak_candidates(points,event_types,static,history,min_cluster=6,max_cluster=800,eligible=None):
    assert (min_cluster,max_cluster)==(6,800)
    points=np.asarray(points,dtype=float).reshape(-1,3)
    indices=np.arange(len(points)) if eligible is None else np.flatnonzero(eligible)
    if not len(indices): return []
    local=points[indices]
    members=[]
    for parent in dbscan_groups(local):
        if len(parent)<min_cluster: continue
        for child in density_peaks(local,parent):
            if min_cluster<=len(child)<=max_cluster: members.append(indices[child])
    if not members: return []
    counts=np.array([len(member) for member in members])
    starts=np.r_[0,np.cumsum(counts)[:-1]]
    flat=np.concatenate(members)
    queried=points[flat]
    low=np.minimum.reduceat(queried,starts,axis=0)
    high=np.maximum.reduceat(queried,starts,axis=0)
    centers=(low+high)/2
    sizes=high-low
    types=event_types[flat]
    event_counts=np.column_stack([np.add.reduceat((types&bit)!=0,starts,dtype=np.int64) for bit in (1,2,4)])
    event_fraction=np.add.reduceat(types!=0,starts,dtype=np.int64)/counts
    novels=[np.add.reduceat(frame.world_tree.query(queried,k=1,workers=1)[0]>.06,starts,dtype=np.int64)/counts
            for frame in history if frame.world_tree is not None]
    novelty=np.minimum(novels[0],novels[1]) if len(novels)==2 else np.zeros(len(members))
    return [dict(indices=member,center=centers[i],size=sizes[i],event_fraction=float(event_fraction[i]),
                 event_counts=event_counts[i].tolist(),novelty=float(novelty[i])) for i,member in enumerate(members)]
