"""Independent offline actual-return depth experiment; unused by ROS entrypoints.

The extra 3 cm lateral bound is necessary geometry, not proof of the same emitted
beam, visible-ray coverage or actual point acquisition time. The production
ROS detector does not call this helper. Its query is copied from the frozen
scratch experiment; return filtering can change Static to Foreground.
"""
import numpy as np
from event_core import DepthFrame as OriginalDepthFrame,Query,LATERAL,DEPTH


class ActualReturnDepthFrame(OriginalDepthFrame):
    """Only query changes; actual point trees, views and event bits are retained."""
    def query(self,points):
        points=np.asarray(points,dtype=float).reshape(-1,3)
        size=len(points)
        front=np.zeros(size,dtype=bool)
        behind=front.copy();static=front.copy()
        unknown=np.ones(size,dtype=bool);nearest=np.full(size,-1,dtype=int)
        if self.tree is None or size==0:
            return Query(front,behind,static,unknown,nearest)
        rays=(points-self.origin)@self.rotation
        ranges=np.linalg.norm(rays,axis=1)
        valid=np.isfinite(rays).all(axis=1)&(ranges>0)
        rows=np.flatnonzero(valid)
        if not len(rows):
            return Query(front,behind,static,unknown,nearest)
        directions=rays[valid]/ranges[valid,None]
        angles=np.arcsin(np.minimum(1.,LATERAL/ranges[valid]))
        neighborhoods=self.tree.query_ball_point(directions,2*np.sin(angles/2),workers=1)
        counts=np.fromiter((len(x) for x in neighborhoods),dtype=int)
        if not counts.sum():
            return Query(front,behind,static,unknown,nearest)
        query_index=np.repeat(rows,counts)
        sample_index=np.concatenate(neighborhoods).astype(int)
        # Original lookup already bounds r_query*sin(theta)<=3cm. Require
        # r_return*sin(theta)<=3cm as well, using the actual recorded range.
        all_directions=np.zeros((size,3))
        all_directions[rows]=directions
        query_direction=all_directions[query_index]
        sample_direction=self.directions[sample_index]
        dot=np.einsum('ij,ij->i',sample_direction,query_direction)
        perpendicular=sample_direction-dot[:,None]*query_direction
        return_lateral=self.ranges[sample_index]*np.linalg.norm(perpendicular,axis=1)
        qualified=(dot>0)&(return_lateral<=LATERAL)
        query_index=query_index[qualified];sample_index=sample_index[qualified]
        if not len(sample_index):
            return Query(front,behind,static,unknown,nearest)
        # Filtering preserves each query's original sorted neighborhood order,
        # including last-equal-depth nearest-return tie handling.
        qualified_counts=np.bincount(query_index,minlength=size)
        observed_rows=np.flatnonzero(qualified_counts)
        starts=np.r_[0,np.cumsum(qualified_counts[observed_rows])[:-1]]
        depth_difference=self.ranges[sample_index]-ranges[query_index]
        exact=np.abs(depth_difference)<=DEPTH
        exact_any=np.zeros(size,dtype=bool)
        exact_any[observed_rows]=np.logical_or.reduceat(exact,starts)
        static[observed_rows]=np.logical_or.reduceat(exact&~self.event[self.indices[sample_index]],starts)
        minimum=np.full(size,np.inf)
        minimum[observed_rows]=np.minimum.reduceat(self.ranges[sample_index],starts)
        static&=~(minimum<ranges-DEPTH)
        first=self.ranges[sample_index]==minimum[query_index]
        last=np.maximum.reduceat(np.where(first,np.arange(len(first)),-1),starts)
        nearest[observed_rows]=self.indices[sample_index[last]]
        observed=np.isfinite(minimum)
        front=observed&~exact_any&(minimum>ranges+DEPTH)
        behind=observed&~exact_any&(minimum<ranges-DEPTH)
        unknown=~(static|front|behind)
        return Query(front,behind,static,unknown,nearest)
