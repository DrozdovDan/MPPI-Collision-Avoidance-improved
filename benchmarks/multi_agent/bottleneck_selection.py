"""Physical bottleneck widths and reproducible width-proportional route draws.

Width is the full transverse free chord through the physical reference path,
before keep-right displacement. It includes both sides and does not subtract
the robot radius. Measuring clearance of the displaced lane would penalize a
wide corridor merely because its lane is close to one wall.
"""

import numpy as np
import shapely


def _cross(a,b):
    return a[...,0]*b[...,1]-a[...,1]*b[...,0]


class CorridorWidths:
    """Minimum wall-to-wall transverse width along polygonal reference paths.

    On one straight reference chord, boundary intersections vary affinely
    between projections of polygon vertices. Checking those events, both
    endpoints and interval interiors covers changes in the first wall hit.
    Chords shared by graph routes are cached, including reverse traversal.
    """
    def __init__(self,world):
        self.bounds=np.asarray(world['bounds'],dtype=float)
        self.obstacles=shapely.union_all([shapely.Polygon(p) for p in world['obstacles']])
        self.free=shapely.box(*self.bounds).difference(self.obstacles)
        if not self.free.is_valid:raise ValueError('Invalid physical free-space geometry')
        self.boundary=shapely.boundary(self.free)
        rings=[np.asarray(shapely.get_coordinates(p)) for p in shapely.get_parts(self.boundary)]
        starts=np.concatenate([p[:-1] for p in rings])
        ends=np.concatenate([p[1:] for p in rings])
        valid=np.linalg.norm(ends-starts,axis=1)>1e-12
        self.starts=starts[valid];self.edges=(ends-starts)[valid]
        self.cache={}

    def chord(self,a,b):
        a,b=tuple(map(float,a)),tuple(map(float,b))
        if b<a:a,b=b,a
        key=a+b
        if key in self.cache:return self.cache[key]
        a=np.asarray(a);b=np.asarray(b);length=float(np.linalg.norm(b-a))
        if length<=1e-10:return None
        tangent=(b-a)/length;normal=np.array([tangent[1],-tangent[0]])
        projected=(self.starts-a)@tangent
        events=np.unique(np.r_[0.,projected[(projected>0)&(projected<length)],length])
        stations=np.unique(np.r_[events,(events[:-1]+events[1:])*.5])
        denominator=_cross(normal,self.edges)
        valid=np.abs(denominator)>1e-12
        edges=self.edges[valid];denominator=denominator[valid];relative=self.starts[valid]-a
        best=None
        for first in range(0,len(stations),128):
            s=stations[first:first+128,None]
            ray=(_cross(relative,edges)[None]-s*_cross(tangent,edges)[None])/denominator[None]
            parameter=(_cross(relative,normal)[None]-s*_cross(tangent,normal))/denominator[None]
            hits=(parameter>=-1e-10)&(parameter<=1+1e-10)
            right=np.min(np.where(hits&(ray>=0),ray,np.inf),axis=1)
            left=np.min(np.where(hits&(ray<=0),-ray,np.inf),axis=1)
            widths=right+left;j=int(np.argmin(widths));width=float(widths[j])
            if not np.isfinite(widths).all() or np.any(widths<=1e-9):
                raise ValueError('Reference path has no positive bounded free cross-section')
            if best is None or width<best['width']:
                point=a+float(s[j,0])*tangent
                best=dict(width=width,point=point.tolist(),normal=normal.tolist(),
                    right_distance=float(right[j]),left_distance=float(left[j]),
                    reference_chord=[a.tolist(),b.tolist()])
        self.cache[key]=best
        return best

    def route(self,route):
        reference=np.asarray(route['physical_path'],dtype=float)
        if reference.ndim!=2 or reference.shape[1]!=2 or not np.isfinite(reference).all():
            raise ValueError('Missing or invalid physical reference path')
        candidates=[self.chord(a,b) for a,b in zip(reference[:-1],reference[1:])]
        candidates=[x for x in candidates if x is not None]
        if not candidates:raise ValueError('Degenerate physical reference path')
        return dict(min(candidates,key=lambda x:x['width']))

    def check_witness(self,witness):
        """Independent GEOS line/boundary intersections at the reported neck."""
        point=np.asarray(witness['point']);normal=np.asarray(witness['normal'])
        extent=2*np.linalg.norm(self.bounds[2:]-self.bounds[:2])+1.
        line=shapely.LineString([point-extent*normal,point+extent*normal])
        # Exact vertex hits can be lost when constructing long ray endpoints
        # introduces roundoff. Snap within a nanometre to retain those contacts.
        line=shapely.snap(line,self.boundary,1e-9)
        coordinates=shapely.get_coordinates(shapely.intersection(self.boundary,line))
        signed=(coordinates-point)@normal
        if not np.any(signed>1e-8) or not np.any(signed< -1e-8):
            raise ValueError('Bottleneck witness does not have walls on both sides')
        measured=float(signed[signed>1e-8].min()-signed[signed< -1e-8].max())
        if not np.isclose(measured,witness['width'],atol=2e-7,rtol=1e-8):
            raise ValueError(f'Independent bottleneck witness check failed: measured={measured}, witness={witness}')
        return measured


def proportional_probabilities(widths):
    widths=np.asarray(widths,dtype=float)
    if widths.ndim!=1 or not len(widths) or not np.isfinite(widths).all() or np.any(widths<=0):
        raise ValueError('Every route needs a positive finite bottleneck width')
    scaled=widths/widths.max()
    return scaled/scaled.sum()


def draw_width_route(widths,uniform_index,jitter):
    """Inverse-CDF draw coupled to the existing uniform benchmark.

    The uniform route index picks an equal-sized bin; an independent uniform
    jitter picks a point inside it. Their combination is uniform on [0,1), so
    inverse-CDF sampling has exactly the requested width probabilities. Equal
    widths reproduce the original uniform route index, reducing comparison
    noise without changing either policy's marginal distribution.
    """
    probabilities=proportional_probabilities(widths);n=len(probabilities)
    if not 0<=uniform_index<n or not 0<=jitter<1:raise ValueError('Invalid sampling variate')
    uniform=(int(uniform_index)+float(jitter))/n
    cumulative=np.cumsum(probabilities);cumulative[-1]=1.
    selected=int(np.searchsorted(cumulative,uniform,side='right'))
    if np.all(np.asarray(widths)==widths[0]):selected=int(uniform_index)
    return selected,probabilities,uniform


def audit_lane_geometry(route,radius,margin):
    counts=dict(split_sections=0,shared_corridor_sections=0,transition_sections=0,
        minimum_right_offset=None,maximum_shared_deviation=0.)
    for section in route['segments']:
        state=section['state'];geometry=np.asarray(section['geometry']);reference=np.asarray(section['reference_geometry'])
        if state=='TRANSITION':counts['transition_sections']+=1
        if state=='SPLIT':
            expected='RightOneWay' if section['forward'] else 'LeftOneWay'
            if section['mode']!=expected or geometry.shape!=reference.shape:
                raise ValueError('Incorrect canonical mode or missing reference for split lane')
            delta=np.diff(reference,axis=0);length=np.linalg.norm(delta,axis=1);valid=length>1e-10
            tangent=delta[valid]/length[valid,None]
            a=geometry[:-1][valid]-reference[:-1][valid]
            b=geometry[1:][valid]-reference[1:][valid]
            offsets=np.r_[_cross(a,tangent),_cross(b,tangent)]
            if not len(offsets) or np.min(offsets)<=1e-8:
                raise ValueError('Split anchor crosses onto the travel-left side or center')
            opposite=shapely.LineString(section['opposing_geometry'])
            if shapely.LineString(geometry).distance(opposite)<2*(radius+margin)-1e-7:
                raise ValueError('Opposing certified lanes have insufficient footprint separation')
            counts['split_sections']+=1;minimum=float(offsets.min())
            counts['minimum_right_offset']=minimum if counts['minimum_right_offset'] is None else min(minimum,counts['minimum_right_offset'])
        elif state=='SHARED' and section['kind']=='corridor':
            if geometry.shape!=reference.shape:raise ValueError('Shared corridor has no matching center reference')
            error=float(np.max(np.linalg.norm(geometry-reference,axis=1)))
            if error>1e-7:raise ValueError('Shared corridor anchor differs from its center reference')
            counts['shared_corridor_sections']+=1;counts['maximum_shared_deviation']=max(error,counts['maximum_shared_deviation'])
    return counts
