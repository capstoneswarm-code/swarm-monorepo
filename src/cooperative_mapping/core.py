"""ROS-independent grid registration, composition and goal coordination."""
from dataclasses import dataclass
import base64
import hashlib
import json
import math
import zlib

import numpy as np
from scipy import ndimage as ndi

MAX_CELLS = 4_000_000
MAX_BODY = 8_000_000


@dataclass(frozen=True)
class Grid:
    data: np.ndarray
    resolution: float
    x: float
    y: float
    yaw: float = 0.0

    def __post_init__(self):
        if (self.data.ndim != 2 or not self.data.size or self.data.size > MAX_CELLS
                or not all(math.isfinite(v) for v in (self.resolution, self.x, self.y, self.yaw))
                or not 0.01 <= self.resolution <= 0.25
                or np.any((self.data < -1) | (self.data > 100))):
            raise ValueError("Invalid occupancy grid")

    @property
    def geometry(self):
        return self.resolution, self.x, self.y, self.yaw

    def points(self, rows, cols):
        x, y = (np.asarray(cols)+0.5)*self.resolution, (np.asarray(rows)+0.5)*self.resolution
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.column_stack((self.x+c*x-s*y, self.y+s*x+c*y))

    def cells(self, points):
        dx, dy = points[:, 0]-self.x, points[:, 1]-self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.floor(np.column_stack(((-s*dx+c*dy), (c*dx+s*dy)))/self.resolution).astype(int)

    def packet(self):
        a = np.ascontiguousarray(self.data, dtype=np.int8)
        meta = dict(shape=list(a.shape), geometry=list(self.geometry))
        raw = a.tobytes()
        key = hashlib.sha256(json.dumps(meta, sort_keys=True).encode()+raw).hexdigest()
        return dict(**meta, revision=key, data=base64.b64encode(zlib.compress(raw, 3)).decode())

    @classmethod
    def unpack(cls, packet):
        h, w = packet['shape']
        if type(h) is not int or type(w) is not int or min(h, w) < 1 or h*w > MAX_CELLS:
            raise ValueError('Oversized grid')
        packed = base64.b64decode(packet['data'], validate=True)
        if len(packed) > MAX_BODY:
            raise ValueError('Oversized compressed grid')
        stream = zlib.decompressobj()
        raw = stream.decompress(packed, h*w+1)
        if len(raw) != h*w or not stream.eof or stream.unused_data or stream.unconsumed_tail:
            raise ValueError('Invalid compressed grid length')
        grid = cls(np.frombuffer(raw, dtype=np.int8).reshape(h, w), *packet['geometry'])
        if grid.packet()['revision'] != packet['revision']:
            raise ValueError('Grid checksum mismatch')
        return grid


def transform(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(points, dtype=float)
    return p @ np.array([[c, s], [-s, c]]) + [x, y]


def inverse(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c*x-s*y, s*x-c*y, -yaw)


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def near_transform(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1]) < 0.25 and abs(angle_delta(a[2], b[2])) < 0.10


def rigid_fit(source, target):
    ac, bc = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-ac).T @ (target-bc))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    t = bc-r@ac
    return float(t[0]), float(t[1]), math.atan2(r[1, 0], r[0, 0])


def agreement(target, source, pose):
    """Check occupied agreement in BOTH directions, plus known-space overlap."""
    def direction(a, b, p):
        rows, cols = np.nonzero(b.data >= 0)
        step = max(1, len(rows)//40_000)
        rows, cols = rows[::step], cols[::step]
        cells = a.cells(transform(b.points(rows, cols), p))
        valid = ((cells[:, 0] >= 0) & (cells[:, 1] >= 0)
                 & (cells[:, 0] < a.data.shape[0]) & (cells[:, 1] < a.data.shape[1]))
        j = np.flatnonzero(valid)
        ar, ac = cells[j, 0], cells[j, 1]
        known = a.data[ar, ac] >= 0
        overlap = int(known.sum())
        occ = known & (b.data[rows[j], cols[j]] >= 65)
        if overlap < 800 or occ.sum() < 60:
            return None
        distance = ndi.distance_transform_edt(a.data < 65)*a.resolution
        support = float((distance[ar[occ], ac[occ]] <= 0.18).mean())
        return support, overlap/max(1, len(rows)), overlap
    forward = direction(target, source, pose)
    backward = direction(source, target, inverse(pose))
    if forward is None or backward is None:
        return None
    support = min(forward[0], backward[0])
    overlap = min(forward[1], backward[1])
    if support < 0.72 or overlap < 0.12:
        return None
    return dict(wall_agreement=support, overlap_fraction=overlap,
                overlap_cells=min(forward[2], backward[2]))


def register(target, source):
    """OpenCV SIFT + mutual matches + RANSAC + rigid SE(2) + grid validation.

    Returns T_target_from_source, or refuses an ambiguous/weak alignment.
    No guarantee of uniqueness in repetitive buildings; inspect the result.
    """
    import cv2
    cv2.setNumThreads(1)

    def features(grid):
        h, w = grid.data.shape
        scale = min(1.0, 1200/max(h, w))
        size = (max(1, round(w*scale)), max(1, round(h*scale)))
        image = np.where(grid.data < 0, 127, np.where(grid.data >= 65, 0, 255)).astype('uint8')
        known = (grid.data >= 0).astype('uint8')*255
        image = cv2.resize(image, size, interpolation=cv2.INTER_NEAREST)
        known = cv2.resize(known, size, interpolation=cv2.INTER_NEAREST)
        # Avoid using the artificial known/unknown outline as map features.
        known = cv2.erode(known, np.ones((9, 9), np.uint8))
        key, desc = cv2.SIFT_create(nfeatures=3500, contrastThreshold=0.025).detectAndCompute(image, known)
        if desc is None or len(key) < 20:
            return None
        p = np.asarray([k.pt for k in key])
        # Resize pixels -> original grid cells -> metric map coordinates.
        rows = (p[:, 1]+0.5)*h/size[1]-0.5
        cols = (p[:, 0]+0.5)*w/size[0]-0.5
        return grid.points(rows, cols), desc

    af, bf = features(target), features(source)
    if af is None or bf is None:
        return None
    ap, ad = af
    bp, bd = bf
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    def pairs(a, b):
        return {p[0].queryIdx: p[0].trainIdx for p in matcher.knnMatch(a, b, k=2)
                if len(p) == 2 and p[0].distance < 0.70*p[1].distance}
    ba, ab = pairs(bd, ad), pairs(ad, bd)
    matches = [(b, a) for b, a in ba.items() if ab.get(a) == b]
    if len(matches) < 20:
        return None
    b, a = np.asarray(matches).T
    matrix, mask = cv2.estimateAffinePartial2D(
        bp[b], ap[a], method=cv2.RANSAC, ransacReprojThreshold=0.15,
        maxIters=4000, confidence=0.999, refineIters=10)
    if matrix is None or mask is None:
        return None
    scale = math.hypot(matrix[0, 0], matrix[1, 0])
    good = mask.ravel().astype(bool)
    if not 0.97 <= scale <= 1.03 or good.sum() < 18 or good.mean() < 0.45:
        return None
    src, dst = bp[b[good]], ap[a[good]]
    spread = np.linalg.eigvalsh(np.cov(src.T))
    if spread[0] < 0.04 or np.linalg.norm(np.ptp(src, axis=0)) < 2.0:
        return None
    pose = rigid_fit(src, dst)
    residual = np.linalg.norm(transform(src, pose)-dst, axis=1)
    if np.median(residual) > 0.08 or np.quantile(residual, 0.90) > 0.18:
        return None
    quality = agreement(target, source, pose)
    if quality is None:
        return None
    quality.update(inliers=int(good.sum()), matches=len(matches), median_error_m=float(np.median(residual)))
    return pose, quality


def merge(target, source, pose):
    """Recompose from two RAW snapshots; never accumulate merged grids.

    Known beats unknown; occupied beats free. Conflicting probabilities use max.
    Output is axis-aligned in target map coordinates, not a renamed peer frame.
    """
    res = max(target.resolution, source.resolution)
    def corners(g):
        h, w = g.data.shape
        return g.points(np.array([-0.5, -0.5, h-0.5, h-0.5]),
                        np.array([-0.5, w-0.5, -0.5, w-0.5]))
    bounds = np.vstack([corners(target), transform(corners(source), pose)])
    lo = np.floor(bounds.min(axis=0)/res)*res
    hi = np.ceil(bounds.max(axis=0)/res)*res
    w, h = np.maximum(1, np.rint((hi-lo)/res).astype(int))
    if h*w > MAX_CELLS:
        raise ValueError('Merged map too large; alignment may be wrong')
    output = np.full((h, w), -1, dtype=np.int8)
    # Inverse sampling fills rotated cells without forward-splat holes.
    for first in range(0, h, 128):
        rr, cc = np.mgrid[first:min(h, first+128), :w]
        xy = np.column_stack((lo[0]+(cc.ravel()+0.5)*res, lo[1]+(rr.ravel()+0.5)*res))
        block = np.full(len(xy), -1, dtype=np.int8)
        for g, p in ((target, (0, 0, 0)), (source, inverse(pose))):
            cells = g.cells(transform(xy, p))
            valid = ((cells[:, 0] >= 0) & (cells[:, 1] >= 0)
                     & (cells[:, 0] < g.data.shape[0]) & (cells[:, 1] < g.data.shape[1]))
            j = np.flatnonzero(valid)
            block[j] = np.maximum(block[j], g.data[cells[j, 0], cells[j, 1]])
        output[first:first+rr.shape[0]] = block.reshape(rr.shape)
    return Grid(output, float(res), float(lo[0]), float(lo[1]))


def separation_mask(mask, geometry, peer_goal, distance=3.0):
    out = mask.copy()
    rows, cols = np.nonzero(out)
    if peer_goal is None or not len(rows):
        return out
    grid = Grid(np.zeros((1, 1), dtype=np.int8), *geometry)
    p = grid.points(rows, cols)
    keep = np.sum((p-np.asarray(peer_goal[:2]))**2, axis=1) >= distance**2
    out[rows[~keep], cols[~keep]] = False
    return out


def must_yield(robot, peer, goal, peer_goal, distance=3.0):
    return (robot > peer and goal is not None and peer_goal is not None
            and math.hypot(goal[0]-peer_goal[0], goal[1]-peer_goal[1]) < distance)
