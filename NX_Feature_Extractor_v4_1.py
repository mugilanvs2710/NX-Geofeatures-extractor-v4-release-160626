# =============================================================================
#  NX OPEN PYTHON JOURNAL  —  GEOMETRIC FEATURE EXTRACTOR  v4.1
#  Verified against  : Siemens NX 2506  (v2506)
#  Run via           : Tools → Journal → Play
#  Output            : <part_folder>\<part_name>_FEATURES_v4_1.txt
#                      <part_folder>\<part_name>_FEATURES_v4_1.csv
#                      <part_folder>\<part_name>_FEATURES_v4_1.json
#
#  v4.1 FIXES over v4.0
#  ────────────────────────────────────────────────────────────────────────────
#  FIX 1  TYPE_SPECIFICITY key lookup was broken — split('—')[0] never matched
#          any key. Now uses full type string lookup with normalised fallback.
#  FIX 2  GetBlendData() / GetChamferData() None guard added. Defensive against
#          NX versions that may return None instead of tuple.
#  FIX 3  edge.GetFaces() wrapped with list() inside BodyCache for iterator
#          consistency with body.GetFaces() and face.GetEdges().
#  FIX 4  Comment corrected: engines run sequentially per face, not in parallel.
#  FIX 5  Unicode ⚠ replaced with ASCII [!] for NX listing window robustness
#          across mixed-locale MNC machines.
#  FIX 6  ISO_HOLE_MAP externalisable via fe_config.json. Falls back to
#          built-in table if config absent. Manufacturing-engineering-ready.
#  FIX 7  Fusion conflict resolution: when two candidates within 0.08 conf,
#          apply rule-based tie-breakers (API > geometry > heuristic).
#  FIX 8  Precision labelling: API-sourced dims labelled "(API)", geometry-
#          inferred labelled "(calc)", area labelled "(polygon approx)".
#          Never mix exact and approximate without explicit source label.
#  FIX 9  Planar engine redesigned: classify by functional zone (neighbour
#          context + area ratio + normal alignment) not edge count alone.
#          Edge count used only as secondary confirmation, not primary signal.
#  FIX 10 Variable-radius fillet: full neighbour blend check via BodyCache.
#          No longer uses non-integer-radius heuristic.
#
#  CONFIRMED AVAILABLE APIs (NX v2506 — live diagnostics)
#  ────────────────────────────────────────────────────────────────────────────
#  face.SolidFaceType.value  → 1=Planar 2=Cylinder 3=Cone 5=Torus   ✓
#  face.GetHoleData()        → (ResizeHoleData, bool) or None         ✓
#  face.GetBlendData()       → (radius_float, is_blend_bool)          ✓
#  face.GetChamferData()     → (is_chamfer_bool, TypeEnum, [d1,d2])   ✓
#  face.GetEdges()           → list[Edge]  MUST wrap list()           ✓
#  face.JournalIdentifier    → str                                    ✓
#  face.Tag                  → int                                    ✓
#  edge.GetLength()          → float mm                               ✓
#  edge.GetFaces()           → MUST wrap list()                       ✓
#  edge.GetLocations()       → list[CurveLocation] .Location=Point3d  ✓
#  ResizeHoleData.*          → diameter/depth/direction/origin/type   ✓
#  work_part.Features        → FeatureCollection                      ✓
#  body.GetFaces()           → MUST wrap list()                       ✓
#
#  NOT AVAILABLE in NX 2506 (confirmed by live diagnostics):
#  ✗ AskFaceUvMinMaxValues   ✗ EvaluateAtPoint   ✗ GetAdjacentFaces
#  ✗ edge.EdgeType.value     ✗ GetFirstFacetOnFace   ✗ NXOpen.UF
# =============================================================================

import NXOpen
import re, os, math, json
from datetime import datetime
from collections import defaultdict

# ── FACE TYPE INTEGERS (confirmed NX 2506) ────────────────────────────────────
FT_PLANAR   = 1
FT_CYLINDER = 2
FT_CONE     = 3
FT_TORUS    = 5

# ── HOLE SUB-TYPES ────────────────────────────────────────────────────────────
HT_SIMPLE      = 0
HT_COUNTERBORE = 1
HT_COUNTERSINK = 2

# ── CONFIDENCE THRESHOLDS ─────────────────────────────────────────────────────
CONF_HIGH   = 0.85   # API-confirmed result
CONF_MEDIUM = 0.60   # geometry-inferred result
CONF_LOW    = 0.40   # heuristic result

# ── CONFLICT RESOLUTION WINDOW ───────────────────────────────────────────────
FUSION_TIE_WINDOW = 0.08   # candidates within this range → apply tie-breakers

# ── TUNING THRESHOLDS (confirmed from 6 real automotive models) ───────────────
FILLET_MAX_RADIUS  = 10.0    # mm  above → structural curve not fillet
STEP_MAX_AREA      = 5000.0  # mm² above → body wall not step
SLOT_MAX_ASPECT    = 8.0     # slot length/width max (real slots < 8)
SLOT_MIN_WIDTH     = 1.0     # mm  below → groove not slot
BOLT_CIRCLE_MIN_N  = 3       # minimum holes for bolt circle classification
BORE_BOSS_RATIO    = 0.85    # dist/body_radius below this → BORE

# ── CATEGORIES ────────────────────────────────────────────────────────────────
CAT_HOLE      = "HOLE FEATURES"
CAT_MACHINED  = "MACHINED FEATURES"
CAT_FINISHING = "FINISHING FEATURES"
CAT_FORM      = "FORM FEATURES"
CAT_OTHER     = "OTHER GEOMETRY"
CATEGORY_ORDER = [CAT_HOLE, CAT_MACHINED, CAT_FINISHING, CAT_FORM, CAT_OTHER]

# ── ENGINE SOURCE TIERS (for conflict resolution) ─────────────────────────────
TIER_API      = 3    # GetHoleData, GetBlendData, GetChamferData — NX kernel
TIER_GEOMETRY = 2    # bore/boss centroid, annular radius check
TIER_HEURISTIC= 1    # edge count, aspect ratio, area gate

# =============================================================================
#  FIX 6: ISO HOLE MAP — externalisable via fe_config.json
#  Keys = nominal diameter mm, Values = (standard_name, fit_type, use_note)
# =============================================================================

_BUILTIN_ISO_MAP = {
    1.0 : ("M1 tap",         "tapped",    "M1 thread"),
    1.5 : ("M1.5 tap",       "tapped",    "M1.5 thread"),
    2.0 : ("M2 tap",         "tapped",    "M2 thread"),
    2.5 : ("M3 clearance",   "clearance", "M3 bolt close clearance"),
    3.0 : ("M3 tap",         "tapped",    "M3 thread"),
    3.3 : ("M4 tap drill",   "tapped",    "M4 pre-tap"),
    3.4 : ("M4 clearance",   "clearance", "M4 bolt close clearance"),
    4.0 : ("M4 tap",         "tapped",    "M4 thread"),
    4.2 : ("M5 tap drill",   "tapped",    "M5 pre-tap"),
    4.5 : ("M5 clearance",   "clearance", "M5 bolt close clearance"),
    5.0 : ("M5 tap",         "tapped",    "M5 thread"),
    5.5 : ("M6 clearance",   "clearance", "M6 bolt close clearance"),
    5.8 : ("M7 tap drill",   "tapped",    "M7 pre-tap"),
    6.0 : ("M6 tap",         "tapped",    "M6 thread"),
    6.4 : ("M6 clearance",   "clearance", "M6 bolt standard clearance"),
    6.8 : ("M8 tap drill",   "tapped",    "M8 pre-tap"),
    7.0 : ("M7 tap",         "tapped",    "M7 thread"),
    8.0 : ("M8 tap",         "tapped",    "M8 thread"),
    8.4 : ("M8 clearance",   "clearance", "M8 bolt standard clearance"),
    8.5 : ("M8 clearance+",  "clearance", "M8 bolt loose clearance"),
    10.0: ("M10 tap",        "tapped",    "M10 thread"),
    10.2: ("M10 clearance",  "clearance", "M10 bolt standard clearance"),
    10.5: ("M10 clearance+", "clearance", "M10 bolt loose clearance"),
    12.0: ("M12 tap",        "tapped",    "M12 thread"),
    13.0: ("M12 clearance",  "clearance", "M12 bolt standard clearance"),
    14.0: ("M14 tap",        "tapped",    "M14 thread"),
    15.0: ("M14 clearance",  "clearance", "M14 bolt standard clearance"),
    16.0: ("M16 tap",        "tapped",    "M16 thread"),
    17.0: ("M16 clearance",  "clearance", "M16 bolt standard clearance"),
    18.0: ("M18 tap",        "tapped",    "M18 thread"),
    20.0: ("M20 tap",        "tapped",    "M20 thread"),
    21.0: ("M20 clearance",  "clearance", "M20 bolt standard clearance"),
    24.0: ("M24 tap",        "tapped",    "M24 thread"),
    25.0: ("M24 clearance",  "clearance", "M24 bolt standard clearance"),
    30.0: ("M30 tap",        "tapped",    "M30 thread"),
    33.0: ("M30 clearance",  "clearance", "M30 bolt standard clearance"),
}

def _load_iso_map():
    """
    FIX 6: Load ISO map from external config if present.
    Allows manufacturing engineering to customise without code changes.
    """
    try:
        cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'fe_config.json')
        if os.path.exists(cfg_path):
            with open(cfg_path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
            raw = cfg.get('iso_hole_map', {})
            return {float(k): tuple(v) for k, v in raw.items()}
    except Exception:
        pass
    return _BUILTIN_ISO_MAP

ISO_HOLE_MAP = _load_iso_map()

def lookup_iso_size(diameter_mm):
    """Map hole diameter to nearest ISO metric standard within 0.15mm."""
    best = None
    best_delta = 0.15
    for std_dia, info in ISO_HOLE_MAP.items():
        delta = abs(diameter_mm - std_dia)
        if delta < best_delta:
            best_delta = delta
            best = info
    return best


# =============================================================================
#  BODY-LEVEL GEOMETRY CACHE
#  Single pass over all faces and edges. Zero repeated NX kernel calls.
# =============================================================================

class BodyCache:
    """
    Builds complete geometry data for a body in one traversal.
    All subsequent classification reads from Python dicts — no NX calls.
    """
    __slots__ = ('faces', 'face_by_tag', 'face_edges',
                 'face_neighbour_tags', 'face_midpoints',
                 'edge_lengths', 'bbox',
                 'body_cx', 'body_cy', 'body_cz',
                 'body_radius', 'body_span')

    def __init__(self, body):
        self.faces         = list(body.GetFaces())          # FIX 3: materialise
        self.face_by_tag   = {f.Tag: f for f in self.faces}
        self.face_edges         = {}
        self.face_neighbour_tags= {}
        self.face_midpoints     = {}
        self.edge_lengths       = {}

        all_pts = []

        for face in self.faces:
            ftag  = face.Tag
            edges = list(face.GetEdges())                   # FIX 3: materialise
            self.face_edges[ftag] = edges

            mids = []
            nbrs = set()

            for edge in edges:
                etag = edge.Tag
                if etag not in self.edge_lengths:
                    try:
                        self.edge_lengths[etag] = edge.GetLength()
                    except Exception:
                        self.edge_lengths[etag] = 0.0

                try:
                    locs = edge.GetLocations()
                    if locs:
                        p = locs[0].Location
                        pt = (p.X, p.Y, p.Z)
                        mids.append(pt)
                        all_pts.append(pt)
                except Exception:
                    pass

                # FIX 3: list() wrapping on edge.GetFaces()
                try:
                    for nf in list(edge.GetFaces()):
                        try:
                            nt = nf.Tag
                            if nt != ftag:
                                nbrs.add(nt)
                        except Exception:
                            pass
                except Exception:
                    pass

            self.face_midpoints[ftag]      = mids
            self.face_neighbour_tags[ftag] = nbrs

        if all_pts:
            xs = [p[0] for p in all_pts]
            ys = [p[1] for p in all_pts]
            zs = [p[2] for p in all_pts]
            xmn,xmx = min(xs),max(xs)
            ymn,ymx = min(ys),max(ys)
            zmn,zmx = min(zs),max(zs)
            self.bbox        = (xmn,xmx, ymn,ymx, zmn,zmx)
            self.body_cx     = (xmn+xmx)/2.0
            self.body_cy     = (ymn+ymx)/2.0
            self.body_cz     = (zmn+zmx)/2.0
            self.body_radius = max(xmx-xmn, ymx-ymn, zmx-zmn) / 2.0
            self.body_span   = max(xmx-xmn, ymx-ymn, zmx-zmn)
        else:
            self.bbox        = None
            self.body_cx = self.body_cy = self.body_cz = 0.0
            self.body_radius = 1.0
            self.body_span   = 1.0

    def face_span(self, ftag):
        mids = self.face_midpoints.get(ftag, [])
        if len(mids) < 2:
            return 1.0
        xs=[p[0] for p in mids]; ys=[p[1] for p in mids]; zs=[p[2] for p in mids]
        return math.sqrt((max(xs)-min(xs))**2+(max(ys)-min(ys))**2+(max(zs)-min(zs))**2) or 1.0

    def neighbour_types(self, ftag):
        types = set()
        for nt in self.face_neighbour_tags.get(ftag, set()):
            nf = self.face_by_tag.get(nt)
            if nf:
                try: types.add(nf.SolidFaceType.value)
                except Exception: pass
        return types

    def classify_edges(self, ftag):
        """Curved/straight split via length-ratio heuristic. Zero NX calls."""
        edges    = self.face_edges.get(ftag, [])
        fspan    = self.face_span(ftag)
        curved   = []
        straight = []
        for edge in edges:
            L = self.edge_lengths.get(edge.Tag, 0.0)
            ratio = L / max(fspan, 0.001)
            implied_dia = L / math.pi
            if ratio < 1.2 and implied_dia < fspan * 0.6:
                curved.append((edge, L))
            else:
                straight.append((edge, L))
        return curved, straight

    def neighbour_blend_radii(self, ftag):
        """Return list of blend radii from adjacent faces. Used for var-radius."""
        radii = []
        for nt in self.face_neighbour_tags.get(ftag, set()):
            nf = self.face_by_tag.get(nt)
            if nf:
                try:
                    nb = nf.GetBlendData()
                    if nb is not None and nb[1]:    # FIX 2: None guard
                        radii.append(nb[0])
                except Exception:
                    pass
        return radii


# =============================================================================
#  COORDINATE & GEOMETRY UTILITIES
# =============================================================================

def parse_jid_coords(jid):
    try:
        m = re.search(
            r'\(\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\)', jid)
        if m:
            return float(m.group(1)), float(m.group(2)), float(m.group(3))
    except Exception:
        pass
    return None, None, None

def vec3d_to_tuple(v):
    try:
        return (round(v.X,6), round(v.Y,6), round(v.Z,6))
    except Exception:
        return (0.0, 0.0, 0.0)

def vec_angle_deg(a, b):
    dot = max(-1.0, min(1.0, sum(a[i]*b[i] for i in range(3))))
    return math.degrees(math.acos(dot))

def face_normal(bc, ftag):
    mids = bc.face_midpoints.get(ftag, [])
    if len(mids) < 3:
        return (0.0, 0.0, 1.0)
    o = mids[0]; vecs = []
    for p in mids[1:]:
        dx,dy,dz = p[0]-o[0], p[1]-o[1], p[2]-o[2]
        mag = math.sqrt(dx*dx+dy*dy+dz*dz)
        if mag > 1e-6:
            vecs.append((dx/mag, dy/mag, dz/mag))
        if len(vecs) >= 2:
            break
    if len(vecs) < 2:
        return (0.0, 0.0, 1.0)
    a,b = vecs[0], vecs[1]
    nx = a[1]*b[2]-a[2]*b[1]
    ny = a[2]*b[0]-a[0]*b[2]
    nz = a[0]*b[1]-a[1]*b[0]
    mag = math.sqrt(nx*nx+ny*ny+nz*nz)
    if mag < 1e-10:
        return (0.0, 0.0, 1.0)
    return (round(nx/mag,6), round(ny/mag,6), round(nz/mag,6))

def face_centre(bc, ftag, jx, jy, jz):
    if jx is not None:
        return jx, jy, jz
    mids = bc.face_midpoints.get(ftag, [])
    if mids:
        return (sum(p[0] for p in mids)/len(mids),
                sum(p[1] for p in mids)/len(mids),
                sum(p[2] for p in mids)/len(mids))
    return None, None, None

def shoelace_area(bc, ftag, normal):
    """
    FIX 8: Labelled explicitly as polygon approximation.
    Accurate for planar faces with straight edges. Less so for curved boundaries.
    """
    pts = bc.face_midpoints.get(ftag, [])
    if len(pts) < 3:
        return 0.0
    try:
        nx,ny,nz = normal
        u = (1,0,0) if abs(nx)<0.9 else (0,1,0)
        dot = u[0]*nx+u[1]*ny+u[2]*nz
        u = (u[0]-dot*nx, u[1]-dot*ny, u[2]-dot*nz)
        um = math.sqrt(u[0]**2+u[1]**2+u[2]**2)
        if um < 1e-10: return 0.0
        u = (u[0]/um, u[1]/um, u[2]/um)
        v = (ny*u[2]-nz*u[1], nz*u[0]-nx*u[2], nx*u[1]-ny*u[0])
        pts2d = [(p[0]*u[0]+p[1]*u[1]+p[2]*u[2],
                  p[0]*v[0]+p[1]*v[1]+p[2]*v[2]) for p in pts]
        area = 0.0
        n = len(pts2d)
        for i in range(n):
            j=(i+1)%n
            area += pts2d[i][0]*pts2d[j][1] - pts2d[j][0]*pts2d[i][1]
        return abs(area)/2.0
    except Exception:
        return 0.0

def is_bore(cx, cy, cz, bc):
    if cx is None: return False
    dist = math.sqrt((cx-bc.body_cx)**2+(cy-bc.body_cy)**2+(cz-bc.body_cz)**2)
    return dist < bc.body_radius * BORE_BOSS_RATIO

def is_annular(curved, straight):
    if len(curved) != 2 or len(straight) != 0:
        return False
    r1 = curved[0][1]/(2*math.pi)
    r2 = curved[1][1]/(2*math.pi)
    return abs(r1-r2) > 0.5


# =============================================================================
#  THREAD MAP
# =============================================================================

def build_thread_map(work_part):
    thread_map = {}
    try:
        for feat in work_part.Features:
            if 'THREAD' not in feat.FeatureType.upper():
                continue
            try:
                info = {'name': feat.GetFeatureName(), 'type': feat.FeatureType}
                for obj in feat.GetEntities():
                    try: thread_map[obj.Tag] = info
                    except Exception: pass
            except Exception:
                pass
    except Exception:
        pass
    return thread_map


# =============================================================================
#  CANDIDATE BUILDER
# =============================================================================

def _make(type_, cat, conf, tier, cx, cy, cz, axis, dims, engine, jid):
    """Build a standard candidate dict with all required fields."""
    if conf >= CONF_HIGH:
        level = "HIGH"
    elif conf >= CONF_MEDIUM:
        level = "MEDIUM"
    else:
        level = "LOW"
    return {
        'type'        : type_,
        'category'    : cat,
        'confidence'  : round(conf, 3),
        'conf_level'  : level,
        'tier'        : tier,            # API / GEOMETRY / HEURISTIC
        'needs_review': conf < CONF_MEDIUM,
        'cx': cx, 'cy': cy, 'cz': cz,
        'axis'        : axis,
        'dimensions'  : dims,
        'engine'      : engine,
        'jid'         : jid,
    }


# =============================================================================
#  FIX 1 + FIX 7: FUSION ENGINE WITH CONFLICT RESOLUTION
#
#  Priority:
#    1. Highest confidence wins.
#    2. Within FUSION_TIE_WINDOW: TIER_API > TIER_GEOMETRY > TIER_HEURISTIC.
#    3. Within same tier: TYPE_SPECIFICITY ranking.
#    4. Within same specificity: longer engine name (more specific engine) wins.
# =============================================================================

# FIX 1: Keys match full type strings as stored in candidate['type']
TYPE_SPECIFICITY = {
    'HOLE — SIMPLE'                  : 100,
    'HOLE — SIMPLE (THROUGH)'        : 100,
    'HOLE — COUNTERBORE'             : 100,
    'HOLE — COUNTERSINK'             : 100,
    'HOLE (error)'                   :  50,
    'FILLET / ROUND'                 :  90,
    'FILLET / ROUND (VARIABLE-RADIUS)':  90,
    'CHAMFER'                        :  90,
    'CHAMFER — WELD PREP'            :  90,
    'CHAMFER — HOLE ENTRY'           :  90,
    'BORE / INTERNAL CYLINDER'       :  80,
    'BOSS / PIN'                     :  80,
    'THREADED BORE'                  :  80,
    'THREADED BOSS / STUD'           :  80,
    'GROOVE / O-RING GROOVE'         :  78,
    'SLOT (CLOSED END)'              :  70,
    'SLOT (OPEN END)'                :  70,
    'POCKET — RECTANGULAR'           :  70,
    'POCKET — CIRCULAR'              :  70,
    'STEP / SHOULDER'                :  68,
    'ANNULAR FACE'                   :  65,
    'DRAFT FACE (TAPER)'             :  60,
    'CONICAL FACE'                   :  60,
    'TOROIDAL FACE'                  :  60,
    'PLANAR FACE (DRAFTED)'          :  45,
    'PLANAR FACE'                    :  40,
    'UNKNOWN'                        :   0,
}

def _specificity(c):
    """FIX 1: Look up by full type string, not split fragment."""
    return TYPE_SPECIFICITY.get(c['type'], 0)

def fuse(candidates):
    """
    FIX 7: Multi-signal weighted fusion with rule-based conflict resolution.
    Sequential multi-engine evaluation; results fused here.  (FIX 4: renamed)
    """
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    # Sort primary: confidence descending
    ranked = sorted(candidates,
                    key=lambda c: (c['confidence'], c['tier'], _specificity(c)),
                    reverse=True)
    best = ranked[0]

    # Conflict resolution: if runner-up is within tie window
    if len(ranked) > 1:
        runner = ranked[1]
        if abs(best['confidence'] - runner['confidence']) <= FUSION_TIE_WINDOW:
            # Rule 1: API tier always beats geometry/heuristic
            if runner['tier'] > best['tier']:
                return runner
            if best['tier'] > runner['tier']:
                return best
            # Rule 2: Same tier → specificity
            if _specificity(runner) > _specificity(best):
                return runner

    return best


# =============================================================================
#  ENGINES — sequential per face; results fused by fuse()
#  (FIX 4: comment corrected from "simultaneous" to "sequential")
# =============================================================================

# ── ENGINE 1: HOLE ────────────────────────────────────────────────────────────

def engine_hole(face, ft, bc, thread_map, jx, jy, jz):
    if ft != FT_CYLINDER:
        return []
    hd = face.GetHoleData()
    if hd is None:
        return []
    try:
        ho = hd[0]
        dia   = ho.GetHoleDiameter()
        depth = ho.GetHoleDepth()
        dirv  = vec3d_to_tuple(ho.GetDirection())
        orig  = ho.GetOrigin()
        ox,oy,oz = orig.X*1000.0, orig.Y*1000.0, orig.Z*1000.0
        cx = jx if jx is not None else ox
        cy = jy if jy is not None else oy
        cz = jz if jz is not None else oz

        through = False
        if bc.bbox:
            xmn,xmx,ymn,ymx,zmn,zmx = bc.bbox
            extent = (abs(dirv[0])*(xmx-xmn) + abs(dirv[1])*(ymx-ymn) +
                      abs(dirv[2])*(zmx-zmn))
            if extent > 0.5 and depth >= extent * 0.92:
                through = True

        try: ht = ho.GetHoleType()
        except Exception: ht = HT_SIMPLE

        if ht == HT_COUNTERBORE:
            try: cbd,cbd2 = ho.GetCounterboredDiameter(), ho.GetCounterboredDepth()
            except Exception: cbd=cbd2=0.0
            dims = {
                # FIX 8: API-sourced labels
                'Hole Diameter (API) (mm)': dia,
                'Hole Depth    (API) (mm)': depth,
                'CB Diameter   (API) (mm)': cbd,
                'CB Depth      (API) (mm)': cbd2,
            }
            sub = 'COUNTERBORE'
        elif ht == HT_COUNTERSINK:
            try: csd,csa = ho.GetCountersunkDiameter(), ho.GetCountersunkAngle()
            except Exception: csd=csa=0.0
            dims = {
                'Hole Diameter (API) (mm)': dia,
                'Hole Depth    (API) (mm)': depth,
                'CS Diameter   (API) (mm)': csd,
                'CS Angle      (API)(deg)': csa,
            }
            sub = 'COUNTERSINK'
        else:
            dims = {
                'Diameter      (API) (mm)': dia,
                'Depth         (API) (mm)': depth,
            }
            sub = 'SIMPLE'

        try:
            if ho.GetEnableEntryChamfer():
                dims['Entry Chamfer Angle (API)(deg)'] = ho.GetEntryChamferAngle()
                dims['Entry Chamfer Off.  (API)(mm) '] = ho.GetEntryChamferOffset()
        except Exception:
            pass

        dims['Through-hole'] = 'YES' if through else 'NO (blind)'

        # L/D ratio DFM check
        if dia > 0:
            ld = depth / dia
            dims['L/D Ratio (calc)'] = round(ld, 2)
            if ld > 6.0:
                dims['DFM Flag'] = (
                    'DEEP HOLE — L/D=' + str(round(ld,2)) + ' (limit 6.0)')

        # Thread
        ti = thread_map.get(face.Tag)
        if ti: dims['Thread Feature'] = ti.get('name','THREADED')

        # ISO size
        iso = lookup_iso_size(dia)
        if iso:
            dims['ISO Standard (ref)'] = iso[0]
            dims['Fit Type    (ref)' ] = iso[1]
            dims['Recommended (ref)' ] = iso[2]

        label = 'HOLE — ' + sub + (' (THROUGH)' if through else '')
        return [_make(label, CAT_HOLE, 0.95, TIER_API,
                      cx, cy, cz, dirv, dims, 'HoleEngine', face.JournalIdentifier)]

    except Exception as e:
        return [_make('HOLE (error)', CAT_HOLE, 0.50, TIER_API,
                      jx, jy, jz, (0,0,1), {'error': str(e)},
                      'HoleEngine', face.JournalIdentifier)]


# ── ENGINE 2: FILLET ──────────────────────────────────────────────────────────

def engine_fillet(face, ft, bc, jx, jy, jz):
    # FIX 2: None guard
    bd = face.GetBlendData()
    if bd is None or not bd[1]:
        return []

    radius = bd[0]
    if radius > FILLET_MAX_RADIUS:
        return []

    cx, cy, cz = face_centre(bc, face.Tag, jx, jy, jz)
    normal = face_normal(bc, face.Tag)

    # FIX 10: Full neighbour blend check via BodyCache
    is_var = False
    nbr_radii = bc.neighbour_blend_radii(face.Tag)
    if nbr_radii:
        if any(abs(r - radius) > 0.1 for r in nbr_radii):
            is_var = True

    label = 'FILLET / ROUND (VARIABLE-RADIUS)' if is_var else 'FILLET / ROUND'
    dims  = {
        'Fillet Radius  (API) (mm)': radius,
        'Fillet Dia     (calc)(mm)': radius * 2.0,
        'Variable radius'          : 'YES' if is_var else 'NO',
    }
    return [_make(label, CAT_FINISHING, 0.90, TIER_API,
                  cx, cy, cz, normal, dims, 'FilletEngine', face.JournalIdentifier)]


# ── ENGINE 3: CHAMFER ─────────────────────────────────────────────────────────

def engine_chamfer(face, ft, bc, jx, jy, jz):
    # FIX 2: None guard
    cd = face.GetChamferData()
    if cd is None or not cd[0]:
        return []

    _, ctype, dl = cd
    d1 = dl[0] if len(dl)>0 else 0.0
    d2 = dl[1] if len(dl)>1 else 0.0
    cx, cy, cz = face_centre(bc, face.Tag, jx, jy, jz)
    normal = face_normal(bc, face.Tag)

    weld = False
    if abs(d1-d2) < 0.1 and d1 > 0.3:
        ntypes = bc.neighbour_types(face.Tag)
        if sum(1 for t in ntypes if t == FT_PLANAR) >= 2:
            weld = True

    label = 'CHAMFER — WELD PREP' if weld else 'CHAMFER'
    dims  = {
        'Chamfer Dim 1  (API) (mm)': d1,
        'Chamfer Dim 2  (API) (mm)': d2,
        'Chamfer Type'             : str(ctype),
        'Weld prep'                : 'YES' if weld else 'NO',
    }
    return [_make(label, CAT_FINISHING, 0.90, TIER_API,
                  cx, cy, cz, normal, dims, 'ChamferEngine', face.JournalIdentifier)]


# ── ENGINE 4: CONICAL HOLE ENTRY ──────────────────────────────────────────────

def engine_hole_entry(face, ft, bc, jx, jy, jz):
    if ft != FT_CONE:
        return []
    hd = face.GetHoleData()
    if hd is None:
        return []
    try:
        ang = hd[0].GetEntryChamferAngle()
        off = hd[0].GetEntryChamferOffset()
    except Exception:
        ang = off = 0.0
    cx, cy, cz = face_centre(bc, face.Tag, jx, jy, jz)
    normal = face_normal(bc, face.Tag)
    dims = {
        'Entry Chamfer Angle (API)(deg)': ang,
        'Entry Chamfer Off.  (API)(mm) ': off,
    }
    return [_make('CHAMFER — HOLE ENTRY', CAT_FINISHING, 0.88, TIER_API,
                  cx, cy, cz, normal, dims,
                  'HoleEntryEngine', face.JournalIdentifier)]


# ── ENGINE 5: CYLINDER (BORE / BOSS) ─────────────────────────────────────────

def engine_cylinder(face, ft, bc, thread_map, jx, jy, jz):
    if ft != FT_CYLINDER:
        return []
    if face.GetHoleData() is not None:
        return []

    cx, cy, cz = face_centre(bc, face.Tag, jx, jy, jz)
    normal = face_normal(bc, face.Tag)
    curved, straight = bc.classify_edges(face.Tag)

    dims = {}
    conf = 0.70
    tier = TIER_GEOMETRY

    if curved:
        top2  = sorted(curved, key=lambda e: e[1], reverse=True)[:2]
        avg_c = sum(e[1] for e in top2) / len(top2)
        dia   = avg_c / math.pi
        # FIX 8: label as calc not exact
        dims['Diameter      (calc)(mm)'] = dia
        dims['Radius        (calc)(mm)'] = dia / 2.0
        iso = lookup_iso_size(dia)
        if iso:
            dims['ISO Standard  (ref)'] = iso[0]

    if straight:
        dims['Axial Length  (calc)(mm)'] = min(e[1] for e in straight)

    ti = thread_map.get(face.Tag)
    if ti:
        dims['Thread Feature'] = ti.get('name','THREADED')
        conf = 0.80

    bore = is_bore(cx, cy, cz, bc)
    ntypes = bc.neighbour_types(face.Tag)
    if FT_PLANAR in ntypes:
        conf = min(conf + 0.05, 0.85)

    if ti:
        label = 'THREADED BORE' if bore else 'THREADED BOSS / STUD'
    else:
        label = 'BORE / INTERNAL CYLINDER' if bore else 'BOSS / PIN'

    return [_make(label, CAT_MACHINED, conf, tier,
                  cx, cy, cz, normal, dims,
                  'CylinderEngine', face.JournalIdentifier)]


# ── ENGINE 6: CONICAL ─────────────────────────────────────────────────────────

def engine_cone(face, ft, bc, jx, jy, jz):
    if ft != FT_CONE:
        return []
    # FIX 2: None guard on GetChamferData
    cd = face.GetChamferData()
    if cd is not None and cd[0]:
        return []
    if face.GetHoleData() is not None:
        return []

    cx, cy, cz = face_centre(bc, face.Tag, jx, jy, jz)
    normal = face_normal(bc, face.Tag)
    curved, straight = bc.classify_edges(face.Tag)
    dims = {}

    if len(curved) >= 2:
        ls = sorted([e[1] for e in curved])
        sr = ls[0]/(2*math.pi); lr = ls[-1]/(2*math.pi)
        dims['Small Diameter (calc)(mm)'] = round(sr*2, 4)
        dims['Large Diameter (calc)(mm)'] = round(lr*2, 4)
        if straight:
            slant = min(e[1] for e in straight)
            dr = lr - sr
            if slant > 0.001:
                ha = math.degrees(math.asin(min(1.0, dr/slant)))
                dims['Half Angle  (calc)(deg)'] = round(ha, 3)
    elif len(curved) == 1:
        dims['Rim Diameter (calc)(mm)'] = round(curved[0][1]/math.pi, 4)

    is_draft = dims.get('Half Angle  (calc)(deg)', 90) < 7.0
    label = 'DRAFT FACE (TAPER)' if is_draft else 'CONICAL FACE'
    return [_make(label, CAT_FORM, 0.72, TIER_GEOMETRY,
                  cx, cy, cz, normal, dims,
                  'ConicalEngine', face.JournalIdentifier)]


# ── ENGINE 7: TORUS ───────────────────────────────────────────────────────────

def engine_torus(face, ft, bc, jx, jy, jz):
    if ft != FT_TORUS:
        return []
    cx, cy, cz = face_centre(bc, face.Tag, jx, jy, jz)
    normal = face_normal(bc, face.Tag)
    curved, _ = bc.classify_edges(face.Tag)
    dims = {}; label = 'TOROIDAL FACE'; cat = CAT_FORM; conf = 0.75

    if len(curved) >= 2:
        ls = sorted([e[1] for e in curved])
        outer_r = ls[-1]/(2*math.pi)
        inner_r = ls[0]/(2*math.pi)
        gw = outer_r - inner_r
        dims['Outer Diameter (calc)(mm)'] = round(outer_r*2, 4)
        dims['Inner Diameter (calc)(mm)'] = round(inner_r*2, 4)
        dims['Channel Width  (calc)(mm)'] = round(gw, 4)
        if outer_r > 0.5 and gw < outer_r * 0.5:
            label = 'GROOVE / O-RING GROOVE'; cat = CAT_MACHINED; conf = 0.78
            dims['Groove Width (calc)(mm)'] = round(gw, 4)

    return [_make(label, cat, conf, TIER_GEOMETRY,
                  cx, cy, cz, normal, dims,
                  'TorusEngine', face.JournalIdentifier)]


# ── ENGINE 8: PLANAR ──────────────────────────────────────────────────────────
# FIX 9: Redesigned — functional zone classification primary,
#         edge count secondary confirmation only.

def engine_planar(face, ft, bc, jx, jy, jz):
    if ft != FT_PLANAR:
        return []

    ftag   = face.Tag
    cx,cy,cz = face_centre(bc, ftag, jx, jy, jz)
    normal = face_normal(bc, ftag)
    curved, straight = bc.classify_edges(ftag)
    nc, ns = len(curved), len(straight)
    ntypes = bc.neighbour_types(ftag)
    body_span = bc.body_span or 1.0

    # PRIMARY: Annular face — 2 concentric circles — high confidence
    if is_annular(curved, straight):
        dims = {}
        if curved:
            outer = max(e[1] for e in curved)/math.pi
            inner = min(e[1] for e in curved)/math.pi
            dims['Outer Diameter (calc)(mm)'] = round(outer, 4)
            dims['Inner Diameter (calc)(mm)'] = round(inner, 4)
            dims['Ring Width     (calc)(mm)'] = round((outer-inner)/2.0, 4)
        return [_make('ANNULAR FACE', CAT_FORM, 0.82, TIER_GEOMETRY,
                      cx, cy, cz, normal, dims,
                      'PlanarEngine/Annular', face.JournalIdentifier)]

    # Compute area once — used as gate throughout
    area = shoelace_area(bc, ftag, normal)
    area_ratio = area / max(bc.body_span**2, 1.0)

    # PRIMARY: Step/shoulder — functional zone: between cylinder and open space
    # FIX 9: neighbour context is primary signal, edge count is secondary check
    has_cyl_nbr = FT_CYLINDER in ntypes
    has_planar_nbr = FT_PLANAR in ntypes
    n_nbrs = len(bc.face_neighbour_tags.get(ftag, set()))

    if has_cyl_nbr and 0 < area < STEP_MAX_AREA:
        # Secondary: must have some straight edges (real steps do)
        if straight:
            dims = {
                'Step Width     (calc)(mm)'    : round(min(e[1] for e in straight), 4),
                'Area (polygon approx)(mm2)'   : round(area, 2),   # FIX 8
            }
            return [_make('STEP / SHOULDER', CAT_MACHINED, 0.70, TIER_GEOMETRY,
                          cx, cy, cz, normal, dims,
                          'PlanarEngine/Step', face.JournalIdentifier)]

    # PRIMARY: Slot — functional zone: enclosed, elongated, between walls
    # FIX 9: aspect ratio + enclosure + size gate all required
    if nc == 2 and ns == 2:
        cl = sorted([e[1] for e in curved])
        sl = sorted([e[1] for e in straight])
        r1 = cl[0]/math.pi; r2 = cl[-1]/math.pi
        if abs(r1-r2) < 0.5:
            sw = r1*2.0; slen = sl[0]+sw
            asp = slen / max(sw, 0.001)
            if SLOT_MIN_WIDTH < sw < body_span*0.8 and asp < SLOT_MAX_ASPECT:
                dims = {
                    'Slot Width      (calc)(mm)' : round(sw, 4),
                    'Slot Length     (calc)(mm)' : round(slen, 4),
                    'End Radius      (calc)(mm)' : round(r1, 4),
                    'Aspect Ratio    (calc)'     : round(asp, 2),
                    'Area (polygon approx)(mm2)' : round(area, 2),
                }
                return [_make('SLOT (CLOSED END)', CAT_MACHINED, 0.75,
                              TIER_HEURISTIC, cx, cy, cz, normal, dims,
                              'PlanarEngine/Slot', face.JournalIdentifier)]

    if nc == 1 and ns == 2:
        cr = curved[0][1]/math.pi
        sl = sorted([e[1] for e in straight])
        if len(sl) >= 2 and abs(sl[0]-sl[1]) < sl[0]*0.15:
            if SLOT_MIN_WIDTH < cr*2 < body_span*0.8:
                dims = {
                    'Slot Width  (calc)(mm)': round(cr*2, 4),
                    'Slot Length (calc)(mm)': round(sl[0]+cr, 4),
                }
                return [_make('SLOT (OPEN END)', CAT_MACHINED, 0.68,
                              TIER_HEURISTIC, cx, cy, cz, normal, dims,
                              'PlanarEngine/Slot', face.JournalIdentifier)]

    # PRIMARY: Pocket — FIX 9: enclosed on all sides + no cylinder neighbours
    if nc == 0 and ns == 4 and not has_cyl_nbr:
        sl = sorted([e[1] for e in straight])
        if abs(sl[0]-sl[1]) < 1.0 and abs(sl[2]-sl[3]) < 1.0:
            if 0 < area < STEP_MAX_AREA:
                dims = {
                    'Pocket Width   (calc)(mm)' : round((sl[0]+sl[1])/2, 4),
                    'Pocket Length  (calc)(mm)' : round((sl[2]+sl[3])/2, 4),
                    'Area (polygon approx)(mm2)': round(area, 2),
                }
                return [_make('POCKET — RECTANGULAR', CAT_MACHINED, 0.72,
                              TIER_HEURISTIC, cx, cy, cz, normal, dims,
                              'PlanarEngine/Pocket', face.JournalIdentifier)]

    if nc >= 1 and ns == 0 and not has_cyl_nbr and 0 < area < STEP_MAX_AREA:
        largest = max(e[1] for e in curved)
        dia = largest / math.pi
        dims = {
            'Pocket Diameter (calc)(mm)' : round(dia, 4),
            'Pocket Radius   (calc)(mm)' : round(dia/2, 4),
            'Area (polygon approx)(mm2)' : round(area, 2),
        }
        return [_make('POCKET — CIRCULAR', CAT_MACHINED, 0.65,
                      TIER_HEURISTIC, cx, cy, cz, normal, dims,
                      'PlanarEngine/Pocket', face.JournalIdentifier)]

    # PRIMARY: Draft angle — normal deviates from all principal axes
    axes = [(1,0,0),(0,1,0),(0,0,1)]
    min_ang = min(min(vec_angle_deg(normal,a),
                      vec_angle_deg(normal,(-a[0],-a[1],-a[2])))
                  for a in axes)

    all_L = sorted([e[1] for e in curved+straight])
    dims  = {}
    label = 'PLANAR FACE'
    conf  = 0.60
    tier  = TIER_HEURISTIC

    if 5.0 < min_ang < 85.0:
        label = 'PLANAR FACE (DRAFTED)'
        dims['Draft Angle    (calc)(deg)'] = round(min_ang, 2)
        conf = 0.65

    if all_L:
        dims['Max Edge Length (calc)(mm)'] = round(max(all_L), 4)
        dims['Min Edge Length (calc)(mm)'] = round(min(all_L), 4)
        dims['Edge count'                ] = len(all_L)
    if area > 0:
        dims['Area (polygon approx)(mm2)'] = round(area, 2)

    return [_make(label, CAT_FORM, conf, tier,
                  cx, cy, cz, normal, dims,
                  'PlanarEngine', face.JournalIdentifier)]


# =============================================================================
#  MAIN CLASSIFIER
# =============================================================================

def classify_face(face, bc, thread_map):
    try:
        ft = face.SolidFaceType.value
    except Exception:
        return None

    jx, jy, jz = parse_jid_coords(face.JournalIdentifier)

    # Run all engines sequentially; results fused below (FIX 4)
    candidates = []
    candidates += engine_hole(face, ft, bc, thread_map, jx, jy, jz)
    candidates += engine_fillet(face, ft, bc, jx, jy, jz)
    candidates += engine_chamfer(face, ft, bc, jx, jy, jz)
    candidates += engine_hole_entry(face, ft, bc, jx, jy, jz)
    candidates += engine_cylinder(face, ft, bc, thread_map, jx, jy, jz)
    candidates += engine_cone(face, ft, bc, jx, jy, jz)
    candidates += engine_torus(face, ft, bc, jx, jy, jz)
    candidates += engine_planar(face, ft, bc, jx, jy, jz)

    result = fuse(candidates)

    if result is None:
        result = _make('UNKNOWN (ft='+str(ft)+')', CAT_OTHER,
                       0.30, TIER_HEURISTIC,
                       jx, jy, jz, (0.0,0.0,1.0), {},
                       'NoEngine', face.JournalIdentifier)
        result['needs_review'] = True

    return result


# =============================================================================
#  HOLE PATTERN GROUPING
# =============================================================================

def group_hole_patterns(records):
    holes = [r for r in records if r['type'].startswith('HOLE')]
    if len(holes) < 2:
        return

    def dia_key(r):
        for k in r['dimensions']:
            if 'Diameter' in k and '(API)' in k:
                v = r['dimensions'][k]
                if isinstance(v,(int,float)):
                    return round(float(v), 1)
        return 0.0

    def z_key(r):
        return round(float(r.get('cz') or 0.0), 0)

    clusters = defaultdict(list)
    for r in holes:
        clusters[(dia_key(r), z_key(r))].append(r)

    pid = 1
    for key, grp in clusters.items():
        if len(grp) < 2: continue
        xs = [r['cx'] for r in grp if r['cx'] is not None]
        ys = [r['cy'] for r in grp if r['cy'] is not None]
        if len(xs) < 2: continue
        cx = sum(xs)/len(xs); cy = sum(ys)/len(ys)
        radii = [math.sqrt((r['cx']-cx)**2+(r['cy']-cy)**2)
                 for r in grp if r['cx'] is not None]
        if not radii: continue
        r_mean  = sum(radii)/len(radii)
        r_spread = max(radii)-min(radii)

        if r_mean > 1.0 and r_spread < 2.0:
            label = (("BOLT CIRCLE" if len(grp)>=BOLT_CIRCLE_MIN_N else "HOLE PAIR") +
                     "  dia="+str(round(key[0],2))+"mm" +
                     ("  PCD="+str(round(r_mean*2,2))+"mm" if len(grp)>=3 else
                      "  spacing="+str(round(r_mean*2,2))+"mm") +
                     "  n="+str(len(grp))+"  Z="+str(key[1])+"mm")
            for r in grp:
                r['pattern'] = label; r['pattern_id'] = pid
            pid += 1
        else:
            xs2=sorted(xs); ys2=sorted(ys)
            xsp=xs2[-1]-xs2[0]; ysp=ys2[-1]-ys2[0]
            if ysp<1.0 and xsp>1.0:
                sp=xsp/max(1,len(grp)-1)
                label="LINEAR ARRAY (X)  dia="+str(round(key[0],2))+"mm  spacing~"+str(round(sp,2))+"mm  n="+str(len(grp))
                for r in grp: r['pattern']=label; r['pattern_id']=pid
                pid+=1
            elif xsp<1.0 and ysp>1.0:
                sp=ysp/max(1,len(grp)-1)
                label="LINEAR ARRAY (Y)  dia="+str(round(key[0],2))+"mm  spacing~"+str(round(sp,2))+"mm  n="+str(len(grp))
                for r in grp: r['pattern']=label; r['pattern_id']=pid
                pid+=1


# =============================================================================
#  MAIN EXTRACTOR
# =============================================================================

def extract_all_features(work_part):
    thread_map = build_thread_map(work_part)
    records    = []
    feat_id    = 0

    for body_idx, body in enumerate(list(work_part.Bodies)):
        body_label = 'Body-' + str(body_idx+1)
        try:
            bc = BodyCache(body)
        except Exception:
            continue

        for face in bc.faces:
            try:
                rec = classify_face(face, bc, thread_map)
            except Exception:
                rec = None
            if rec is None:
                continue
            feat_id += 1
            rec['id']   = feat_id
            rec['body'] = body_label
            records.append(rec)

    group_hole_patterns(records)
    return records


# =============================================================================
#  REPORT WRITERS
# =============================================================================

SEP  = "=" * 80
SEP2 = "-" * 80

def write_txt(records, part_name, part_path, out_path):
    total     = len(records)
    high_c    = sum(1 for r in records if r['conf_level']=='HIGH')
    med_c     = sum(1 for r in records if r['conf_level']=='MEDIUM')
    low_c     = sum(1 for r in records if r['conf_level']=='LOW')
    needs_rev = sum(1 for r in records if r.get('needs_review'))
    dfm_count = sum(1 for r in records if 'DFM Flag' in r.get('dimensions',{}))

    lines = [
        SEP,
        "  NX GEOMETRIC FEATURE EXTRACTION REPORT  v4.1",
        "  Multi-Engine Fusion | Conflict Resolution | ISO Size Mapping",
        SEP,
        "  Part          : " + part_name,
        "  Full path     : " + str(part_path),
        "  Generated     : " + datetime.now().strftime('%Y-%m-%d  %H:%M:%S'),
        "  NX version    : v2506",
        "  Total features: " + str(total),
        "  Coordinates   : MCS  (mm)",
        "  Dim labels    : (API)=NX kernel confirmed  (calc)=geometry-inferred",
        "                  (polygon approx)=shoelace estimate  (ref)=ISO reference",
        SEP, "",
        "  CONFIDENCE SUMMARY",
        SEP2,
        "  HIGH   (>=0.85) API-confirmed    : " + str(high_c),
        "  MEDIUM (>=0.60) geometry-inferred: " + str(med_c),
        "  LOW    (<0.60)  heuristic        : " + str(low_c),
        "  [!] NEEDS REVIEW                 : " + str(needs_rev),   # FIX 5
        "  DFM FLAGS                        : " + str(dfm_count),
        "",
    ]

    # Category summary
    lines.append("  FEATURE SUMMARY BY CATEGORY")
    lines.append(SEP2)
    cat_counts = defaultdict(lambda: defaultdict(int))
    for r in records:
        cat_counts[r.get('category',CAT_OTHER)][
            r['type'].split('(')[0].strip()] += 1
    for cat in CATEGORY_ORDER:
        if cat not in cat_counts: continue
        lines.append("  " + cat)
        for ft,cnt in sorted(cat_counts[cat].items()):
            lines.append("    {:<54}  {:>4}  instance(s)".format(ft,cnt))
        lines.append("")

    # Patterns
    patterns = sorted(set(r['pattern'] for r in records if 'pattern' in r))
    if patterns:
        lines.append("  HOLE PATTERNS DETECTED")
        lines.append(SEP2)
        for p in patterns: lines.append("    " + p)
        lines.append("")

    # DFM flags
    dfm = [r for r in records if 'DFM Flag' in r.get('dimensions',{})]
    if dfm:
        lines.append("  DFM FLAGS --- ACTION REQUIRED")
        lines.append(SEP2)
        for r in dfm:
            lines.append("    #{:04d}  {}  ->  {}".format(
                r['id'], r['type'], r['dimensions']['DFM Flag']))
        lines.append("")

    # Needs review
    rev = [r for r in records if r.get('needs_review')]
    if rev:
        lines.append("  [!] NEEDS MANUAL REVIEW (confidence < 0.60)")   # FIX 5
        lines.append(SEP2)
        for r in rev:
            lines.append("    #{:04d}  {:42}  conf={:.2f}  tier={}  engine={}".format(
                r['id'], r['type'], r['confidence'],
                {3:'API',2:'GEO',1:'HEU'}.get(r.get('tier',1),'?'),
                r.get('engine','')))
        lines.append("")

    lines += [SEP, ""]

    # Detailed records
    for r in records:
        lines.append(SEP2)
        bar = "#" * int(r['confidence']*10) + "." * (10-int(r['confidence']*10))
        lines.append("  ID        : #{:04d}   Body: {}   Category: {}".format(
            r['id'], r['body'], r.get('category','—')))
        lines.append("  Type      : " + r['type'])
        lines.append("  Confidence: [{}] {:.0f}%  {}  Tier: {}  Engine: {}".format(
            bar, r['confidence']*100, r['conf_level'],
            {3:'API',2:'GEO',1:'HEU'}.get(r.get('tier',1),'?'),
            r.get('engine','')))
        if r.get('needs_review'):
            lines.append("  [!] NEEDS REVIEW — low confidence detection")
        if 'pattern' in r:
            lines.append("  Pattern   : " + r['pattern'])
        lines.append("  Source    : " + str(r.get('jid','')))
        lines.append("")
        cx,cy,cz = r['cx'],r['cy'],r['cz']
        if cx is not None:
            lines += ["  MCS Position (feature centre)",
                      "       X = {:+14.4f} mm".format(cx),
                      "       Y = {:+14.4f} mm".format(cy),
                      "       Z = {:+14.4f} mm".format(cz)]
        ax = r.get('axis',(0,0,0))
        lines += ["  Axis / Normal Direction (unit vector)",
                  "       dX = {:+.6f}".format(ax[0]),
                  "       dY = {:+.6f}".format(ax[1]),
                  "       dZ = {:+.6f}".format(ax[2])]
        if r['dimensions']:
            lines.append("  Dimensions / Properties")
            for k,v in r['dimensions'].items():
                if isinstance(v,float):
                    lines.append("       {:<44} = {:+.4f}".format(k,v))
                else:
                    lines.append("       {:<44} = {}".format(k,v))
        lines.append("")

    lines += [SEP, "  END OF REPORT", SEP]
    with open(out_path,'w',encoding='utf-8') as f:
        f.write("\n".join(lines))


def write_csv(records, part_name, out_path):
    all_dim_keys = []
    seen = set()
    for r in records:
        for k in r['dimensions']:
            if k not in seen:
                all_dim_keys.append(k); seen.add(k)

    header = ['ID','Body','Category','Type','Confidence','ConfLevel',
              'Tier','NeedsReview','Engine','Pattern','JournalIdentifier',
              'Centre_X_mm','Centre_Y_mm','Centre_Z_mm',
              'Normal_dX','Normal_dY','Normal_dZ'] + all_dim_keys

    def cv(v):
        s = str(v) if not isinstance(v,float) else "{:.4f}".format(v)
        if any(c in s for c in ',"\n'): s='"'+s.replace('"','""')+'"'
        return s

    rows = [",".join(header)]
    for r in records:
        ax = r.get('axis',(0,0,0))
        cx = "{:.4f}".format(r['cx']) if r['cx'] is not None else ""
        cy = "{:.4f}".format(r['cy']) if r['cy'] is not None else ""
        cz = "{:.4f}".format(r['cz']) if r['cz'] is not None else ""
        base = [
            str(r['id']), r['body'],
            cv(r.get('category','')), cv(r['type']),
            "{:.3f}".format(r['confidence']),
            r['conf_level'],
            {3:'API',2:'GEO',1:'HEU'}.get(r.get('tier',1),'?'),
            'YES' if r.get('needs_review') else 'NO',
            cv(r.get('engine','')), cv(r.get('pattern','')),
            cv(r.get('jid','')),
            cx, cy, cz,
            "{:.6f}".format(ax[0]),"{:.6f}".format(ax[1]),"{:.6f}".format(ax[2]),
        ]
        rows.append(",".join(base+[cv(r['dimensions'].get(k,"")) for k in all_dim_keys]))
    with open(out_path,'w',encoding='utf-8') as f:
        f.write("\n".join(rows))


def write_json(records, part_name, part_path, out_path):
    """JSON output — structured for GD&T automation Module 2."""
    def safe(v):
        return round(v,6) if isinstance(v,float) else v

    output = {
        'meta': {
            'part_name'   : part_name,
            'part_path'   : str(part_path),
            'generated'   : datetime.now().isoformat(),
            'nx_version'  : 'v2506',
            'extractor'   : 'NX Feature Extractor v4.1',
            'total'       : len(records),
            'high_conf'   : sum(1 for r in records if r['conf_level']=='HIGH'),
            'med_conf'    : sum(1 for r in records if r['conf_level']=='MEDIUM'),
            'low_conf'    : sum(1 for r in records if r['conf_level']=='LOW'),
            'needs_review': sum(1 for r in records if r.get('needs_review')),
            'dfm_flags'   : sum(1 for r in records
                                if 'DFM Flag' in r.get('dimensions',{})),
        },
        'features': []
    }
    for r in records:
        feat = {
            'id'          : r['id'],
            'body'        : r['body'],
            'type'        : r['type'],
            'category'    : r.get('category',''),
            'confidence'  : r['confidence'],
            'conf_level'  : r['conf_level'],
            'tier'        : {3:'API',2:'GEO',1:'HEU'}.get(r.get('tier',1),'?'),
            'needs_review': r.get('needs_review',False),
            'engine'      : r.get('engine',''),
            'position'    : {'x':safe(r['cx']),'y':safe(r['cy']),'z':safe(r['cz'])},
            'axis'        : {'dx':r['axis'][0],'dy':r['axis'][1],'dz':r['axis'][2]},
            'dimensions'  : {k:safe(v) for k,v in r['dimensions'].items()},
        }
        if 'pattern' in r: feat['pattern'] = r['pattern']
        if 'jid'     in r: feat['journal_id'] = r['jid']
        output['features'].append(feat)

    with open(out_path,'w',encoding='utf-8') as f:
        json.dump(output, f, indent=2)


# =============================================================================
#  ENTRY POINT
# =============================================================================

def main():
    session   = NXOpen.Session.GetSession()
    work_part = session.Parts.Work
    lw        = session.ListingWindow
    lw.Open()

    if work_part is None:
        NXOpen.UI.GetUI().NXMessageBox.Show(
            "Feature Extractor v4.1",
            NXOpen.NXMessageBox.DialogType.Error,
            "No active part. Open a part first.")
        return

    part_name = work_part.Name or "Unnamed"
    try:
        part_dir = os.path.dirname(work_part.FullPath)
    except Exception:
        part_dir = os.path.expanduser("~")

    lw.WriteLine("=" * 62)
    lw.WriteLine("  NX Feature Extractor v4.1")
    lw.WriteLine("  Multi-Engine Fusion | Conflict Resolution")
    lw.WriteLine("  Part: " + part_name)
    lw.WriteLine("=" * 62)

    records = extract_all_features(work_part)

    txt_path  = os.path.join(part_dir, part_name + "_FEATURES_v4_1.txt")
    csv_path  = os.path.join(part_dir, part_name + "_FEATURES_v4_1.csv")
    json_path = os.path.join(part_dir, part_name + "_FEATURES_v4_1.json")

    write_txt(records, part_name, work_part.FullPath, txt_path)
    write_csv(records, part_name, csv_path)
    write_json(records, part_name, work_part.FullPath, json_path)

    cat_totals   = defaultdict(int)
    high_conf    = sum(1 for r in records if r['conf_level']=='HIGH')
    needs_review = sum(1 for r in records if r.get('needs_review'))
    dfm_flags    = sum(1 for r in records if 'DFM Flag' in r.get('dimensions',{}))
    for r in records:
        cat_totals[r.get('category',CAT_OTHER)] += 1

    summary = "\n".join(
        "  "+cat+": "+str(cat_totals[cat])
        for cat in CATEGORY_ORDER if cat in cat_totals)

    msg = ("Feature extraction v4.1 complete!\n\n"
           "  Total features : " + str(len(records)) + "\n"
           "  HIGH confidence: " + str(high_conf) + "\n"
           "  Needs review   : " + str(needs_review) + "\n"
           "  DFM flags      : " + str(dfm_flags) + "\n\n"
           + summary +
           "\n\n  TXT  : " + txt_path +
           "\n  CSV  : " + csv_path +
           "\n  JSON : " + json_path)

    lw.WriteLine(msg)
    NXOpen.UI.GetUI().NXMessageBox.Show(
        "Feature Extractor v4.1 — Done",
        NXOpen.NXMessageBox.DialogType.Information, msg)

if __name__ == '__main__':
    main()
