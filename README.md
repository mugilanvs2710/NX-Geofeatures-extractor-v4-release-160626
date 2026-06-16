# 🔩 NX Geometric Feature Extractor

> **Automatically classify every geometric feature in a Siemens NX 3D model — with confidence scores, ISO size mapping, DFM flags, and structured JSON output ready for GD&T automation.**

[![NX Version](https://img.shields.io/badge/Siemens%20NX-v2506-0078D4?style=flat-square)](https://plm.sw.siemens.com/en-US/nx/)
[![Language](https://img.shields.io/badge/Language-NXOpen%20Python-3776AB?style=flat-square)](https://docs.sw.siemens.com/)
[![Version](https://img.shields.io/badge/Latest-v4.1-7C3AED?style=flat-square)]()
[![Architecture](https://img.shields.io/badge/Architecture-Multi--Engine%20Fusion-0D9488?style=flat-square)]()
[![Accuracy](https://img.shields.io/badge/Accuracy-~90%25-10B981?style=flat-square)]()
[![License](https://img.shields.io/badge/License-MIT-F59E0B?style=flat-square)]()

---

## What's New in v4.1 — 10 Critical Fixes

| Fix | Description | Severity |
|---|---|---|
| FIX 1 | TYPE_SPECIFICITY key lookup — full type string | CRITICAL |
| FIX 2 | GetBlendData/GetChamferData None guard | SAFETY |
| FIX 3 | edge.GetFaces() list() wrap | BUG |
| FIX 4 | Sequential comment correction | CLARITY |
| FIX 5 | ASCII-only listing output | COMPAT |
| FIX 6 | ISO map externalisable via fe_config.json | DEPLOY |
| FIX 7 | Fusion conflict resolution: API > GEO > HEU | ACCURACY |
| FIX 8 | Precision labelling (API)/(calc)/(polygon approx) | INTEGRITY |
| FIX 9 | Planar engine redesign: functional zone primary | ACCURACY |
| FIX 10 | Variable-radius fillet: BodyCache neighbour check | ACCURACY |

---

## Architecture — Multi-Engine Fusion

```
BodyCache (single NX pass per body)
    → 8 Engines (sequential per face)
        HoleEngine     conf=0.95  tier=API
        FilletEngine   conf=0.90  tier=API
        ChamferEngine  conf=0.90  tier=API
        HoleEntryEng   conf=0.88  tier=API
        CylinderEngine conf=0.70  tier=GEO
        ConicalEngine  conf=0.72  tier=GEO
        TorusEngine    conf=0.75  tier=GEO
        PlanarEngine   conf=0.60  tier=HEU
    → Fusion + Conflict Resolution
        1. Confidence score
        2. API > GEO > HEU (within 0.08 window)
        3. Type specificity
    → ISO Mapper + DFM Flags + Pattern Grouping
    → TXT + CSV + JSON
```

---

## How to Run

```
1. Open part: File → Open → .prt  OR  File → Import → STEP 214
2. Play journal: Tools → Journal → Play → NX_Feature_Extractor_v4_1.py
3. Outputs saved in part folder: _FEATURES_v4_1.txt / .csv / .json
4. Progress: View → Information → Listing Window
```

---

## Confidence Levels

```
HIGH   (>=0.85)  API-confirmed     Safe for GD&T automation
MEDIUM (>=0.60)  geometry-inferred Correct in most cases
LOW    (<0.60)   heuristic         [!] NEEDS REVIEW flagged
```

---

## Feature Types Detected (25+)

HOLE: Simple, Simple(THROUGH), Counterbore, Countersink
MACHINED: Bore, Boss/Pin, Threaded Bore/Boss, Slot(Closed/Open),
          Pocket(Rect/Circular), Step/Shoulder, Groove/O-Ring
FINISHING: Fillet, Fillet(Variable), Chamfer, Chamfer-WeldPrep, Chamfer-Entry
FORM: Annular, Draft Face, Conical, Toroidal, Planar, Planar(Drafted)
PATTERNS: Bolt Circle, Hole Pair, Linear Array X/Y

---

## Confirmed APIs — NX v2506

WORKS: face.SolidFaceType.value, GetHoleData(), GetBlendData(),
       GetChamferData(), list(GetEdges()), JournalIdentifier,
       edge.GetLength(), list(GetFaces()), GetLocations(),
       ResizeHoleData.*, list(body.GetFaces())

NOT AVAILABLE: AskFaceUvMinMaxValues, EvaluateAtPoint,
               GetAdjacentFaces, edge.EdgeType.value,
               GetFirstFacetOnFace, NXOpen.UF

CRITICAL: Always wrap NX iterators with list() — generators exhaust silently.

---

## Accuracy by Version

| Feature    | v1   | v2   | v3   | v4.1 |
|------------|------|------|------|------|
| Holes      | 95%  | 95%  | 97%  | 98%  |
| Fillets    | 70%  | 40%  | 85%  | 90%  |
| Boss/Bore  | 50%  | 50%  | 80%  | 85%  |
| Pockets    | 0%   | 30%  | 75%  | 80%  |
| Slots      | 0%   | 40%  | 70%  | 75%  |
| Overall    | ~60% | ~55% | ~82% | ~90% |

---

## Roadmap

Module 1  Feature Extraction    COMPLETE (v4.1)
Module 2  Tolerance Rule Engine  NEXT — ISO GPS + ASME Y14.5
Module 3  NX PMI Writer          PLANNED — annotations in NX workspace
Module 4  ISO/ASME Toggle        PLANNED — single-button standard switch

---

## License: MIT
