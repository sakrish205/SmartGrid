"""Toolpath quality measurement for SmartGrid paths.

measure_route() takes gun-space waypoints and a mesh, projects each sample
onto the surface, then reports spacing, standoff, crossings, thickness
coverage, and collisions.

ponytail: inner spacing loop is O(N_samples * k) where k = avg neighbours
per query_ball_point call. Acceptable for <=200 passes; for larger routes
vectorise the side-filter step.
"""
from __future__ import annotations
import numpy as np
import trimesh
from scipy.spatial import cKDTree

from app.path.resampler import resample_arc


def measure_route(
    mesh: trimesh.Trimesh,
    passes: list,               # list of np.ndarray (N, 3)
    pitch_mm: float,
    standoff_mm: float,
    standoff_tol_mm: float = 5.0,
    surface_tol_frac: float = 0.05,
    gen_time_s: float | None = None,
) -> dict:
    """Measure quality metrics for a toolpath against a mesh surface.

    Parameters
    ----------
    mesh            : trimesh.Trimesh — the workpiece; must be watertight for
                      collision detection (mesh.contains); open meshes still
                      yield valid spacing, standoff and thickness results.
    passes          : list of (N, 3) arrays of gun-space waypoints, one per pass.
    pitch_mm        : target centre-to-centre spacing between passes (mm).
    standoff_mm     : target distance from gun to mesh surface (mm).
    standoff_tol_mm : allowable deviation from standoff_mm before flagging
                      OUT_OF_TOLERANCE (default ±5 mm).
    surface_tol_frac: fraction of pitch_mm allowed as spacing error before
                      classifying OVERLAP or GAP (default 5%).
    gen_time_s      : optional generation time to include in the report.

    Returns
    -------
    dict with keys:
        passes         : list of per-pass dicts (spacing, standoff, collision, waypoints)
        global         : aggregate totals
        crossings      : list of {pass_a, pass_b, location, dist_mm} dicts
        thickness      : {mean, min, max, cv, pct_uncovered}
    """
    if not passes:
        return {'passes': [], 'global': {}, 'crossings': [], 'thickness': {}}

    resample_s     = max(pitch_mm / 10.0, 0.1)
    sigma          = 0.5 * pitch_mm
    overlap_thresh = pitch_mm * (1.0 - surface_tol_frac)
    gap_thresh     = pitch_mm * (1.0 + surface_tol_frac)
    cross_thresh   = 0.05 * pitch_mm

    # ---- Step 1 & 2: resample, project, standoff ----
    pass_gun      = []   # (M, 3) resampled gun points
    pass_aim      = []   # (M, 3) nearest surface points
    pass_aim_nrm  = []   # (M, 3) surface normals at aim points
    pass_standoff = []   # (M,)   actual gun-to-surface distances

    for pts in passes:
        pts  = np.asarray(pts, dtype=float)
        gun  = resample_arc(pts, resample_s) if len(pts) >= 2 else pts.copy()
        aim, dists, tri_ids = trimesh.proximity.closest_point(mesh, gun)
        nrms = mesh.face_normals[tri_ids]
        pass_gun.append(gun)
        pass_aim.append(aim)
        pass_aim_nrm.append(nrms)
        pass_standoff.append(dists)

    # ---- Build combined KDTree of all aim points ----
    all_aim      = np.concatenate(pass_aim, axis=0)
    all_pass_idx = np.concatenate([np.full(len(a), i, dtype=np.int32)
                                   for i, a in enumerate(pass_aim)])
    tree = cKDTree(all_aim)

    # ---- Steps 3-5: spacing, crossings ----
    per_pass_rows = []
    crossings     = []
    seen_cross    = set()   # deduplicate (a,b) pairs

    for k, (aim, nrms, dists) in enumerate(zip(pass_aim, pass_aim_nrm, pass_standoff)):
        m = len(aim)

        # ---- standoff stats ----
        out_of_tol = np.abs(dists - standoff_mm) > standoff_tol_mm
        std_stats  = {
            'min_mm'          : float(dists.min()),
            'mean_mm'         : float(dists.mean()),
            'max_mm'          : float(dists.max()),
            'count_out_of_tol': int(out_of_tol.sum()),
        }

        # ---- spacing: side-vector approach ----
        spacings = []   # collect all valid measurements this pass

        for j in range(m):
            S   = aim[j]
            nrm = nrms[j]
            nl  = np.linalg.norm(nrm)
            if nl < 1e-9:
                continue
            nrm = nrm / nl

            # tangent from surface neighbours (use aim points for accuracy)
            if j == 0:
                t = aim[1] - aim[0] if m > 1 else np.array([1.0, 0.0, 0.0])
            elif j == m - 1:
                t = aim[-1] - aim[-2]
            else:
                t = aim[j + 1] - aim[j - 1]
            tl = np.linalg.norm(t)
            if tl < 1e-9:
                continue
            t = t / tl

            sv  = np.cross(nrm, t)
            svl = np.linalg.norm(sv)
            if svl < 1e-9:
                continue
            sv = sv / svl

            # neighbours within 2 * pitch on the surface
            idx_list = tree.query_ball_point(S, 2.0 * pitch_mm)

            best_pos = np.inf
            best_neg = np.inf

            for i in idx_list:
                ki = all_pass_idx[i]
                if ki == k:
                    continue
                q   = all_aim[i]
                dq  = np.linalg.norm(q - S)

                # crossing: two passes that meet at the surface
                if dq < cross_thresh:
                    key = (min(k, ki), max(k, ki))
                    if key not in seen_cross:
                        seen_cross.add(key)
                        crossings.append({
                            'pass_a'  : k,
                            'pass_b'  : ki,
                            'location': S.tolist(),
                            'dist_mm' : float(dq),
                        })

                side = float(np.dot(q - S, sv))
                if side > 0:
                    if dq < best_pos:
                        best_pos = dq
                elif side < 0:
                    if dq < best_neg:
                        best_neg = dq

            if best_pos < np.inf:
                spacings.append(best_pos)
            if best_neg < np.inf:
                spacings.append(best_neg)

        # ---- classify spacing ----
        if spacings:
            sp    = np.array(spacings)
            ok_m  = (sp >= overlap_thresh) & (sp <= gap_thresh)
            ov_m  = sp < overlap_thresh
            gap_m = sp > gap_thresh
            sp_stats = {
                'min_mm'       : float(sp.min()),
                'mean_mm'      : float(sp.mean()),
                'max_mm'       : float(sp.max()),
                'pct_ok'       : float(ok_m.sum()  / len(sp) * 100.0),
                'pct_overlap'  : float(ov_m.sum()  / len(sp) * 100.0),
                'pct_gap'      : float(gap_m.sum() / len(sp) * 100.0),
                'count_ok'     : int(ok_m.sum()),
                'count_overlap': int(ov_m.sum()),
                'count_gap'    : int(gap_m.sum()),
            }
        else:
            sp_stats = {'min_mm': None, 'mean_mm': None, 'max_mm': None,
                        'pct_ok': None, 'pct_overlap': None, 'pct_gap': None,
                        'count_ok': 0, 'count_overlap': 0, 'count_gap': 0}

        # ---- collision: waypoints inside mesh ----
        gun_pts  = pass_gun[k]
        gun_dists = pass_standoff[k]   # already computed above
        if mesh.is_watertight:
            inside   = mesh.contains(gun_pts)
            n_inside = int(inside.sum())
            method   = 'contains'
        else:
            # Normal-side test: gun on the inward side of its nearest surface point
            diff     = gun_pts - pass_aim[k]
            dotprod  = np.einsum('ij,ij->i', diff, pass_aim_nrm[k])
            n_inside = int((dotprod < 0).sum())
            method   = 'normal_side'
        coll_stats = {
            'waypoints_inside': n_inside,
            'method'          : method,
            'min_dist_mm'     : float(gun_dists.min()),
            'mean_dist_mm'    : float(gun_dists.mean()),
            'max_dist_mm'     : float(gun_dists.max()),
        }

        per_pass_rows.append({
            'spacing'       : sp_stats,
            'standoff'      : std_stats,
            'collision'     : coll_stats,
            'waypoint_count': len(gun_pts),
        })

    # ---- Step 7: thickness simulation (Gaussian kernel) ----
    thickness_stats = _thickness(mesh, all_aim, sigma)

    # ---- global aggregates ----
    total_wpts    = sum(r['waypoint_count'] for r in per_pass_rows)
    total_inside  = sum(r['collision']['waypoints_inside'] for r in per_pass_rows)
    total_oot     = sum(r['standoff']['count_out_of_tol'] for r in per_pass_rows)

    total_ok  = sum(r['spacing']['count_ok']        for r in per_pass_rows)
    total_ov  = sum(r['spacing']['count_overlap']   for r in per_pass_rows)
    total_gap = sum(r['spacing']['count_gap']       for r in per_pass_rows)
    total_sp  = total_ok + total_ov + total_gap

    if total_sp > 0:
        g_pct_ok  = float(total_ok  / total_sp * 100.0)
        g_pct_ov  = float(total_ov  / total_sp * 100.0)
        g_pct_gap = float(total_gap / total_sp * 100.0)
    else:
        g_pct_ok = g_pct_ov = g_pct_gap = None

    global_stats = {
        'total_passes'        : len(passes),
        'total_waypoints'     : total_wpts,
        'pct_ok'              : g_pct_ok,
        'pct_overlap'         : g_pct_ov,
        'pct_gap'             : g_pct_gap,
        'count_out_of_tol'    : total_oot,
        'total_waypoints_inside': total_inside,
    }
    if gen_time_s is not None:
        global_stats['gen_time_s'] = gen_time_s

    return {
        'passes'   : per_pass_rows,
        'global'   : global_stats,
        'crossings': crossings,
        'thickness': thickness_stats,
    }


def _thickness(
    mesh: trimesh.Trimesh,
    aim_pts: np.ndarray,
    sigma: float,
) -> dict:
    """Gaussian paint thickness at each mesh vertex.

    thickness(v) = sum_i exp(-||v - aim_i||^2 / (2 * sigma^2))
    where the sum is over all aim points within 3*sigma of v.

    Returns mean, min, max, cv (over vertices that received any paint),
    and pct_uncovered (fraction of all mesh vertices with zero coverage).
    """
    if len(aim_pts) == 0:
        return {'mean': 0.0, 'min': 0.0, 'max': 0.0, 'cv': 0.0, 'pct_uncovered': 100.0}

    verts    = mesh.vertices
    tree_aim = cKDTree(aim_pts)
    radius   = 3.0 * sigma
    inv2s2   = 1.0 / (2.0 * sigma ** 2)

    thickness = np.zeros(len(verts))
    for vi, v in enumerate(verts):
        idxs = tree_aim.query_ball_point(v, radius)
        if idxs:
            dists2      = np.sum((aim_pts[idxs] - v) ** 2, axis=1)
            thickness[vi] = float(np.sum(np.exp(-dists2 * inv2s2)))

    covered = thickness > 0.0
    pct_unc = float((~covered).sum() / len(verts) * 100.0)

    if covered.any():
        t_cov = thickness[covered]
        mean  = float(t_cov.mean())
        std   = float(t_cov.std())
        cv    = float(std / mean) if mean > 1e-12 else 0.0
        return {
            'mean'         : mean,
            'min'          : float(t_cov.min()),
            'max'          : float(t_cov.max()),
            'cv'           : cv,
            'pct_uncovered': pct_unc,
        }

    return {'mean': 0.0, 'min': 0.0, 'max': 0.0, 'cv': 0.0, 'pct_uncovered': 100.0}
