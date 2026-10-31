"""装甲板 PnP 位姿解算：IPPE 初值 + LM 精修，畸变直接参与优化。"""
from dataclasses import dataclass

import cv2
import numpy as np

from .armor import ARMOR_H, ARMOR_W, armor_object_points


@dataclass
class PnPResult:
    ok: bool
    R: np.ndarray | None = None        # 板 -> 相机
    t: np.ndarray | None = None        # 板中心在相机系
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    reproj: float = float("nan")
    reason: str = ""


def solve_armor_pnp(corners, K, dist, w=ARMOR_W, h=ARMOR_H,
                    max_reproj=6.0, min_z=0.05):
    """corners: (4,2) 左上/左下/右下/右上。"""
    obj = armor_object_points(w, h)
    img = np.asarray(corners, dtype=np.float64).reshape(4, 1, 2)
    if not np.all(np.isfinite(img)):
        return PnPResult(False, reason="corners not finite")
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error as e:                     # noqa: BLE001
        return PnPResult(False, reason="ippe error: %s" % e)
    if n == 0:
        return PnPResult(False, reason="no solution")
    best = None
    for i in range(n):
        rv = rvecs[i].reshape(3, 1)
        tv = tvecs[i].reshape(3, 1)
        R, _ = cv2.Rodrigues(rv)
        pts_c = (R @ obj.T).T + tv.reshape(1, 3)
        if np.any(pts_c[:, 2] < min_z):
            continue
        rv2, tv2 = cv2.solvePnPRefineLM(obj, img, K, dist, rv.copy(), tv.copy())
        proj, _ = cv2.projectPoints(obj, rv2, tv2, K, dist)
        e = proj.reshape(-1, 2) - img.reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))
        R2, _ = cv2.Rodrigues(rv2)
        pts2 = (R2 @ obj.T).T + tv2.reshape(1, 3)
        if np.any(pts2[:, 2] < min_z):
            continue
        if best is None or rms < best[0]:
            best = (rms, rv2, tv2, R2)
    if best is None:
        return PnPResult(False, reason="degenerate (z<=0)")
    rms, rv, tv, R = best
    if not np.isfinite(rms) or rms > max_reproj:
        return PnPResult(False, R=R, t=tv.reshape(3), rvec=rv, tvec=tv,
                         reproj=rms, reason="reproj %.2f px > %.2f" % (rms, max_reproj))
    return PnPResult(True, R=R, t=tv.reshape(3), rvec=rv, tvec=tv, reproj=rms)
