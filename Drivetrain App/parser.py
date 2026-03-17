
from __future__ import annotations

import json
import math
from typing import Optional, Tuple

from models import Sample


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def parse_line(line: str, *, track_width_in: Optional[float]) -> Optional[Sample]:
    line = line.strip()
    if not line:
        return None

    def vw_from_wheels(vl_: float, vr_: float) -> Tuple[float, float]:
        v_ = 0.5 * (vl_ + vr_)
        if track_width_in is None or abs(track_width_in) < 1e-9:
            w_ = 0.0
        else:
            w_ = (vr_ - vl_) / track_width_in
        return v_, w_

    if line.startswith("{") and line.endswith("}"):
        try:
            d = json.loads(line)

            def get_first(keys, default=None):
                for k in keys:
                    if k in d and d[k] is not None:
                        return d[k]
                return default

            t_raw = float(get_first(["t", "time"], 0.0))
            if t_raw > 1e6:
                t_s = t_raw * 1e-6
            elif t_raw > 1e3:
                t_s = t_raw * 1e-3
            else:
                t_s = t_raw

            x = float(d["x"])
            y = float(d["y"])
            th = float(get_first(["th", "theta", "heading"], 0.0))

            vl = float(get_first(["vl", "vL", "left"], 0.0))
            vr = float(get_first(["vr", "vR", "right"], 0.0))
            v = float(get_first(["v"], 0.5 * (vl + vr)))
            w = get_first(["w"], None)
            if w is None:
                v_auto, w_auto = vw_from_wheels(vl, vr)
                if "v" not in d:
                    v = v_auto
                w = w_auto
            else:
                w = float(w)

            xd = get_first(["xd", "x_d", "desired_x"], None)
            yd = get_first(["yd", "y_d", "desired_y"], None)
            thd = get_first(["thd", "theta_d", "heading_d", "desired_theta"], None)

            vl_ref = get_first(["vl_ref", "vL_ref", "vlRef"], None)
            vr_ref = get_first(["vr_ref", "vR_ref", "vrRef"], None)
            pl = get_first(["pl", "pL", "left_pos"], None)
            pr = get_first(["pr", "pR", "right_pos"], None)
            pl_ref = get_first(["pl_ref", "pL_ref", "plRef"], None)
            pr_ref = get_first(["pr_ref", "pR_ref", "prRef"], None)

            vl_ref_f = float(vl_ref) if vl_ref is not None else None
            vr_ref_f = float(vr_ref) if vr_ref is not None else None
            v_ref = float(d["v_ref"]) if ("v_ref" in d and d["v_ref"] is not None) else None
            w_ref = float(d["w_ref"]) if ("w_ref" in d and d["w_ref"] is not None) else None
            if v_ref is None and vl_ref_f is not None and vr_ref_f is not None:
                v_ref = 0.5 * (vl_ref_f + vr_ref_f)
            if w_ref is None and vl_ref_f is not None and vr_ref_f is not None and track_width_in:
                w_ref = (vr_ref_f - vl_ref_f) / track_width_in

            return Sample(
                t=t_s,
                x=x,
                y=y,
                th=wrap_pi(th),
                v=v,
                w=w,
                vl=vl,
                vr=vr,
                xd=float(xd) if xd is not None else None,
                yd=float(yd) if yd is not None else None,
                thd=wrap_pi(float(thd)) if thd is not None else None,
                vl_ref=vl_ref_f,
                vr_ref=vr_ref_f,
                v_ref=v_ref,
                w_ref=w_ref,
                pl=float(pl) if pl is not None else None,
                pr=float(pr) if pr is not None else None,
                pl_ref=float(pl_ref) if pl_ref is not None else None,
                pr_ref=float(pr_ref) if pr_ref is not None else None,
            )
        except Exception:
            return None

    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    if not parts:
        return None

    def is_number(s: str) -> bool:
        try:
            float(s)
            return True
        except Exception:
            return False

    if not is_number(parts[0]):
        if parts[0].upper() not in ("P", "POSE", "TELEM", "T"):
            return None
        parts = parts[1:]

    try:
        vals = list(map(float, parts))
    except Exception:
        return None

    try:
        if len(vals) == 6:
            t_ms, x, y, th, vl, vr = vals
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_ms * 1e-3, x, y, wrap_pi(th), v, w, vl, vr)

        if len(vals) == 8:
            t_ms, x, y, th, v, w, vl, vr = vals
            return Sample(t_ms * 1e-3, x, y, wrap_pi(th), v, w, vl, vr)

        if len(vals) == 9:
            t_ms, x, y, th, vl, vr, pl, pr, _aux = vals
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_ms * 1e-3, x, y, wrap_pi(th), v, w, vl, vr, pl=pl, pr=pr)

        if len(vals) == 10:
            t_ms, x, y, th, vl, vr, pl, pr, pl_ref, pr_ref = vals
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_ms * 1e-3, x, y, wrap_pi(th), v, w, vl, vr, pl=pl, pr=pr, pl_ref=pl_ref, pr_ref=pr_ref)

        if len(vals) == 12:
            t_ms, x, y, th, xd, yd, thd, v, w, vl, vr, _aux = vals
            return Sample(t_ms * 1e-3, x, y, wrap_pi(th), v, w, vl, vr, xd=xd, yd=yd, thd=wrap_pi(thd))

        if len(vals) >= 15:
            t_ms, x, y, th, xd, yd, thd, vl, vr, vl_ref, vr_ref, pl, pr, pl_ref, pr_ref = vals[:15]
            v, w = vw_from_wheels(vl, vr)
            v_ref, w_ref = vw_from_wheels(vl_ref, vr_ref)
            return Sample(
                t_ms * 1e-3, x, y, wrap_pi(th), v, w, vl, vr,
                xd=xd, yd=yd, thd=wrap_pi(thd),
                vl_ref=vl_ref, vr_ref=vr_ref, v_ref=v_ref, w_ref=w_ref,
                pl=pl, pr=pr, pl_ref=pl_ref, pr_ref=pr_ref
            )
    except Exception:
        return None

    return None