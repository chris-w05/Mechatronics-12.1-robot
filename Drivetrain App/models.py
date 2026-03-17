
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class Sample:
    t: float
    x: float
    y: float
    th: float
    v: float
    w: float
    vl: float
    vr: float
    xd: Optional[float] = None
    yd: Optional[float] = None
    thd: Optional[float] = None
    vl_ref: Optional[float] = None
    vr_ref: Optional[float] = None
    v_ref: Optional[float] = None
    w_ref: Optional[float] = None
    pl: Optional[float] = None
    pr: Optional[float] = None
    pl_ref: Optional[float] = None
    pr_ref: Optional[float] = None