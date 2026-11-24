import sys, math, numpy as np
from dataclasses import replace
from pathlib import Path; ROOT=str(Path(__file__).resolve().parents[3])
sys.path.insert(0, ROOT+"/pi_rover"); sys.path.insert(0, ROOT+"/analysis/2026-11-24-breadcrumb-path")
import replay_leader_csv as R
from breadcrumb_path import BreadcrumbPath
from follower_formation import BreadcrumbFormationTracker, PathFormationTracker, FormationPlan
from simulate_formation import status

class IntentPath(BreadcrumbPath):
    """Breadcrumbs that also store the Leader's planned curvature at each point."""
    def __init__(self, **kw):
        super().__init__(**kw); self._k_intent=[]; self._pending=0.0
    def observe_intent(self, x, y, h, kappa):
        n=self.point_count; self.observe(x, y, h)
        if self.point_count>n: self._k_intent.append(kappa)
    def prune(self, s):
        n=self.point_count; super().prune(s); d=n-self.point_count
        if d: del self._k_intent[:d]
    def _refresh(self):
        dirty=self._dirty; super()._refresh()
        if dirty: self._curvature=list(self._k_intent)

class IntentTracker(BreadcrumbFormationTracker):
    def observe_leader(self, leader):
        if leader is self._last_observed: return
        self._last_observed=leader
        k=R.ROUTE.pose_at_s(leader.path.progress_s_mm, 0.0)[3]   # Leader's planned curvature now
        self.breadcrumbs.observe_intent(leader.status.x_mm, leader.status.y_mm, leader.status.heading_deg, k)

frames=R.leader_frames(R.RUN); frames18=R.leader_frames(R.OLD_RUN)
def run(frames, kind, lane, window=160.0, lag_s=0.4):
    plan=FormationPlan(offset_x_mm=-400, offset_y_mm=lane, follower_start_x_mm=-400, follower_start_y_mm=lane)
    tr={"analytic":lambda: PathFormationTracker(plan),
        "breadcrumb":lambda: BreadcrumbFormationTracker(plan, BreadcrumbPath(smoothing_window_mm=window)),
        "breadcrumb+intent":lambda: IntentTracker(plan, IntentPath(smoothing_window_mm=window))}[kind]()
    x=y=h=0.0; vl=vr=0.0; lat=[]; t=frames[0].received_monotonic_s; k=0
    while t<=frames[-1].received_monotonic_s:
        while k+1<len(frames) and frames[k+1].received_monotonic_s<=t: k+=1
        if kind!="analytic": tr.observe_leader(frames[k])
        c=tr.control(replace(frames[k], received_monotonic_s=t), status(x,y,math.degrees(h)))
        for _ in range(5):
            a=min(1.0,0.02/lag_s); vl+=a*(c.left_mm_per_second-vl); vr+=a*(c.right_mm_per_second-vr)
            v,w=(vl+vr)/2,(vr-vl)/130; x+=v*math.cos(h)*.02; y+=v*math.sin(h)*.02; h+=w*.02
        lat.append(abs(R.ROUTE.project(x-400, y+lane, lane)[1])); t+=0.1
    return f"{max(lat):.1f}/{np.median(lat):.1f}"
for name, fr in (("11/24 Leader", frames), ("11/18 Leader", frames18)):
    print(name)
    for kind, w in (("analytic",160),("breadcrumb",160),("breadcrumb",300),("breadcrumb+intent",160),("breadcrumb+intent",300)):
        print(f"  {kind:18s} window {w}: F1 {run(fr,kind,400,w)}  F2 {run(fr,kind,-400,w)}")
