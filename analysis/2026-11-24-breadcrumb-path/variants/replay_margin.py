import sys, math, numpy as np
from dataclasses import replace
from pathlib import Path; ROOT=str(Path(__file__).resolve().parents[3])
sys.path.insert(0, ROOT+"/pi_rover"); sys.path.insert(0, ROOT+"/analysis/2026-11-24-breadcrumb-path")
import replay_leader_csv as R
from breadcrumb_path import BreadcrumbPath, LeaderTarget
from follower_formation import BreadcrumbFormationTracker, PathFormationTracker, FormationPlan
from simulate_formation import status
frames0 = R.leader_frames(R.RUN)
def with_target(frames, ahead):
    if ahead is None: return frames
    out=[]
    for f in frames:
        x,y,h,_ = R.ROUTE.pose_at_s(f.path.progress_s_mm+ahead, 0.0)
        out.append(replace(f, next_target=LeaderTarget(x,y,h,ahead)))
    return out
def run(frames, lag, source, lane, window=300.0, lag_s=0.4):
    plan=FormationPlan(offset_x_mm=-lag, offset_y_mm=lane, follower_start_x_mm=-lag, follower_start_y_mm=lane)
    tr = BreadcrumbFormationTracker(plan, BreadcrumbPath(smoothing_window_mm=window)) if source=="bc" else PathFormationTracker(plan)
    x=y=h=0.0; vl=vr=0.0; lat=[]; t=frames[0].received_monotonic_s; k=0
    while t<=frames[-1].received_monotonic_s:
        while k+1<len(frames) and frames[k+1].received_monotonic_s<=t: k+=1
        if source=="bc": tr.observe_leader(frames[k])
        c=tr.control(replace(frames[k], received_monotonic_s=t), status(x,y,math.degrees(h)))
        if not c.tracking_safe: return f"STOP {c.stop_reason}"
        for _ in range(5):
            a=min(1.0,0.02/lag_s); vl+=a*(c.left_mm_per_second-vl); vr+=a*(c.right_mm_per_second-vr)
            v,w=(vl+vr)/2,(vr-vl)/130; x+=v*math.cos(h)*.02; y+=v*math.sin(h)*.02; h+=w*.02
        lat.append(abs(R.ROUTE.project(x-lag, y+lane, lane)[1])); t+=0.1
    return max(lat)
print("lag | target | F1 analytic / breadcrumb | F2 analytic / breadcrumb   (max lateral vs ideal lane, mm, wheel lag 0.4 s)")
for lag in (400, 200, 100, 50):
    for ahead in (None, 300, 1000):
        fr=with_target(frames0, ahead)
        res=[run(fr,lag,s,lane) for lane in (400,-400) for s in ("an","bc")]
        fmt=lambda v: v if isinstance(v,str) else f"{v:.1f}"
        print(f"{lag:4d} | {str(ahead):5s} | {fmt(res[0])} / {fmt(res[1])} | {fmt(res[2])} / {fmt(res[3])}")
print("window sweep at lag 400, no target (breadcrumb F1/F2):")
for w in (160, 300, 500, 700):
    print(w, f"{run(frames0,400,'bc',400,w):.1f} / {run(frames0,400,'bc',-400,w):.1f}")
