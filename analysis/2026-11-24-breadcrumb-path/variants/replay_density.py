import sys, math, time, numpy as np
from dataclasses import replace
from pathlib import Path; ROOT=str(Path(__file__).resolve().parents[3])
sys.path.insert(0, ROOT+"/pi_rover"); sys.path.insert(0, ROOT+"/analysis/2026-11-24-breadcrumb-path")
import replay_leader_csv as R
from breadcrumb_path import BreadcrumbPath
from follower_formation import BreadcrumbFormationTracker, FormationPlan
from simulate_formation import status
frames = R.leader_frames(R.RUN)
def run(spacing, window, lane, lag_s=0.4):
    plan=FormationPlan(offset_x_mm=-400, offset_y_mm=lane, follower_start_x_mm=-400, follower_start_y_mm=lane)
    tr=BreadcrumbFormationTracker(plan, BreadcrumbPath(spacing_mm=spacing, smoothing_window_mm=window))
    x=y=h=0.0; vl=vr=0.0; lat=[]; ct=[]; t=frames[0].received_monotonic_s; k=0
    while t<=frames[-1].received_monotonic_s:
        while k+1<len(frames) and frames[k+1].received_monotonic_s<=t: k+=1
        t0=time.perf_counter()
        tr.observe_leader(frames[k])
        c=tr.control(replace(frames[k], received_monotonic_s=t), status(x,y,math.degrees(h)))
        ct.append(time.perf_counter()-t0)
        for _ in range(5):
            a=min(1.0,0.02/lag_s); vl+=a*(c.left_mm_per_second-vl); vr+=a*(c.right_mm_per_second-vr)
            v,w=(vl+vr)/2,(vr-vl)/130; x+=v*math.cos(h)*.02; y+=v*math.sin(h)*.02; h+=w*.02
        lat.append(abs(R.ROUTE.project(x-400, y+lane, lane)[1])); t+=0.1
    return max(lat), np.median(lat), max(ct)*1000, tr.breadcrumbs.point_count
print("spacing window | F1 max/median | F2 max/median | worst ms per cycle (Mac) | points kept")
for window in (160, 300):
    for sp in (5, 10, 20, 40):
        if window < 2*sp: continue
        a=run(sp,window,400); b=run(sp,window,-400)
        print(f"{sp:4d} {window:4d}   | {a[0]:.1f}/{a[1]:.1f} | {b[0]:.1f}/{b[1]:.1f} | {max(a[2],b[2]):.2f} | {a[3]}")
