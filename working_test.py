#!/usr/bin/env python
import sys, time
import numpy as np
import openvr

def mat34_to_rt(m):
    R = np.array([[m[0][0], m[0][1], m[0][2]],
                  [m[1][0], m[1][1], m[1][2]],
                  [m[2][0], m[2][1], m[2][2]]], dtype=float)
    t = np.array([m[0][3], m[1][3], m[2][3]], dtype=float)
    return R, t

def r_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
    return (float(x), float(y), float(z), float(w))

def get_prop_string(vrsys, i, prop):
    try:
        return vrsys.getStringTrackedDeviceProperty(i, prop)
    except openvr.error_code.OpenVRError:
        return ""

def main():
    # 렌더링 안 할 거면 Other/Background로 초기화
    openvr.init(openvr.VRApplication_Other)
    vrsys = openvr.VRSystem()

    # 트래커 인덱스 수집 (GenericTracker)
    tracker_ids = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vrsys.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_GenericTracker:
            model = get_prop_string(vrsys, i, openvr.Prop_ModelNumber_String)
            serial = get_prop_string(vrsys, i, openvr.Prop_SerialNumber_String)
            ctype = get_prop_string(vrsys, i, openvr.Prop_ControllerType_String)
            print(f"tracker idx={i} model={model} serial={serial} type={ctype}")
            tracker_ids.append(i)

    if not tracker_ids:
        print("트래커가 감지되지 않았습니다. SteamVR에서 페어링/활성화 확인.")
        openvr.shutdown()
        return

    print("\nReading poses for 100 seconds...")
    t0 = time.time()
    # 미리 pose 배열 준비
    maxN = openvr.k_unMaxTrackedDeviceCount
    while time.time() - t0 < 100.0:
        # Compositor 없이 시스템에서 직접 포즈를 얻는다
        # TrackingUniverseStanding(룸스케일) 기준, 예측시간 0초
        poses = vrsys.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0.0, maxN
        )
        for i in tracker_ids:
            p = poses[i]
            if not p.bDeviceIsConnected or not p.bPoseIsValid:
                continue
            R, t = mat34_to_rt(p.mDeviceToAbsoluteTracking)
            q = r_to_quat(R)
            print(f"[{i}] pos(m)=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) "
                  f"quat=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})")
        sys.stdout.flush()
        time.sleep(0.02)

    openvr.shutdown()

if __name__ == "__main__":
    main()
