import sys, time
import threading

import numpy as np
import openvr

############################## Class ##############################
class Trackers:
    def __init__(self):
        self.info = []
        self.maxN = 0
        # 트래커 인덱스 수집 (GenericTracker)
        self.tracker_ids = []

        self.print_poses_enable = False

        self.connect()

    def connect(self):
        try:
            # 렌더링 안 할 거면 Other/Background로 초기화
            openvr.init(openvr.VRApplication_Other)
            
        except openvr.error_code.InitError_VendorSpecific_OculusLinkNotEnabled:
            print("[오류] Oculus Link가 활성화되어 있지 않습니다. Oculus PC 앱에서 Link 기능을 켜고 다시 시도하세요.")
            exit(0)

        except openvr.error_code.InitError_Init_HmdNotFound:
            print("[오류] HMD(헤드셋)를 찾을 수 없습니다. 연결 상태를 확인하세요.")

        except openvr.error_code.InitError_Init_PathRegistryNotFound:
            print("[오류] OpenVR 런타임이 설치되지 않았거나 경로를 찾을 수 없습니다.")

        except openvr.OpenVRError as e:
            print(f"[OpenVR 초기화 실패] {e}")

        else:
            self.vrsys = openvr.VRSystem()
            self.maxN = openvr.k_unMaxTrackedDeviceCount

            for i in range(self.maxN):
                if self.vrsys.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_GenericTracker:
                    model = self.get_prop_string(self.vrsys, i, openvr.Prop_ModelNumber_String)
                    serial = self.get_prop_string(self.vrsys, i, openvr.Prop_SerialNumber_String)
                    ctype = self.get_prop_string(self.vrsys, i, openvr.Prop_ControllerType_String)
                    # print(f"tracker idx={i} model={model} serial={serial} type={ctype}")
                    self.tracker_ids.append(i)
                    self.info.append({
                        "idx": int(i),
                        "model": model,
                        "serial": serial,
                        "type": ctype,
                    })

            if not self.tracker_ids:
                print("트래커가 감지되지 않았습니다. SteamVR에서 페어링/활성화 확인.")
                openvr.shutdown()
                return
            
    def disconnect(self):
        if getattr(self, "thread", None) is not None:
            self.thread.join(timeout=0.1)
        openvr.shutdown()
            
    def print_info(self):
        for i in self.info:
            print(i)
        
    def get_prop_string(self, vrsys, i, prop):
        try:
            return vrsys.getStringTrackedDeviceProperty(i, prop)
        except openvr.error_code.OpenVRError:
            return ""
        
    def get_poses(self):
        # Compositor 없이 시스템에서 직접 포즈를 얻는다
        # TrackingUniverseStanding(룸스케일) 기준, 예측시간 0초
        poses = self.vrsys.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0.0, self.maxN
        )
        for i in self.tracker_ids:
            p = poses[i]
            if not p.bDeviceIsConnected or not p.bPoseIsValid:
                continue
            R, self.t = mat34_to_rt(p.mDeviceToAbsoluteTracking)
            self.q = r_to_quat(R)
            if self.print_poses_enable:
                print(f"[{i}] pos(m)=({self.t[0]:+.3f},{self.t[1]:+.3f},{self.t[2]:+.3f}) "
                    f"quat=({self.q[0]:+.3f},{self.q[1]:+.3f},{self.q[2]:+.3f},{self.q[3]:+.3f})")
        sys.stdout.flush()

    def get_poses_loop(self):
        while True:
            self.get_poses()
            time.sleep(0.02)

    def start_get_poses(self):
        self.thread = threading.Thread(target=self.get_poses_loop, daemon=True)
        self.thread.start()

    

        


############################## Functions ##############################
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

############################## Test ##############################
def main():
    trackers = Trackers()
    trackers.print_info()
    trackers.start_get_poses()
    trackers.print_poses_enable = True
    time.sleep(5)
    trackers.print_poses_enable = False
    time.sleep(5)
    trackers.disconnect()




if __name__ == "__main__":
    main()
############################## END ##############################
############################## END ##############################