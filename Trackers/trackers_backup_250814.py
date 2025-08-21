import sys, time
import threading

import numpy as np
import openvr

############################## Class ##############################
class Trackers:
    '''
    trackers = Trackers() # init
    trackers.print_info_all() 
    # {'idx': 1, 'role': 'left_hand', 't': [0.0, 0.0, 0.0], 'q': [0.0, 0.0, 0.0, 0.0], 'serial': '53-A33503456', 'model': 'VIVE Ultimate Tracker 1', 'type': 'vive_ultimate_tracker'}
    # trackers.start_get_poses() # start get pose thread(default: auto_start=True)

    trackers.print_poses_enable = True # enable print trackers pos, quat
    [left_hand] pos=[+0.057783, +0.494660, -0.163734] quat=[+0.654288, +0.292908, -0.304005, +0.627449]
    time.sleep(5)

    t, q = trackers.get_tracker_transform("left_hand") # get trackers pos, quat
    print(t, q)
    # [np.float64(0.05778329074382782), np.float64(0.4946601986885071), np.float64(-0.1637340635061264)] [0.6542882641961044, 0.29290816397643943, -0.304004754360601, 0.6274494359879317]
    time.sleep(5)

    trackers.print_poses_enable = False

    trackers.disconnect()
    '''
    tracker_name =[
        "left_hand" ,
        "right_hand",
        "waist"     ,
        "left_foot" ,
        "right_foot"]

    tracker_serial_num = {
        tracker_name[0] : "53-A33503456",
        tracker_name[1] : "53-A33500591",
        tracker_name[2] : "53-A33502625",
        tracker_name[3] : "53-A33500346",
        tracker_name[4] : "53-A33503087"}
    
    tracker_index_num = {
        tracker_name[0] : -1,
        tracker_name[1] : -1,
        tracker_name[2] : -1,
        tracker_name[3] : -1,
        tracker_name[4] : -1}
    
    def __init__(self, auto_start=True):
        self.auto_start = auto_start
        self.print_poses_enable = False
        self.get_data_interver_sec = 0.02

        self.maxN = 0
        self.tracker_ids = [] # 트래커 인덱스 수집 (GenericTracker)

        self.info = []
        self.tracker_num = 0

        self._stop_event = threading.Event()
        self._lock = threading.Lock()  # (선택) 읽기/쓰기 동시성 보호

        self.connect()

        self.deviceIdx_to_infoIdx = {}  # 디바이스 인덱스 -> self.info 인덱스
        for info_idx, item in enumerate(self.info):
            self.deviceIdx_to_infoIdx[item["idx"]] = info_idx

    def connect(self):
        try:
            # 렌더링 안 할 거면 Other/Background로 초기화
            openvr.init(openvr.VRApplication_Other)
            
        except openvr.error_code.InitError_VendorSpecific_OculusLinkNotEnabled:
            print("[Trackers] Oculus Link가 활성화되어 있지 않습니다. Oculus PC 앱에서 Link 기능을 켜고 다시 시도하세요.")
            exit(0)
        except openvr.error_code.InitError_Init_HmdNotFound:
            print("[Trackers] HMD(헤드셋)를 찾을 수 없습니다. 연결 상태를 확인하세요.")
        except openvr.error_code.InitError_Init_PathRegistryNotFound:
            print("[Trackers] OpenVR 런타임이 설치되지 않았거나 경로를 찾을 수 없습니다.")
        except openvr.OpenVRError as e:
            print(f"[Trackers] OpenVR 초기화 실패: {e}")

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
                        "role":"",
                        "t":[0.0, 0.0, 0.0],
                        "q":[0.0, 0.0, 0.0, 0.0],
                        "serial": serial,
                        "model": model,
                        "type": ctype,
                    })

            if not self.tracker_ids:
                print("[Trackers] 트래커가 감지되지 않았습니다. SteamVR에서 페어링/활성화 확인.")
                openvr.shutdown()
                return
            
            self.tracker_num = len(self.info)
            self.set_role()
            if self.auto_start:
                self.start_get_poses()
            
    def disconnect(self):
        if getattr(self, "thread", None) is not None:
            self._stop_event.set()
            self.thread.join(timeout=0.1)
        openvr.shutdown()
            
    def print_info_all(self):
        for i in self.info:
            print(i)

    def print_info_short(self):
        for item in self.info:
            #t_list = [float(v) for v in item['t']]  # np.float64 → float 변환
            t_str = ", ".join(f"{float(v):+.6f}" for v in item['t'])
            q_str = ", ".join(f"{float(v):+.6f}" for v in item['q'])
            print(f"[{item['role']}] pos=[{t_str}] quat=[{q_str}]")
            # print(f"[{item['role']}] {t_list}\t{item['q']}")
    
    def get_tracker_transform(self, role="left_hand"):
        idx = self.tracker_index_num[role]
        if idx == -1:
            print(f"[Trackers] {role} tracker is not connected")
            return [0.0]*3, [0.0]*4
        
        t_list = self.info[idx]["t"]
        q_list = self.info[idx]["q"]
        return t_list, q_list
        # for i in range(self.tracker_num):
        #     if role == self.info[i]["role"]:
        #         return self.info[i][role]

### Don't call the function from outside.
    def set_role(self):
        for i in range(self.tracker_num):
            for j in range(len(self.tracker_name)):
                if self.info[i]["serial"] == self.tracker_serial_num[self.tracker_name[j]]:
                    self.info[i]["role"] = self.tracker_name[j]
                    self.tracker_index_num[self.tracker_name[j]] = i
                    break
            # elif self.info[i]["serial"] == self.tracker_serial_num[self.tracker_name[1]]:
            #     self.info[i]["role"] = self.tracker_name[1]
            #     self.tracker_index_num[self.tracker_name[1]] = i
            # elif self.info[i]["serial"] == self.tracker_serial_num[self.tracker_name[2]]:
            #     self.info[i]["role"] = self.tracker_name[2]
            #     self.tracker_index_num[self.tracker_name[2]] = i
            # elif self.info[i]["serial"] == self.tracker_serial_num[self.tracker_name[3]]:
            #     self.info[i]["role"] = self.tracker_name[3]
            #     self.tracker_index_num[self.tracker_name[3]] = i
            # elif self.info[i]["serial"] == self.tracker_serial_num[self.tracker_name[4]]:
            #     self.info[i]["role"] = self.tracker_name[4]
            #     self.tracker_index_num[self.tracker_name[4]] = i

    def get_prop_string(self, vrsys, i, prop):
        try:
            return vrsys.getStringTrackedDeviceProperty(i, prop)
        except openvr.error_code.OpenVRError:
            return ""
        
    def get_poses(self):
        try:
            # Compositor 없이 시스템에서 직접 포즈를 얻는다
            # TrackingUniverseStanding(룸스케일) 기준, 예측시간 0초
            poses = self.vrsys.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0.0, self.maxN
            )
            # for i in self.tracker_ids:
            #     p = poses[i]
            #     if not p.bDeviceIsConnected or not p.bPoseIsValid:
            #         continue
            #     R, t = mat34_to_rt(p.mDeviceToAbsoluteTracking)
            #     q = r_to_quat(R)

            #     for j in range(3):
            #         self.info[i-1]["t"][j] = t[j]
            #     for j in range(4):
            #         self.info[i-1]["q"][j] = q[j]
            for i in self.tracker_ids:
                p = poses[i]
                if not p.bDeviceIsConnected or not p.bPoseIsValid:
                    continue

                R, t = mat34_to_rt(p.mDeviceToAbsoluteTracking)
                q = r_to_quat(R)

                info_idx = self.deviceIdx_to_infoIdx.get(i, None)
                if info_idx is None:
                    # 발견된 적 없는 디바이스라면 건너뜀
                    continue

                # numpy → float 강제 캐스팅(선택)
                self.info[info_idx]["t"][0] = float(t[0])
                self.info[info_idx]["t"][1] = float(t[1])
                self.info[info_idx]["t"][2] = float(t[2])
                self.info[info_idx]["q"][0] = float(q[0])
                self.info[info_idx]["q"][1] = float(q[1])
                self.info[info_idx]["q"][2] = float(q[2])
                self.info[info_idx]["q"][3] = float(q[3])

                # if self.print_poses_enable:
                    #self.print_info_short()
                    # print(f"[{i}] pos(m)=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) "
                    #     f"quat=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})")
            sys.stdout.flush()
        except:
            pass

    def get_poses_loop(self):
        while not self._stop_event.is_set():
            self.get_poses()
            if self.print_poses_enable:
                self.print_info_short()
            time.sleep(self.get_data_interver_sec)

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
    trackers = Trackers(auto_start=True)
    trackers.print_info_all()
    # trackers.start_get_poses()

    trackers.print_poses_enable = True
    time.sleep(5)

    t, q = trackers.get_tracker_transform("left_hand")
    print(t, q)
    time.sleep(5)

    trackers.print_poses_enable = False

    trackers.disconnect()

if __name__ == "__main__":
    main()
############################## END ##############################
############################## END ##############################