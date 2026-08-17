"""
teamNSSUR LoRa 텔레메트리 관제 프로그램 (수신기 PC 측)

LoRaReceiver.ino가 시리얼(USB)로 출력하는 CSV 라인을 읽어
1) ECUMASTER DataMaster 스타일의 실시간 대시보드(블랙 테마 + 상단 대형 숫자
   readout + 하단 그래프 + 수신 상태 OK/NOT OK 표시)로 보여주고
2) 타임스탬프가 붙은 CSV 파일로 저장한다.

LoRaReceiver.ino 출력 포맷 (한 줄당 한 패킷):
  FAST,vehicleId,seq,rpm,tpsX10,vss,rssi,snr
  SLOW,vehicleId,seq,gear,clt,battMv,fuelUsedX100,rssi,snr

필요 패키지 설치:
  pip install pyserial matplotlib

실행:
  python3 telemetry_viewer.py                         (MAC_SERIAL_PORT 사용)
  python3 telemetry_viewer.py --demo                  (강제 미리보기 모드)
  python3 telemetry_viewer.py --port /dev/cu.usbserial-5B090155161
  python3 telemetry_viewer.py --no-auto-demo          (포트 오류 시 자동 DEMO 전환 금지)
"""

import argparse
import csv
import math
import os
import random
import queue
import threading
import time
from collections import deque
from datetime import datetime

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch

# ---------------------------------------------------------------------------
# 사용자 설정
# ---------------------------------------------------------------------------
# macOS에서 실제 LoRa 수신기가 연결된 시리얼 포트를 지정하세요.
#
# 확인 명령:
#   ls /dev/cu.*
#
# 예:
#   /dev/cu.usbmodem1101
#   /dev/cu.usbserial-5B090151091
#
# 포트 번호가 바뀌면 아래 문자열만 수정하면 됩니다.
MAC_SERIAL_PORT = "/dev/cu.usbserial-5B090155161"

DEFAULT_PORT = MAC_SERIAL_PORT
DEFAULT_BAUD = 115200
DEFAULT_DEMO = False
DEMO_UPDATE_SEC = 0.10

PLOT_WINDOW_SEC = 60        # 그래프에 표시할 최근 구간(초)
MAX_HISTORY_POINTS = 5000   # 그래프용 롤링 버퍼 최대 길이 (CSV 저장에는 영향 없음)
PLOT_UPDATE_MS = 200        # 그래프 갱신 주기

# 그래프 Y축 고정 범위. 수신값에 따라 축이 확대/축소되지 않도록 한다.
RPM_Y_LIMITS = (0, 10000)
TPS_Y_LIMITS = (0, 100)
VSS_Y_LIMITS = (0, 160)
CLT_Y_LIMITS = (0, 150)
BATT_Y_LIMITS = (0, 16)
GEAR_Y_LIMITS = (0, 6)
FUEL_Y_LIMITS = (0, 20)

# 수신 상태(OK/NOT OK) 판정 기준
LINK_TIMEOUT_SEC = 2.0      # 이 시간 동안 패킷이 없으면 무조건 NOT OK
RSSI_OK_DBM = -110          # SX1276 SF7 기준 안정 수신 여유 마진
SNR_OK_DB = -5

CSV_HEADER = [
    "timestamp_iso", "elapsed_s", "msg_type", "vehicle_id", "seq",
    "rpm", "tps_pct", "vss_kmh",
    "gear", "clt_c", "batt_v", "fuel_used_l",
    "rssi", "snr",
]

# ---------------------------------------------------------------------------
# ECUMASTER DataMaster 풍 다크 대시보드 색상
# ---------------------------------------------------------------------------
BG = "#000000"
PANEL_BG = "#0a0a0a"
GRID_COLOR = "#242424"
AXIS_COLOR = "#4a4a4a"
DIM_TEXT = "#7a7a7a"

C_RPM = "#ff3b30"
C_TPS = "#0a84ff"
C_VSS = "#30d158"
C_GEAR = "#ffd60a"
C_CLT = "#ff9f0a"
C_BATT = "#bf5af2"
C_FUEL = "#64d2ff"
C_OK = "#30d158"
C_NOT_OK = "#ff3b30"

plt.rcParams.update({
    "figure.facecolor": BG,
    "axes.facecolor": BG,
    "axes.edgecolor": AXIS_COLOR,
    "axes.labelcolor": "#e0e0e0",
    "text.color": "#e0e0e0",
    "xtick.color": "#9a9a9a",
    "ytick.color": "#9a9a9a",
    "grid.color": GRID_COLOR,
    "font.family": "monospace",
})


def make_csv_path():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)
    return os.path.join(log_dir, f"telemetry_{ts}.csv")


def parse_line(line):
    """LoRaReceiver.ino의 CSV 한 줄을 dict로 변환. 형식이 안 맞으면 None 반환."""
    parts = line.split(",")
    if not parts:
        return None
    msg_type = parts[0]

    try:
        if msg_type == "FAST" and len(parts) == 8:
            _, vehicle_id, seq, rpm, tps_x10, vss, rssi, snr = parts
            return {
                "msg_type": "FAST",
                "vehicle_id": int(vehicle_id),
                "seq": int(seq),
                "rpm": int(rpm),
                "tps_pct": int(tps_x10) / 10.0,
                "vss_kmh": int(vss),
                "rssi": int(rssi),
                "snr": float(snr),
            }
        if msg_type == "SLOW" and len(parts) == 9:
            _, vehicle_id, seq, gear, clt, batt_mv, fuel_x100, rssi, snr = parts
            return {
                "msg_type": "SLOW",
                "vehicle_id": int(vehicle_id),
                "seq": int(seq),
                "gear": int(gear),
                "clt_c": int(clt),
                "batt_v": int(batt_mv) / 1000.0,
                "fuel_used_l": int(fuel_x100) / 100.0,
                "rssi": int(rssi),
                "snr": float(snr),
            }
    except ValueError:
        return None  # 깨진 라인 등 파싱 실패 -> 무시

    return None  # 알 수 없는 포맷 -> 무시


class SerialReader(threading.Thread):
    """백그라운드에서 시리얼을 읽어 파싱된 레코드를 큐에 넣는 스레드"""

    def __init__(self, port, baud, out_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as e:
            print(f"[ERROR] {self.port} 포트를 열 수 없습니다: {e}")
            self.out_queue.put(None)  # 메인 스레드에 종료 신호
            return

        print(f"[INFO] {self.port} @ {self.baud}bps 연결됨. 수신 대기 중...")
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException as e:
                print(f"[ERROR] 시리얼 읽기 실패: {e}")
                break
            if not line:
                continue
            record = parse_line(line)
            if record is not None:
                self.out_queue.put(record)

        if self.ser and self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.stop_event.set()


class DemoReader(threading.Thread):
    """시리얼 장치 없이 미리보기용 가상 텔레메트리를 생성하는 스레드"""

    def __init__(self, out_queue):
        super().__init__(daemon=True)
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.seq_fast = 0
        self.seq_slow = 0
        self.start_time = time.time()

    def run(self):
        print("[INFO] DEMO 모드: 시리얼 포트 없이 가상 텔레메트리를 생성합니다.")
        next_slow = 0.0

        while not self.stop_event.is_set():
            elapsed = time.time() - self.start_time

            # 가속/감속을 반복하는 차량 주행 패턴
            cycle = elapsed % 24.0
            if cycle < 8.0:
                throttle = cycle / 8.0
            elif cycle < 13.0:
                throttle = 1.0
            elif cycle < 21.0:
                throttle = 1.0 - ((cycle - 13.0) / 8.0)
            else:
                throttle = 0.0

            rpm = int(1500 + throttle * 7000 + 250 * math.sin(elapsed * 2.4))
            rpm = max(1200, min(9000, rpm))
            tps = max(0.0, min(100.0, throttle * 100.0 + random.uniform(-1.5, 1.5)))
            vss = int(max(0.0, throttle * 125.0 + 4.0 * math.sin(elapsed * 0.7)))

            fast_record = {
                "msg_type": "FAST",
                "vehicle_id": 1,
                "seq": self.seq_fast,
                "rpm": rpm,
                "tps_pct": round(tps, 1),
                "vss_kmh": vss,
                "rssi": int(-72 + random.uniform(-5, 5)),
                "snr": round(8.0 + random.uniform(-2, 2), 1),
            }
            self.seq_fast = (self.seq_fast + 1) & 0xFFFF
            self.out_queue.put(fast_record)

            if elapsed >= next_slow:
                gear = 0 if vss < 3 else min(6, max(1, int(vss / 22) + 1))
                clt = int(min(96, 24 + elapsed * 0.45 + throttle * 18))
                batt = 13.8 + 0.15 * math.sin(elapsed * 0.25)
                fuel = elapsed * (0.00016 + throttle * 0.00055)

                slow_record = {
                    "msg_type": "SLOW",
                    "vehicle_id": 1,
                    "seq": self.seq_slow,
                    "gear": gear,
                    "clt_c": clt,
                    "batt_v": round(batt, 3),
                    "fuel_used_l": round(fuel, 2),
                    "rssi": int(-72 + random.uniform(-5, 5)),
                    "snr": round(8.0 + random.uniform(-2, 2), 1),
                }
                self.seq_slow = (self.seq_slow + 1) & 0xFFFF
                self.out_queue.put(slow_record)
                next_slow = elapsed + 0.5

            time.sleep(DEMO_UPDATE_SEC)

    def stop(self):
        self.stop_event.set()


class TelemetryMonitor:
    def __init__(self, port, baud, demo=False):
        self.start_time = time.time()
        self.data_queue = queue.Queue()
        self.demo = demo
        self.reader = DemoReader(self.data_queue) if demo else SerialReader(port, baud, self.data_queue)

        self.csv_path = make_csv_path()
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        print(f"[INFO] CSV 저장 경로: {self.csv_path}")

        # 그래프용 롤링 버퍼: (elapsed_s, value)
        self.t_rpm = deque(maxlen=MAX_HISTORY_POINTS)
        self.rpm = deque(maxlen=MAX_HISTORY_POINTS)
        self.t_tps_vss = deque(maxlen=MAX_HISTORY_POINTS)
        self.tps = deque(maxlen=MAX_HISTORY_POINTS)
        self.vss = deque(maxlen=MAX_HISTORY_POINTS)
        self.t_slow = deque(maxlen=MAX_HISTORY_POINTS)
        self.clt = deque(maxlen=MAX_HISTORY_POINTS)
        self.batt = deque(maxlen=MAX_HISTORY_POINTS)
        self.gear = deque(maxlen=MAX_HISTORY_POINTS)
        self.fuel = deque(maxlen=MAX_HISTORY_POINTS)

        # 상단 readout에 표시할 최신값 (SLOW 필드는 마지막 값을 계속 유지)
        self.latest = {
            "rpm": 0, "tps": 0.0, "vss": 0,
            "gear": 0, "clt": 0, "batt": 0.0, "fuel": 0.0,
        }
        self.last_rssi = None
        self.last_snr = None
        self.last_packet_time = None

        self._build_figure()

    # -----------------------------------------------------------------
    # 대시보드 구성
    # -----------------------------------------------------------------
    def _build_figure(self):
        self.fig = plt.figure(figsize=(13, 8.5), facecolor=BG)
        try:
            self.fig.canvas.manager.set_window_title("teamNSSUR LoRa Telemetry Monitor")
        except AttributeError:
            pass

        gs = self.fig.add_gridspec(
            3, 2, height_ratios=[1.15, 2, 2], hspace=0.5, wspace=0.28,
            left=0.06, right=0.96, top=0.93, bottom=0.06,
        )

        self.ax_readout = self.fig.add_subplot(gs[0, :])
        self.ax_rpm = self.fig.add_subplot(gs[1, 0])
        self.ax_tv = self.fig.add_subplot(gs[1, 1])
        self.ax_clt_batt = self.fig.add_subplot(gs[2, 0])
        self.ax_gear_fuel = self.fig.add_subplot(gs[2, 1])

        self.fig.text(
            0.06, 0.965,
            "teamNSSUR  //  LoRa TELEMETRY" + ("  [DEMO]" if self.demo else ""),
            color="#e0e0e0",
            fontsize=15, fontweight="bold", family="monospace",
        )

        self._build_readout_panel()
        self._build_graphs()

    def _build_readout_panel(self):
        ax = self.ax_readout
        ax.set_facecolor(PANEL_BG)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis("off")

        # RPM, TPS, VSS, GEAR, CLT, BATT, FUEL + LINK 상태 = 8칸
        metric_defs = [
            ("rpm", "RPM", C_RPM),
            ("tps", "TPS %", C_TPS),
            ("vss", "VSS km/h", C_VSS),
            ("gear", "GEAR", C_GEAR),
            ("clt", "CLT C", C_CLT),
            ("batt", "BATT V", C_BATT),
            ("fuel", "FUEL L", C_FUEL),
        ]
        n_slots = len(metric_defs) + 1  # 마지막 슬롯은 LINK 상태
        self.readout_value_text = {}

        for i, (key, label, color) in enumerate(metric_defs):
            x = (i + 0.5) / n_slots
            ax.text(x, 0.80, label, color=DIM_TEXT, fontsize=10.5,
                    ha="center", va="center", family="monospace")
            value_text = ax.text(x, 0.32, "0", color=color, fontsize=27,
                                  fontweight="bold", ha="center", va="center",
                                  family="monospace")
            self.readout_value_text[key] = value_text

        # LINK 상태 칸 (마지막 슬롯) - 배경 박스로 강조
        link_x = (n_slots - 0.5) / n_slots
        box = FancyBboxPatch(
            (link_x - 0.075, 0.12), 0.15, 0.72,
            boxstyle="round,pad=0.01,rounding_size=0.02",
            transform=ax.transAxes,
            linewidth=1.5, edgecolor=C_NOT_OK, facecolor="#1a0000",
        )
        ax.add_patch(box)
        self.link_box = box

        ax.text(link_x, 0.80, "LINK", color=DIM_TEXT, fontsize=10.5,
                ha="center", va="center", family="monospace")
        self.link_status_text = ax.text(
            link_x, 0.42, "NOT OK", color=C_NOT_OK, fontsize=19,
            fontweight="bold", ha="center", va="center", family="monospace",
        )
        self.link_detail_text = ax.text(
            link_x, 0.20, "RSSI -- / SNR --", color=DIM_TEXT, fontsize=8.5,
            ha="center", va="center", family="monospace",
        )

        self.csv_label_text = self.fig.text(
            0.96, 0.965, "", color=DIM_TEXT, fontsize=8.5,
            ha="right", family="monospace",
        )

    def _build_graphs(self):
        self.line_rpm, = self.ax_rpm.plot([], [], color=C_RPM, linewidth=1.6)
        self.ax_rpm.set_ylabel("RPM")
        self.ax_rpm.set_ylim(*RPM_Y_LIMITS)
        self.ax_rpm.set_title("ENGINE RPM", color="#cfcfcf", fontsize=10, loc="left")
        self.ax_rpm.grid(True, alpha=0.5, linewidth=0.6)

        self.line_tps, = self.ax_tv.plot([], [], color=C_TPS, linewidth=1.6)
        self.ax_tv.set_ylabel("TPS %", color=C_TPS)
        self.ax_tv.set_ylim(*TPS_Y_LIMITS)
        self.ax_tv.tick_params(axis="y", colors=C_TPS)
        self.ax_tv.spines["left"].set_color(C_TPS)
        self.ax_tv.set_title("TPS / VSS", color="#cfcfcf", fontsize=10, loc="left")
        self.ax_tv.grid(True, alpha=0.5, linewidth=0.6)
        self.ax_vss = self.ax_tv.twinx()
        self.line_vss, = self.ax_vss.plot([], [], color=C_VSS, linewidth=1.6)
        self.ax_vss.set_ylabel("VSS km/h", color=C_VSS)
        self.ax_vss.set_ylim(*VSS_Y_LIMITS)
        self.ax_vss.tick_params(axis="y", colors=C_VSS)
        self.ax_vss.spines["right"].set_color(C_VSS)
        self.ax_vss.spines["left"].set_color(C_TPS)

        self.line_clt, = self.ax_clt_batt.plot([], [], color=C_CLT, linewidth=1.6)
        self.ax_clt_batt.set_ylabel("CLT C", color=C_CLT)
        self.ax_clt_batt.set_ylim(*CLT_Y_LIMITS)
        self.ax_clt_batt.tick_params(axis="y", colors=C_CLT)
        self.ax_clt_batt.spines["left"].set_color(C_CLT)
        self.ax_clt_batt.set_title("COOLANT / BATTERY", color="#cfcfcf", fontsize=10, loc="left")
        self.ax_clt_batt.grid(True, alpha=0.5, linewidth=0.6)
        self.ax_batt = self.ax_clt_batt.twinx()
        self.line_batt, = self.ax_batt.plot([], [], color=C_BATT, linewidth=1.6)
        self.ax_batt.set_ylabel("BATT V", color=C_BATT)
        self.ax_batt.set_ylim(*BATT_Y_LIMITS)
        self.ax_batt.tick_params(axis="y", colors=C_BATT)
        self.ax_batt.spines["right"].set_color(C_BATT)
        self.ax_batt.spines["left"].set_color(C_CLT)

        self.line_gear, = self.ax_gear_fuel.plot(
            [], [], color=C_GEAR, linewidth=1.6, drawstyle="steps-post"
        )
        self.ax_gear_fuel.set_ylabel("GEAR", color=C_GEAR)
        self.ax_gear_fuel.set_ylim(*GEAR_Y_LIMITS)
        self.ax_gear_fuel.set_yticks(range(GEAR_Y_LIMITS[0], GEAR_Y_LIMITS[1] + 1))
        self.ax_gear_fuel.tick_params(axis="y", colors=C_GEAR)
        self.ax_gear_fuel.spines["left"].set_color(C_GEAR)
        self.ax_gear_fuel.set_title("GEAR / FUEL USED", color="#cfcfcf", fontsize=10, loc="left")
        self.ax_gear_fuel.grid(True, alpha=0.5, linewidth=0.6)
        self.ax_fuel = self.ax_gear_fuel.twinx()
        self.line_fuel, = self.ax_fuel.plot([], [], color=C_FUEL, linewidth=1.6)
        self.ax_fuel.set_ylabel("FUEL L", color=C_FUEL)
        self.ax_fuel.set_ylim(*FUEL_Y_LIMITS)
        self.ax_fuel.tick_params(axis="y", colors=C_FUEL)
        self.ax_fuel.spines["right"].set_color(C_FUEL)
        self.ax_fuel.spines["left"].set_color(C_GEAR)

        for ax in (self.ax_rpm, self.ax_tv, self.ax_clt_batt, self.ax_gear_fuel):
            ax.set_xlabel("Elapsed (s)", color="#9a9a9a", fontsize=9)

    # -----------------------------------------------------------------
    # 데이터 수집
    # -----------------------------------------------------------------
    def _drain_queue(self):
        while True:
            try:
                record = self.data_queue.get_nowait()
            except queue.Empty:
                break
            if record is None:
                print("[ERROR] 시리얼 포트 연결 실패로 프로그램을 종료합니다.")
                plt.close(self.fig)
                return
            self._handle_record(record)

    def _handle_record(self, rec):
        elapsed = time.time() - self.start_time
        self.last_rssi = rec["rssi"]
        self.last_snr = rec["snr"]
        self.last_packet_time = time.time()

        row = {h: "" for h in CSV_HEADER}
        row["timestamp_iso"] = datetime.now().isoformat(timespec="milliseconds")
        row["elapsed_s"] = f"{elapsed:.3f}"
        row["msg_type"] = rec["msg_type"]
        row["vehicle_id"] = rec["vehicle_id"]
        row["seq"] = rec["seq"]
        row["rssi"] = rec["rssi"]
        row["snr"] = rec["snr"]

        if rec["msg_type"] == "FAST":
            row["rpm"] = rec["rpm"]
            row["tps_pct"] = rec["tps_pct"]
            row["vss_kmh"] = rec["vss_kmh"]

            self.latest["rpm"] = rec["rpm"]
            self.latest["tps"] = rec["tps_pct"]
            self.latest["vss"] = rec["vss_kmh"]

            self.t_rpm.append(elapsed)
            self.rpm.append(rec["rpm"])
            self.t_tps_vss.append(elapsed)
            self.tps.append(rec["tps_pct"])
            self.vss.append(rec["vss_kmh"])
        else:  # SLOW
            row["gear"] = rec["gear"]
            row["clt_c"] = rec["clt_c"]
            row["batt_v"] = rec["batt_v"]
            row["fuel_used_l"] = rec["fuel_used_l"]

            self.latest["gear"] = rec["gear"]
            self.latest["clt"] = rec["clt_c"]
            self.latest["batt"] = rec["batt_v"]
            self.latest["fuel"] = rec["fuel_used_l"]

            self.t_slow.append(elapsed)
            self.clt.append(rec["clt_c"])
            self.batt.append(rec["batt_v"])
            self.gear.append(rec["gear"])
            self.fuel.append(rec["fuel_used_l"])

        self.csv_writer.writerow([row[h] for h in CSV_HEADER])
        self.csv_file.flush()

    # -----------------------------------------------------------------
    # 수신 상태 판정 (송수신 OK / NOT OK)
    # -----------------------------------------------------------------
    def _link_ok(self):
        if self.last_packet_time is None:
            return False
        if time.time() - self.last_packet_time > LINK_TIMEOUT_SEC:
            return False
        if self.last_rssi is not None and self.last_rssi < RSSI_OK_DBM:
            return False
        if self.last_snr is not None and self.last_snr < SNR_OK_DB:
            return False
        return True

    # -----------------------------------------------------------------
    # 화면 갱신
    # -----------------------------------------------------------------
    @staticmethod
    def _update_line(ax, line, t_buf, y_buf, x_min, x_max):
        xs, ys = [], []
        for t, y in zip(t_buf, y_buf):
            if t >= x_min:
                xs.append(t)
                ys.append(y)
        line.set_data(xs, ys)
        ax.set_xlim(x_min, max(x_max, x_min + 1))

    def _update_readouts(self):
        self.readout_value_text["rpm"].set_text(f"{self.latest['rpm']:d}")
        self.readout_value_text["tps"].set_text(f"{self.latest['tps']:.1f}")
        self.readout_value_text["vss"].set_text(f"{self.latest['vss']:d}")
        gear_val = self.latest["gear"]
        self.readout_value_text["gear"].set_text("N" if gear_val == 0 else str(gear_val))
        self.readout_value_text["clt"].set_text(f"{self.latest['clt']:d}")
        self.readout_value_text["batt"].set_text(f"{self.latest['batt']:.2f}")
        self.readout_value_text["fuel"].set_text(f"{self.latest['fuel']:.2f}")

        ok = self._link_ok()
        if ok:
            self.link_status_text.set_text("OK")
            self.link_status_text.set_color(C_OK)
            self.link_box.set_edgecolor(C_OK)
        else:
            self.link_status_text.set_text("NOT OK")
            self.link_status_text.set_color(C_NOT_OK)
            self.link_box.set_edgecolor(C_NOT_OK)

        if self.last_rssi is not None:
            prefix = "DEMO / " if self.demo else ""
            self.link_detail_text.set_text(f"{prefix}RSSI {self.last_rssi}dBm / SNR {self.last_snr:.1f}dB")
        else:
            self.link_detail_text.set_text("RSSI -- / SNR --")

        self.csv_label_text.set_text(f"LOG: {os.path.basename(self.csv_path)}")

    def _update_plot(self, _frame):
        self._drain_queue()

        now = time.time() - self.start_time
        x_min = max(0.0, now - PLOT_WINDOW_SEC)

        self._update_line(self.ax_rpm, self.line_rpm, self.t_rpm, self.rpm, x_min, now)
        self._update_line(self.ax_tv, self.line_tps, self.t_tps_vss, self.tps, x_min, now)
        self._update_line(self.ax_vss, self.line_vss, self.t_tps_vss, self.vss, x_min, now)
        self._update_line(self.ax_clt_batt, self.line_clt, self.t_slow, self.clt, x_min, now)
        self._update_line(self.ax_batt, self.line_batt, self.t_slow, self.batt, x_min, now)
        self._update_line(self.ax_gear_fuel, self.line_gear, self.t_slow, self.gear, x_min, now)
        self._update_line(self.ax_fuel, self.line_fuel, self.t_slow, self.fuel, x_min, now)

        self._update_readouts()

        return (
            self.line_rpm, self.line_tps, self.line_vss,
            self.line_clt, self.line_batt, self.line_gear, self.line_fuel,
        )

    def run(self):
        self.reader.start()
        ani = animation.FuncAnimation(
            self.fig, self._update_plot, interval=PLOT_UPDATE_MS, cache_frame_data=False
        )
        try:
            plt.show()
        finally:
            self.reader.stop()
            self.csv_file.close()
            print(f"[INFO] 종료. CSV 저장 완료: {self.csv_path}")


def main():
    parser = argparse.ArgumentParser(description="teamNSSUR LoRa 텔레메트리 실시간 모니터")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"시리얼 포트 (기본값: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"보드레이트 (기본값: {DEFAULT_BAUD})")
    parser.add_argument(
        "--demo",
        action="store_true",
        default=DEFAULT_DEMO,
        help="시리얼 포트 없이 가상 데이터로 대시보드 미리보기",
    )
    parser.add_argument(
        "--no-auto-demo",
        action="store_true",
        help="시리얼 포트를 열 수 없을 때 DEMO 모드 자동 전환 금지",
    )
    args = parser.parse_args()

    demo_mode = args.demo

    if not demo_mode and not args.no_auto_demo:
        try:
            test_serial = serial.Serial(args.port, args.baud, timeout=0.2)
            test_serial.close()
        except serial.SerialException as e:
            print(f"[WARN] {args.port} 포트를 열 수 없습니다: {e}")
            print("[INFO] DEMO 모드로 자동 전환합니다.")
            demo_mode = True

    monitor = TelemetryMonitor(args.port, args.baud, demo=demo_mode)
    monitor.run()


if __name__ == "__main__":
    main()
