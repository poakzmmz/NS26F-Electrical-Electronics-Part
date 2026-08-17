"""
teamNSSUR LoRa 텔레메트리 관제 프로그램 (수신기 PC 측)

LoRaReceiver.ino가 시리얼(USB)로 출력하는 CSV 라인을 읽어
1) 채널별 카드(현재값 + 스파크라인) 형태의 다크 대시보드로 보여주고
2) 타임스탬프가 붙은 CSV 파일로 저장한다.

채널마다 독립된 카드/축을 사용해 서로 다른 스케일의 값이 한 그래프에
겹치지 않도록 했다 (이전 버전은 트윈 축에 두 채널을 겹쳐 그려서 가독성이
떨어졌음). 카드 = 라벨 + 큰 숫자 readout + 하단 스파크라인.

LoRaReceiver.ino 출력 포맷 (한 줄당 한 패킷):
  FAST,vehicleId,seq,rpm,tpsX10,vss,rssi,snr
  SLOW,vehicleId,seq,gear,clt,battMv,fuelUsedX100,rssi,snr

필요 패키지 설치:
  pip install pyserial matplotlib

실행:
  python lora_monitor.py                 (기본: COM4, 115200bps)
  python lora_monitor.py --port COM5
"""

import argparse
import csv
import json
import os
import queue
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch
from matplotlib.colors import to_rgba
from matplotlib.widgets import Button

DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 115200
DEFAULT_WEB_PORT = 8765

PLOT_WINDOW_SEC = 60        # 스파크라인에 표시할 최근 구간(초)
MAX_HISTORY_POINTS = 5000   # 그래프용 롤링 버퍼 최대 길이 (CSV 저장에는 영향 없음)
PLOT_UPDATE_MS = 200        # 화면 갱신 주기

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
# 다크 / 화이트 테마 색상
# ---------------------------------------------------------------------------
THEMES = {
    "dark": {
        "bg": "#0c0c0e",
        "card_bg": "#17171a",
        "dim_text": "#8a8a8f",
        "label_text": "#c7c7cc",
    },
    "light": {
        "bg": "#f2f2f5",
        "card_bg": "#ffffff",
        "dim_text": "#6e6e73",
        "label_text": "#1c1c1e",
    },
}

# 초기(다크) 테마 기본값 - plt.rcParams 초기화 및 폴백용
BG = THEMES["dark"]["bg"]
CARD_BG = THEMES["dark"]["card_bg"]
DIM_TEXT = THEMES["dark"]["dim_text"]
LABEL_TEXT = THEMES["dark"]["label_text"]

# 채널별 강조 색상은 테마와 무관하게 고정 (다크/화이트 배경 모두에서 식별 가능하도록 선택됨)
C_RPM = "#ff5a4d"
C_TPS = "#3aa0ff"
C_VSS = "#3ee08a"
C_GEAR = "#ffd23f"
C_CLT = "#ff9f45"
C_BATT = "#b18cff"
C_FUEL = "#4fd3e8"
C_OK = "#3ee08a"
C_NOT_OK = "#ff5a4d"

plt.rcParams.update({
    "figure.facecolor": BG,
    "axes.facecolor": BG,
    "text.color": LABEL_TEXT,
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "Arial", "DejaVu Sans"],
})

# (key, 라벨, 단위, 색상, 시간버퍼 속성명, 값버퍼 속성명)
CARD_LAYOUT = [
    ("rpm", "RPM", "", C_RPM, "t_rpm", "rpm"),
    ("tps", "TPS", "%", C_TPS, "t_tps_vss", "tps"),
    ("vss", "VSS", "km/h", C_VSS, "t_tps_vss", "vss"),
    ("gear", "GEAR", "", C_GEAR, "t_slow", "gear"),
    ("clt", "CLT", "C", C_CLT, "t_slow", "clt"),
    ("batt", "BATT", "V", C_BATT, "t_slow", "batt"),
    ("fuel", "FUEL", "L", C_FUEL, "t_slow", "fuel"),
]
GRID_ROWS, GRID_COLS = 2, 4  # 채널 7개 + LINK 카드 = 8칸


def _format_tick(key, value):
    """스파크라인 y축 min/max 눈금 라벨 포맷 (채널별 자릿수)"""
    if key == "gear":
        return "N" if round(value) == 0 else f"{value:.0f}"
    if key in ("rpm", "vss", "clt"):
        return f"{value:.0f}"
    if key == "tps":
        return f"{value:.1f}"
    return f"{value:.2f}"  # batt, fuel


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


class WebBridge:
    """tools/rf_bridge.py를 서브프로세스로 띄우고, 파싱된 레코드를 JSON 한 줄씩
    그 표준입력으로 흘려보내 웹(ws://localhost:<port>)으로 브로드캐스트되게 한다.
    rf_bridge.py를 못 띄우거나 파이프가 끊기면 조용히 비활성화되고, 로컬
    matplotlib 대시보드/CSV 저장에는 영향을 주지 않는다."""

    def __init__(self, port):
        self.port = port
        self.proc = None
        self.enabled = False
        self._warned = False

        script = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "telemetry-dashboard", "tools", "rf_bridge.py",
        ))
        if not os.path.isfile(script):
            print(f"[WARN] {script} 가 없어 웹 송출을 건너뜁니다.")
            return

        try:
            self.proc = subprocess.Popen(
                [sys.executable, script, "--stdin", "--port", str(port)],
                stdin=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self.enabled = True
            print(f"[INFO] 웹 송출 브릿지 시작 (ws://localhost:{port})")
        except OSError as e:
            print(f"[WARN] 웹 송출 브릿지를 시작하지 못했습니다: {e}")

    def send(self, record):
        if not self.enabled:
            return
        try:
            self.proc.stdin.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError):
            self.enabled = False
            if not self._warned:
                print("[WARN] 웹 송출 브릿지 연결이 끊어졌습니다. 로컬 모니터링은 계속됩니다.")
                self._warned = True

    def stop(self):
        if self.proc is None:
            return
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        self.proc.terminate()


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


class TelemetryMonitor:
    def __init__(self, port, baud, web_port=None):
        self.start_time = time.time()
        self.data_queue = queue.Queue()
        self.reader = SerialReader(port, baud, self.data_queue)
        self.web_bridge = WebBridge(web_port) if web_port else None

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

        # 연비 계산용: VSS를 시간에 대해 적분한 누적 주행거리(km)
        self.total_distance_km = 0.0
        self.last_fast_elapsed = None

        # 카드에 표시할 최신값 (SLOW 필드는 마지막 수신값을 계속 유지)
        self.latest = {
            "rpm": 0, "tps": 0.0, "vss": 0,
            "gear": 0, "clt": 0, "batt": 0.0, "fuel": 0.0,
        }
        self.last_rssi = None
        self.last_snr = None
        self.last_packet_time = None

        self.theme = "dark"

        self.card_value_text = {}
        self.card_lines = {}
        self.card_fills = {}
        self.card_spark_axes = {}
        self.card_box_patches = {}
        self.card_outer_axes = {}
        self.dim_texts = []  # 테마 전환 시 DIM_TEXT 색으로 갱신할 텍스트들

        self._build_figure()

    # -----------------------------------------------------------------
    # 테마(다크/화이트) 전환
    # -----------------------------------------------------------------
    @property
    def colors(self):
        return THEMES[self.theme]

    def _toggle_theme(self, _event=None):
        self.theme = "light" if self.theme == "dark" else "dark"
        self._apply_theme()
        self.fig.canvas.draw_idle()

    def _on_key_press(self, event):
        if event.key and event.key.lower() == "t":
            self._toggle_theme()

    def _apply_theme(self):
        c = self.colors

        self.fig.patch.set_facecolor(c["bg"])
        self.header_team_text.set_color(c["label_text"])
        self.header_sub_text.set_color(c["dim_text"])

        for text in self.dim_texts:
            text.set_color(c["dim_text"])

        for ax in self.card_outer_axes.values():
            ax.set_facecolor(c["card_bg"])
        for box in self.card_box_patches.values():
            box.set_facecolor(c["card_bg"])

        for spark_ax in self.card_spark_axes.values():
            spark_ax.spines["left"].set_color(c["dim_text"])
            spark_ax.tick_params(axis="y", colors=c["dim_text"])

        next_label = "Dark Mode" if self.theme == "light" else "Light Mode"
        self.theme_button.label.set_text(next_label)
        self.theme_button_ax.set_facecolor(c["card_bg"])
        self.theme_button.color = c["card_bg"]
        self.theme_button.hovercolor = c["dim_text"]
        self.theme_button.label.set_color(c["label_text"])

        # fuel_econ_text 등 데이터에 따라 색이 바뀌는 텍스트도 즉시 재적용
        self._update_readouts()

    # -----------------------------------------------------------------
    # 대시보드 구성
    # -----------------------------------------------------------------
    def _build_figure(self):
        self.fig = plt.figure(figsize=(15, 7.5), facecolor=BG)
        try:
            self.fig.canvas.manager.set_window_title("teamNSSUR LoRa Telemetry Monitor")
        except AttributeError:
            pass

        self.header_team_text = self.fig.text(
            0.025, 0.965, "teamNSSUR", color=LABEL_TEXT, fontsize=14, fontweight="bold"
        )
        self.header_sub_text = self.fig.text(
            0.025, 0.938, "LoRa TELEMETRY MONITOR", color=DIM_TEXT, fontsize=9.5
        )
        self.csv_label_text = self.fig.text(
            0.975, 0.938, "", color=DIM_TEXT, fontsize=9, ha="right"
        )
        self.dim_texts.extend([self.header_sub_text, self.csv_label_text])

        # 다크/화이트 모드 전환 버튼 (우상단)
        self.theme_button_ax = self.fig.add_axes([0.895, 0.95, 0.08, 0.04])
        self.theme_button = Button(self.theme_button_ax, "Light Mode",
                                    color=CARD_BG, hovercolor=DIM_TEXT)
        self.theme_button.label.set_fontsize(8.5)
        self.theme_button.label.set_color(LABEL_TEXT)
        self.theme_button.on_clicked(self._toggle_theme)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)

        gs = self.fig.add_gridspec(
            GRID_ROWS, GRID_COLS, hspace=0.32, wspace=0.18,
            left=0.025, right=0.975, top=0.90, bottom=0.05,
        )

        for i, (key, label, unit, color, _t_attr, _v_attr) in enumerate(CARD_LAYOUT):
            row, col = divmod(i, GRID_COLS)
            ax = self.fig.add_subplot(gs[row, col])
            self.card_outer_axes[key] = ax
            self._build_metric_card(ax, key, label, unit, color)

        row, col = divmod(len(CARD_LAYOUT), GRID_COLS)
        self.ax_link = self.fig.add_subplot(gs[row, col])
        self.card_outer_axes["link"] = self.ax_link
        self._build_link_card(self.ax_link)

    @staticmethod
    def _style_card_axes(ax, border_color, border_alpha=0.55):
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.set_facecolor(CARD_BG)
        box = FancyBboxPatch(
            (0.012, 0.03), 0.976, 0.94,
            transform=ax.transAxes, clip_on=False,
            boxstyle="round,pad=0,rounding_size=0.045",
            linewidth=1.2,
            edgecolor=to_rgba(border_color, border_alpha),
            facecolor=CARD_BG,
        )
        ax.add_patch(box)
        return box

    def _build_metric_card(self, ax, key, label, unit, color):
        box = self._style_card_axes(ax, color)
        self.card_box_patches[key] = box

        title = f"{label}  {unit}".strip()
        title_text = ax.text(0.09, 0.82, title, transform=ax.transAxes, color=DIM_TEXT,
                              fontsize=11, va="center", ha="left")
        self.dim_texts.append(title_text)

        if key == "fuel":
            # 연료 카드에는 같은 박스 안, 제목 줄 우측에 평균 연비를 같이 표시
            self.fuel_econ_text = ax.text(
                0.93, 0.82, "-- km/L", transform=ax.transAxes, color=DIM_TEXT,
                fontsize=10.5, va="center", ha="right", fontweight="bold",
            )

        value_text = ax.text(
            0.09, 0.52, "--", transform=ax.transAxes, color=color,
            fontsize=30, fontweight="bold", va="center", ha="left",
            family="monospace",
        )
        self.card_value_text[key] = value_text

        # 스파크라인: x축은 숨기고, 왼쪽에 최소/최대 값만 보여주는 y축을 둔다.
        # 눈금 라벨이 카드 왼쪽 테두리에 잘리지 않도록 축 시작 위치를 살짝 안쪽으로 둠.
        spark_ax = ax.inset_axes([0.15, 0.10, 0.79, 0.26])
        spark_ax.set_facecolor("none")
        for side in ("top", "right", "bottom"):
            spark_ax.spines[side].set_visible(False)
        spark_ax.spines["left"].set_visible(True)
        spark_ax.spines["left"].set_color(DIM_TEXT)
        spark_ax.spines["left"].set_alpha(0.35)
        spark_ax.spines["left"].set_linewidth(0.8)
        spark_ax.set_xticks([])
        spark_ax.tick_params(axis="y", colors=DIM_TEXT, labelsize=7, length=2.5,
                              pad=2, width=0.8)
        spark_ax.set_yticks([])
        line, = spark_ax.plot([], [], color=color, linewidth=1.8, solid_capstyle="round")
        fill = spark_ax.fill_between([], [], color=color, alpha=0.12, linewidth=0)
        self.card_lines[key] = line
        self.card_fills[key] = fill
        self.card_spark_axes[key] = spark_ax

    def _build_link_card(self, ax):
        self.link_box = self._style_card_axes(ax, C_NOT_OK, border_alpha=0.9)
        self.card_box_patches["link"] = self.link_box

        link_title_text = ax.text(0.09, 0.82, "LINK", transform=ax.transAxes, color=DIM_TEXT,
                                   fontsize=11, va="center", ha="left")
        self.dim_texts.append(link_title_text)
        self.link_status_text = ax.text(
            0.09, 0.50, "NOT OK", transform=ax.transAxes, color=C_NOT_OK,
            fontsize=24, fontweight="bold", va="center", ha="left",
        )
        self.link_detail_text = ax.text(
            0.09, 0.16, "RSSI --   SNR --", transform=ax.transAxes,
            color=DIM_TEXT, fontsize=9.5, va="center", ha="left",
        )
        self.dim_texts.append(self.link_detail_text)

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
        if self.web_bridge is not None:
            self.web_bridge.send(rec)

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

            if self.last_fast_elapsed is not None:
                dt_h = (elapsed - self.last_fast_elapsed) / 3600.0
                avg_speed = (self.latest["vss"] + rec["vss_kmh"]) / 2.0
                self.total_distance_km += avg_speed * dt_h
            self.last_fast_elapsed = elapsed

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
    def _update_sparkline(self, key, t_attr, v_attr, x_min, x_max):
        t_buf = getattr(self, t_attr)
        v_buf = getattr(self, v_attr)
        xs, ys = [], []
        for t, v in zip(t_buf, v_buf):
            if t >= x_min:
                xs.append(t)
                ys.append(v)

        line = self.card_lines[key]
        spark_ax = self.card_spark_axes[key]
        line.set_data(xs, ys)
        spark_ax.set_xlim(x_min, max(x_max, x_min + 1))

        if ys:
            y_min, y_max = min(ys), max(ys)
            pad = (y_max - y_min) * 0.2 or max(abs(y_max), 1) * 0.1
            spark_ax.set_ylim(y_min - pad, y_max + pad)
            if y_max > y_min:
                spark_ax.set_yticks([y_min, y_max])
                spark_ax.set_yticklabels([_format_tick(key, y_min), _format_tick(key, y_max)])
            else:
                spark_ax.set_yticks([y_min])
                spark_ax.set_yticklabels([_format_tick(key, y_min)])
        else:
            spark_ax.set_ylim(0, 1)
            spark_ax.set_yticks([])

        self.card_fills[key].remove()
        if xs:
            self.card_fills[key] = spark_ax.fill_between(
                xs, ys, min(ys), color=line.get_color(), alpha=0.12, linewidth=0
            )
        else:
            self.card_fills[key] = spark_ax.fill_between([], [], color=line.get_color(), alpha=0.12)

    def _update_readouts(self):
        self.card_value_text["rpm"].set_text(f"{self.latest['rpm']:d}")
        self.card_value_text["tps"].set_text(f"{self.latest['tps']:.1f}")
        self.card_value_text["vss"].set_text(f"{self.latest['vss']:d}")
        gear_val = self.latest["gear"]
        self.card_value_text["gear"].set_text("N" if gear_val == 0 else str(gear_val))
        self.card_value_text["clt"].set_text(f"{self.latest['clt']:d}")
        self.card_value_text["batt"].set_text(f"{self.latest['batt']:.2f}")
        self.card_value_text["fuel"].set_text(f"{self.latest['fuel']:.2f}")

        fuel_used = self.latest["fuel"]
        if fuel_used > 0:
            economy = self.total_distance_km / fuel_used
            self.fuel_econ_text.set_text(f"{economy:.1f} km/L")
            self.fuel_econ_text.set_color(C_FUEL)
        else:
            self.fuel_econ_text.set_text("-- km/L")
            self.fuel_econ_text.set_color(self.colors["dim_text"])

        ok = self._link_ok()
        status_color = C_OK if ok else C_NOT_OK
        self.link_status_text.set_text("OK" if ok else "NOT OK")
        self.link_status_text.set_color(status_color)
        self.link_box.set_edgecolor(to_rgba(status_color, 0.9))

        if self.last_rssi is not None:
            self.link_detail_text.set_text(f"RSSI {self.last_rssi} dBm   SNR {self.last_snr:.1f} dB")
        else:
            self.link_detail_text.set_text("RSSI --   SNR --")

        self.csv_label_text.set_text(f"LOG  {os.path.basename(self.csv_path)}")

    def _update_plot(self, _frame):
        self._drain_queue()

        now = time.time() - self.start_time
        x_min = max(0.0, now - PLOT_WINDOW_SEC)

        for key, _label, _unit, _color, t_attr, v_attr in CARD_LAYOUT:
            self._update_sparkline(key, t_attr, v_attr, x_min, now)

        self._update_readouts()

        return tuple(self.card_lines.values())

    def run(self):
        self.reader.start()
        ani = animation.FuncAnimation(
            self.fig, self._update_plot, interval=PLOT_UPDATE_MS, cache_frame_data=False
        )
        try:
            plt.show()
        finally:
            self.reader.stop()
            if self.web_bridge is not None:
                self.web_bridge.stop()
            self.csv_file.close()
            print(f"[INFO] 종료. CSV 저장 완료: {self.csv_path}")


def main():
    parser = argparse.ArgumentParser(description="teamNSSUR LoRa 텔레메트리 실시간 모니터")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"시리얼 포트 (기본값: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"보드레이트 (기본값: {DEFAULT_BAUD})")
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT,
                         help=f"웹 송출용 rf_bridge.py 포트 (기본값: {DEFAULT_WEB_PORT})")
    parser.add_argument("--no-web", action="store_true", help="웹 송출 브릿지를 띄우지 않음")
    args = parser.parse_args()

    monitor = TelemetryMonitor(args.port, args.baud, web_port=None if args.no_web else args.web_port)
    monitor.run()


if __name__ == "__main__":
    main()
