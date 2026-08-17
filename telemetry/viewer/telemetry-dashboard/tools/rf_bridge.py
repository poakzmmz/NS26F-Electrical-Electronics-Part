#!/usr/bin/env python3
"""
NS26F Telemetry — RF 수신 PC용 중계 서버
=========================================

차량 ──RF(LoRa)──▶ [이 PC] ──WebSocket──▶ 대시보드(여러 팀원 브라우저)

차량과 RF 통신하는 PC에서 이 스크립트를 실행하면, 수신한 텔레메트리를
웹소켓으로 뿌려줍니다. 팀원들은 대시보드 5번 탭에 이 PC의 주소를 넣고
연결하면 같은 값을 실시간으로 보게 됩니다.

표준 라이브러리만 사용합니다. (시리얼 입력을 쓸 때만 pyserial 필요)

사용법
------
  # 1) 하드웨어 없이 시험 — 가짜 데이터 생성
  python3 rf_bridge.py --sim

  # 2) 기존 로그 파일을 실시간처럼 재생 (UI 확인용)
  python3 rf_bridge.py --replay telemetry_20260801_095829.csv

  # 3) 실제 RF 수신기(시리얼 포트)에서 읽기
  python3 rf_bridge.py --serial /dev/tty.usbserial-0001 --baud 115200
  python3 rf_bridge.py --serial COM3 --baud 115200          # 윈도우

  # 4) 표준입력으로 받기 (다른 프로그램과 파이프 연결)
  my_rf_reader | python3 rf_bridge.py --stdin

접속 주소
---------
  같은 공유기(랙 텐트 와이파이): ws://<이 PC의 IP>:8765
  외부에서 접속시키려면 터널을 하나 띄우세요. 예)
      cloudflared tunnel --url http://localhost:8765
      ngrok http 8765
  터널이 https 주소를 주면 대시보드에는 wss:// 로 바꿔 넣습니다.

입력 형식
---------
  아래 중 아무거나 한 줄씩 들어오면 됩니다.
    · JSON      {"msg_type":"FAST","seq":12,"rpm":5200,"tps_pct":31.5,"rssi":-72,"snr":9.4}
    · CSV       먼저 '#'으로 헤더를 한 번 보내고, 이후 값만 콤마로
                #msg_type,seq,rpm,tps_pct,rssi,snr
                FAST,12,5200,31.5,-72,9.4
  인식하지 못한 줄은 조용히 버립니다.
"""

import argparse
import base64
import csv
import hashlib
import json
import math
import os
import random
import socket
import struct
import sys
import threading
import time
from collections import deque

WS_MAGIC = b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

# ---------------------------------------------------------------- 클라이언트 관리

class Hub:
    """접속한 클라이언트들에게 텔레메트리를 뿌리고, 폴링용 버퍼도 유지합니다."""

    def __init__(self, history=600):
        self.lock = threading.Lock()
        self.clients = []              # WebSocket 소켓 목록
        self.buffer = deque(maxlen=history)
        self.cursor = 0                # 폴링 클라이언트가 쓰는 단조 증가 번호
        self.total = 0

    def add(self, sock):
        with self.lock:
            self.clients.append(sock)
            n = len(self.clients)
        log(f'클라이언트 접속 (현재 {n}명)')

    def remove(self, sock):
        with self.lock:
            if sock in self.clients:
                self.clients.remove(sock)
            n = len(self.clients)
        try:
            sock.close()
        except OSError:
            pass
        log(f'클라이언트 해제 (현재 {n}명)')

    def broadcast(self, record):
        payload = json.dumps(record, ensure_ascii=False, separators=(',', ':'))
        frame = ws_frame(payload)
        with self.lock:
            self.cursor += 1
            self.total += 1
            self.buffer.append((self.cursor, record))
            targets = list(self.clients)
        dead = []
        for c in targets:
            try:
                c.sendall(frame)
            except OSError:
                dead.append(c)
        for c in dead:
            self.remove(c)

    def since(self, cur):
        with self.lock:
            if cur is None:
                return self.cursor, []
            return self.cursor, [r for (i, r) in self.buffer if i > cur]

    def snapshot_cursor(self):
        with self.lock:
            return self.cursor


HUB = Hub()
VERBOSE = False


def log(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


# ---------------------------------------------------------------- WebSocket 프레임

def ws_frame(text):
    """서버→클라이언트 텍스트 프레임 (마스킹 없음)."""
    data = text.encode('utf-8')
    n = len(data)
    head = bytearray([0x81])           # FIN + opcode 1 (text)
    if n < 126:
        head.append(n)
    elif n < 65536:
        head.append(126)
        head += struct.pack('>H', n)
    else:
        head.append(127)
        head += struct.pack('>Q', n)
    return bytes(head) + data


def ws_read_frame(sock):
    """클라이언트→서버 프레임을 하나 읽습니다. (opcode, payload) 반환."""
    def recv_exact(n):
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    hdr = recv_exact(2)
    if not hdr:
        return None, None
    opcode = hdr[0] & 0x0F
    masked = hdr[1] & 0x80
    ln = hdr[1] & 0x7F
    if ln == 126:
        ext = recv_exact(2)
        if not ext:
            return None, None
        ln = struct.unpack('>H', ext)[0]
    elif ln == 127:
        ext = recv_exact(8)
        if not ext:
            return None, None
        ln = struct.unpack('>Q', ext)[0]
    mask = recv_exact(4) if masked else None
    if masked and mask is None:
        return None, None
    payload = recv_exact(ln) if ln else b''
    if payload is None:
        return None, None
    if masked:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return opcode, payload


def handshake(sock, headers):
    key = headers.get('sec-websocket-key')
    if not key:
        return False
    accept = base64.b64encode(hashlib.sha1(key.encode() + WS_MAGIC).digest()).decode()
    sock.sendall(
        b'HTTP/1.1 101 Switching Protocols\r\n'
        b'Upgrade: websocket\r\n'
        b'Connection: Upgrade\r\n'
        b'Sec-WebSocket-Accept: ' + accept.encode() + b'\r\n\r\n'
    )
    return True


# ---------------------------------------------------------------- HTTP / WS 서버

def http_response(sock, body, ctype='application/json; charset=utf-8', status='200 OK'):
    data = body.encode('utf-8') if isinstance(body, str) else body
    sock.sendall(
        f'HTTP/1.1 {status}\r\n'
        f'Content-Type: {ctype}\r\n'
        f'Content-Length: {len(data)}\r\n'
        'Access-Control-Allow-Origin: *\r\n'
        'Cache-Control: no-store\r\n'
        'Connection: close\r\n\r\n'.encode() + data
    )


def handle_client(sock, addr):
    sock.settimeout(20)
    try:
        raw = b''
        while b'\r\n\r\n' not in raw:
            chunk = sock.recv(4096)
            if not chunk:
                sock.close()
                return
            raw += chunk
            if len(raw) > 65536:
                sock.close()
                return

        head = raw.split(b'\r\n\r\n', 1)[0].decode('latin-1')
        lines = head.split('\r\n')
        request = lines[0]
        headers = {}
        for line in lines[1:]:
            if ':' in line:
                k, v = line.split(':', 1)
                headers[k.strip().lower()] = v.strip()

        parts = request.split(' ')
        path = parts[1] if len(parts) > 1 else '/'

        # --- WebSocket 업그레이드 ---
        if headers.get('upgrade', '').lower() == 'websocket':
            if not handshake(sock, headers):
                sock.close()
                return
            sock.settimeout(None)
            HUB.add(sock)
            try:
                while True:
                    op, payload = ws_read_frame(sock)
                    if op is None or op == 0x8:      # 연결 종료
                        break
                    if op == 0x9:                    # ping → pong
                        pong = bytearray([0x8A, len(payload)]) + payload
                        sock.sendall(bytes(pong))
            except OSError:
                pass
            finally:
                HUB.remove(sock)
            return

        # --- SSE ---
        if path.startswith('/events'):
            sock.sendall(
                b'HTTP/1.1 200 OK\r\n'
                b'Content-Type: text/event-stream; charset=utf-8\r\n'
                b'Access-Control-Allow-Origin: *\r\n'
                b'Cache-Control: no-cache\r\n'
                b'Connection: keep-alive\r\n\r\n'
            )
            sock.settimeout(None)
            cur = HUB.snapshot_cursor()
            try:
                while True:
                    cur, recs = HUB.since(cur)
                    for r in recs:
                        line = json.dumps(r, ensure_ascii=False, separators=(',', ':'))
                        sock.sendall(f'data: {line}\n\n'.encode('utf-8'))
                    time.sleep(0.1)
            except OSError:
                pass
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
            return

        # --- HTTP 폴링 ---
        since = None
        if '?' in path:
            qs = path.split('?', 1)[1]
            for kv in qs.split('&'):
                if kv.startswith('since='):
                    try:
                        since = int(kv[6:])
                    except ValueError:
                        since = None
        cur, recs = HUB.since(since)
        http_response(sock, json.dumps({'cursor': cur, 'data': recs},
                                       ensure_ascii=False, separators=(',', ':')))
        sock.close()

    except (OSError, UnicodeDecodeError):
        try:
            sock.close()
        except OSError:
            pass


def serve(host, port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(32)

    ips = local_ips()
    log(f'중계 서버 시작 — 포트 {port}')
    log('대시보드 5번 탭에 아래 주소 중 하나를 입력하세요:')
    for ip in ips:
        log(f'    ws://{ip}:{port}')
    log(f'  (폴링 방식: http://{ips[0]}:{port}/   ·  SSE: http://{ips[0]}:{port}/events)')

    while True:
        try:
            sock, addr = srv.accept()
        except OSError:
            break
        threading.Thread(target=handle_client, args=(sock, addr), daemon=True).start()


def local_ips():
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    ips.append('localhost')
    return ips


# ---------------------------------------------------------------- 입력 소스

def parse_line(line, state):
    """RF 수신기가 뱉은 한 줄을 dict로 변환. 인식 못 하면 None."""
    line = line.strip()
    if not line:
        return None
    if line.startswith('#'):
        state['header'] = [h.strip() for h in line[1:].split(',')]
        return None
    if line[0] == '{':
        try:
            return json.loads(line)
        except ValueError:
            return None
    if state.get('header'):
        vals = line.split(',')
        rec = {}
        for i, h in enumerate(state['header']):
            if i >= len(vals):
                break
            v = vals[i].strip()
            if v == '':
                continue
            try:
                rec[h] = int(v)
            except ValueError:
                try:
                    rec[h] = float(v)
                except ValueError:
                    rec[h] = v
        return rec or None
    return None


def source_stdin():
    state = {}
    for line in sys.stdin:
        rec = parse_line(line, state)
        if rec:
            HUB.broadcast(rec)


def source_serial(port, baud):
    try:
        import serial  # pyserial
    except ImportError:
        log('오류: pyserial이 필요합니다.  pip install pyserial')
        sys.exit(1)
    log(f'시리얼 열기: {port} @ {baud}')
    ser = serial.Serial(port, baud, timeout=1)
    state = {}
    while True:
        try:
            raw = ser.readline().decode('utf-8', 'replace')
        except Exception as e:           # 케이블이 빠져도 죽지 않게
            log(f'시리얼 오류: {e} — 3초 후 재시도')
            time.sleep(3)
            try:
                ser.close()
                ser = serial.Serial(port, baud, timeout=1)
            except Exception:
                pass
            continue
        rec = parse_line(raw, state)
        if rec:
            HUB.broadcast(rec)
            if VERBOSE:
                log(str(rec))


def source_replay(path, speed):
    """CSV 로그를 원래 시간 간격대로 재생합니다."""
    with open(path, newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        log('재생할 데이터가 없습니다.')
        return
    tcol = 'elapsed_s' if 'elapsed_s' in rows[0] else None
    log(f'재생 시작: {path} ({len(rows)}행, {speed}배속)')

    while True:
        prev = None
        for row in rows:
            rec = {}
            for k, v in row.items():
                if v is None or v == '':
                    continue
                try:
                    rec[k] = int(v)
                except ValueError:
                    try:
                        rec[k] = float(v)
                    except ValueError:
                        rec[k] = v
            if tcol and tcol in rec:
                t = float(rec[tcol])
                if prev is not None:
                    dt = (t - prev) / max(speed, 0.01)
                    if 0 < dt < 5:          # 로그 중간의 긴 공백은 건너뜀
                        time.sleep(dt)
                prev = t
            else:
                time.sleep(0.24 / max(speed, 0.01))
            HUB.broadcast(rec)
        log('재생 끝 — 처음부터 반복')


def source_sim():
    """하드웨어 없이 실제 로그 통계를 흉내 낸 데이터를 생성합니다."""
    log('시뮬레이션 모드 (실제 RF 로그 통계 기반)')
    seq_f = seq_s = k = 0
    clt, fuel = 72.0, 0.0
    while True:
        k += 1
        phase = (k % 400) / 400.0
        tps = max(0.0, min(100.0, 55 + 45 * math.sin(phase * math.pi * 6 + 0.6) + random.uniform(-3, 3)))
        rpm = int(max(900, min(9200, 3200 + 5200 * (tps / 100) + 700 * math.sin(phase * math.pi * 12))))
        gear = max(1, min(4, round(1 + 3 * (tps / 100))))
        vss = int(max(0, 20 + 85 * (tps / 100)))
        clt = max(60.0, min(103.0, clt + (0.02 if tps > 60 else -0.015)))
        fuel += 0.0006 * (0.3 + tps / 100)

        dist = 0.5 + 0.5 * math.sin(phase * math.pi * 2)
        rssi = int(-55 - 62 * dist + random.uniform(-2.5, 2.5))
        snr = max(-9.5, min(12.5, 10.5 - 17 * max(0.0, dist - 0.55) / 0.45 + random.uniform(-0.6, 0.6)))
        loss_p = 0.01 if snr >= 8 else 0.04 if snr >= 6 else 0.10 if snr >= 3 else 0.22 if snr >= 0 else 0.40

        if k % 2:
            seq_f = (seq_f + 1) & 0xFF
            if random.random() > loss_p:
                HUB.broadcast({'msg_type': 'FAST', 'seq': seq_f, 'rpm': rpm,
                               'tps_pct': round(tps, 1), 'vss_kmh': vss,
                               'rssi': rssi, 'snr': round(snr, 1)})
        else:
            seq_s = (seq_s + 1) & 0xFF
            if random.random() > loss_p:
                HUB.broadcast({'msg_type': 'SLOW', 'seq': seq_s, 'gear': gear,
                               'clt_c': round(clt), 'batt_v': round(13.9 + 0.5 * math.sin(k / 90), 2),
                               'fuel_used_l': round(fuel, 2), 'rssi': rssi, 'snr': round(snr, 1)})
        time.sleep(0.118)


# ---------------------------------------------------------------- 진입점

def main():
    global VERBOSE
    ap = argparse.ArgumentParser(description='NS26F 텔레메트리 RF 중계 서버',
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--serial', metavar='PORT', help='RF 수신기 시리얼 포트')
    src.add_argument('--replay', metavar='CSV', help='CSV 로그를 실시간처럼 재생')
    src.add_argument('--stdin', action='store_true', help='표준입력에서 읽기')
    src.add_argument('--sim', action='store_true', help='가짜 데이터 생성 (장비 불필요)')
    ap.add_argument('--baud', type=int, default=115200, help='시리얼 보드레이트 (기본 115200)')
    ap.add_argument('--port', type=int, default=8765, help='서버 포트 (기본 8765)')
    ap.add_argument('--host', default='0.0.0.0', help='바인드 주소 (기본 0.0.0.0)')
    ap.add_argument('--speed', type=float, default=1.0, help='--replay 재생 배속 (기본 1.0)')
    ap.add_argument('-v', '--verbose', action='store_true', help='수신 내용 출력')
    args = ap.parse_args()
    VERBOSE = args.verbose

    threading.Thread(target=serve, args=(args.host, args.port), daemon=True).start()
    time.sleep(0.3)

    try:
        if args.serial:
            source_serial(args.serial, args.baud)
        elif args.replay:
            source_replay(args.replay, args.speed)
        elif args.stdin:
            source_stdin()
        else:
            source_sim()
    except KeyboardInterrupt:
        log('종료합니다.')


if __name__ == '__main__':
    main()
