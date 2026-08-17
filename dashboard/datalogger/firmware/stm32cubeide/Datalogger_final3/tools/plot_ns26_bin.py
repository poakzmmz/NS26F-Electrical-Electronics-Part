#!/usr/bin/env python3
"""Decode NS26DLG1 binary logs and generate CSV plus an interactive HTML plot."""

from __future__ import annotations

import argparse
import csv
import json
import struct
from pathlib import Path


MAGIC = b"NS26DLG1"
HEADER = struct.Struct("<8sHHIIII")
ADC_RECORD = struct.Struct("<BBI6H")
TYPE4_RECORD = struct.Struct("<BBI4I")


def decode(path: Path) -> dict:
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise ValueError(f"{path}: file is shorter than the {HEADER.size}-byte header")

    magic, version, header_size, flags, circumference_mm, timer_hz, wheel_teeth = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError(f"{path}: invalid magic {magic!r}")
    if header_size < HEADER.size or header_size > len(data):
        raise ValueError(f"{path}: invalid header size {header_size}")

    adc = []
    type4 = []
    offset = header_size
    while offset < len(data):
        if offset + 2 > len(data) or data[offset] != 0xAA:
            raise ValueError(f"{path}: invalid record marker at offset {offset}")
        record_type = data[offset + 1]
        if record_type == 1:
            if offset + ADC_RECORD.size > len(data):
                raise ValueError(f"{path}: truncated ADC record at offset {offset}")
            _, _, timestamp_ms, *raw = ADC_RECORD.unpack_from(data, offset)
            adc.append({"timestamp_ms": timestamp_ms, "raw": raw})
            offset += ADC_RECORD.size
        elif record_type == 4:
            if offset + TYPE4_RECORD.size > len(data):
                raise ValueError(f"{path}: truncated type-4 record at offset {offset}")
            _, _, timestamp_ms, *values = TYPE4_RECORD.unpack_from(data, offset)
            type4.append({"timestamp_ms": timestamp_ms, "values": values})
            offset += TYPE4_RECORD.size
        else:
            raise ValueError(f"{path}: unknown record type {record_type} at offset {offset}")

    return {
        "name": path.name,
        "size": len(data),
        "version": version,
        "flags": flags,
        "circumference_mm": circumference_mm,
        "timer_hz": timer_hz,
        "wheel_teeth": wheel_teeth,
        "adc": adc,
        "type4": type4,
    }


def write_csv(logs: list[dict], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "timestamp_ms", "elapsed_s", *[f"adc{i}_raw" for i in range(1, 7)],
                         *[f"adc{i}_input_v_est" for i in range(1, 7)]])
        for log in logs:
            if not log["adc"]:
                continue
            start = log["adc"][0]["timestamp_ms"]
            for sample in log["adc"]:
                raw = sample["raw"]
                volts = [value * 5.1 / 4095.0 for value in raw]
                writer.writerow([log["name"], sample["timestamp_ms"],
                                 f"{(sample['timestamp_ms'] - start) / 1000.0:.3f}",
                                 *raw, *[f"{value:.4f}" for value in volts]])


def channel_stats(samples: list[dict]) -> list[dict]:
    if not samples:
        return []
    stats = []
    for index in range(6):
        values = [sample["raw"][index] for sample in samples]
        mean = sum(values) / len(values)
        stats.append({"channel": index + 1, "min": min(values), "max": max(values), "mean": mean})
    return stats


def write_html(logs: list[dict], path: Path) -> None:
    payload = []
    summaries = []
    for log in logs:
        samples = log["adc"]
        start = samples[0]["timestamp_ms"] if samples else 0
        payload.append({
            "name": log["name"],
            "time": [(sample["timestamp_ms"] - start) / 1000.0 for sample in samples],
            "channels": [[sample["raw"][i] for sample in samples] for i in range(6)],
        })
        duration = ((samples[-1]["timestamp_ms"] - start) / 1000.0) if samples else 0
        intervals = [samples[i]["timestamp_ms"] - samples[i - 1]["timestamp_ms"] for i in range(1, len(samples))]
        summaries.append({
            "name": log["name"], "size": log["size"], "count": len(samples),
            "type4_count": len(log["type4"]), "duration": duration,
            "mean_interval": (sum(intervals) / len(intervals)) if intervals else 0,
            "stats": channel_stats(samples),
        })

    html = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NS26 Datalogger BIN 분석</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;margin:0;background:#10151d;color:#e8edf4}main{max-width:1500px;margin:auto;padding:24px}.card{background:#18212d;border:1px solid #2b3949;border-radius:12px;padding:18px;margin:14px 0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}th,td{padding:7px 10px;border-bottom:1px solid #2b3949;text-align:right}th:first-child,td:first-child{text-align:left}.plot{height:560px}.note{color:#aeb9c8}h1,h2{margin:0 0 12px}</style></head>
<body><main><h1>NS26 Datalogger BIN 분석</h1><p class="note">ADC raw: 0–4095. 입력전압 추정치는 회로의 1.8kΩ/3.3kΩ 분압과 3.3V ADC 기준을 적용한 raw × 5.1 / 4095 입니다.</p>
<div id="summary" class="grid"></div><div class="card"><h2>ADC raw</h2><div id="raw" class="plot"></div></div><div class="card"><h2>추정 입력전압</h2><div id="volts" class="plot"></div></div>
<script>const logs=__PAYLOAD__;const summaries=__SUMMARIES__;const colors=['#55aaff','#ff7b72','#3ddc97','#ffd166','#c792ea','#ff9f43'];
document.getElementById('summary').innerHTML=summaries.map(s=>`<section class="card"><h2>${s.name}</h2><p>${s.size.toLocaleString()} bytes · ADC ${s.count.toLocaleString()}개 · type-4 ${s.type4_count.toLocaleString()}개<br>구간 ${s.duration.toFixed(3)} s · 평균 간격 ${s.mean_interval.toFixed(2)} ms</p>${s.stats.length?`<table><tr><th>채널</th><th>최소</th><th>평균</th><th>최대</th></tr>${s.stats.map(x=>`<tr><td>ADC${x.channel}</td><td>${x.min}</td><td>${x.mean.toFixed(1)}</td><td>${x.max}</td></tr>`).join('')}</table>`:'<p>데이터 레코드 없음</p>'}</section>`).join('');
const traces=[];logs.forEach(log=>log.channels.forEach((values,i)=>{if(values.length)traces.push({x:log.time,y:values,name:`${log.name} ADC${i+1}`,mode:'lines',line:{width:1.4,color:colors[i]},hovertemplate:'%{x:.3f} s<br>%{y} raw<extra>%{fullData.name}</extra>'})}));
const layout={paper_bgcolor:'#18212d',plot_bgcolor:'#18212d',font:{color:'#e8edf4'},margin:{l:65,r:25,t:20,b:55},xaxis:{title:'Elapsed time (s)',gridcolor:'#2b3949'},yaxis:{title:'ADC raw',range:[0,4095],gridcolor:'#2b3949'},legend:{orientation:'h'},hovermode:'x unified'};
Plotly.newPlot('raw',traces,layout,{responsive:true,displaylogo:false});
const vtraces=traces.map(t=>({...t,y:t.y.map(v=>v*5.1/4095),hovertemplate:'%{x:.3f} s<br>%{y:.4f} V<extra>%{fullData.name}</extra>'}));Plotly.newPlot('volts',vtraces,{...layout,yaxis:{title:'Estimated input voltage (V)',range:[0,5.1],gridcolor:'#2b3949'}},{responsive:true,displaylogo:false});
</script></main></body></html>"""
    html = html.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":")))
    html = html.replace("__SUMMARIES__", json.dumps(summaries, separators=(",", ":")))
    path.write_text(html, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/bin_analysis"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    logs = [decode(path) for path in args.inputs]
    write_csv(logs, args.output_dir / "decoded_adc.csv")
    write_html(logs, args.output_dir / "adc_plot.html")
    for log in logs:
        print(f"{log['name']}: {len(log['adc'])} ADC, {len(log['type4'])} type-4 records")


if __name__ == "__main__":
    main()
