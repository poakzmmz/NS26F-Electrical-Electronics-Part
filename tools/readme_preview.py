#!/usr/bin/env python3

import argparse
import html
import re
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"


def inline(text: str) -> str:
    placeholders = []

    def keep(value: str) -> str:
        placeholders.append(value)
        return f"\x00{len(placeholders) - 1}\x00"

    text = re.sub(
        r"!\[([^]]*)\]\(([^)]+)\)",
        lambda m: keep(
            f'<img src="/{html.escape(m.group(2), quote=True)}" '
            f'alt="{html.escape(m.group(1), quote=True)}">'
        ),
        text,
    )
    text = re.sub(
        r"\[([^]]+)\]\(([^)]+)\)",
        lambda m: keep(
            f'<a href="{html.escape(m.group(2), quote=True)}">'
            f'{html.escape(m.group(1))}</a>'
        ),
        text,
    )
    text = re.sub(
        r"`([^`]+)`", lambda m: keep(f"<code>{html.escape(m.group(1))}</code>"), text
    )
    text = html.escape(text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", text)

    for index, value in enumerate(placeholders):
        text = text.replace(f"\x00{index}\x00", value)
    return text


def render_markdown(source: str) -> str:
    lines = source.splitlines()
    output = []
    paragraph = []
    list_open = False
    index = 0

    def flush_paragraph() -> None:
        nonlocal paragraph
        if paragraph:
            output.append(f"<p>{inline(' '.join(part.strip() for part in paragraph))}</p>")
            paragraph = []

    def close_list() -> None:
        nonlocal list_open
        if list_open:
            output.append("</ul>")
            list_open = False

    while index < len(lines):
        line = lines[index]

        if line.startswith("```"):
            flush_paragraph()
            close_list()
            language = line[3:].strip()
            code = []
            index += 1
            while index < len(lines) and not lines[index].startswith("```"):
                code.append(lines[index])
                index += 1
            language_class = f' class="language-{html.escape(language)}"' if language else ""
            output.append(
                f"<pre><code{language_class}>{html.escape(chr(10).join(code))}</code></pre>"
            )
            index += 1
            continue

        if re.match(r"^<p\s+align=[\"']center[\"']>.*</p>$", line.strip()):
            flush_paragraph()
            close_list()
            output.append(line.strip())
            index += 1
            continue

        heading = re.match(r"^(#{1,6})\s+(.+)$", line)
        if heading:
            flush_paragraph()
            close_list()
            level = len(heading.group(1))
            output.append(f"<h{level}>{inline(heading.group(2))}</h{level}>")
            index += 1
            continue

        if (
            line.startswith("|")
            and index + 1 < len(lines)
            and re.match(r"^\|?[\s:|-]+\|?$", lines[index + 1])
        ):
            flush_paragraph()
            close_list()
            headers = [cell.strip() for cell in line.strip("|").split("|")]
            index += 2
            rows = []
            while index < len(lines) and lines[index].startswith("|"):
                rows.append([cell.strip() for cell in lines[index].strip("|").split("|")])
                index += 1
            output.append("<table><thead><tr>")
            output.extend(f"<th>{inline(cell)}</th>" for cell in headers)
            output.append("</tr></thead><tbody>")
            for row in rows:
                output.append("<tr>")
                output.extend(f"<td>{inline(cell)}</td>" for cell in row)
                output.append("</tr>")
            output.append("</tbody></table>")
            continue

        item = re.match(r"^\s*-\s+(.+)$", line)
        if item:
            flush_paragraph()
            if not list_open:
                output.append("<ul>")
                list_open = True
            output.append(f"<li>{inline(item.group(1))}</li>")
            index += 1
            continue

        if not line.strip():
            flush_paragraph()
            close_list()
        else:
            close_list()
            paragraph.append(line)
        index += 1

    flush_paragraph()
    close_list()
    return "\n".join(output)


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>README Preview</title>
<style>
:root { color-scheme: light dark; }
* { box-sizing: border-box; }
body { margin: 0; color: #1f2328; background: #f6f8fa; font: 16px/1.5 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
.bar { position: sticky; top: 0; z-index: 2; display: flex; justify-content: space-between; padding: 10px 18px; color: #fff; background: #24292f; }
.bar code { color: #fff; background: rgba(255,255,255,.12); }
.markdown-body { width: min(1012px, calc(100% - 32px)); margin: 28px auto; padding: 32px; background: #fff; border: 1px solid #d0d7de; border-radius: 6px; }
h1, h2 { padding-bottom: .3em; border-bottom: 1px solid #d8dee4; }
h1 { font-size: 2em; } h2 { margin-top: 24px; font-size: 1.5em; }
h3 { margin-top: 24px; font-size: 1.25em; }
a { color: #0969da; text-decoration: none; } a:hover { text-decoration: underline; }
img { max-width: 100%; height: auto; }
code { padding: .2em .4em; font: 85% ui-monospace, SFMono-Regular, Menlo, monospace; background: #eff1f3; border-radius: 6px; }
pre { overflow: auto; padding: 16px; background: #f6f8fa; border-radius: 6px; }
pre code { padding: 0; background: transparent; }
table { width: max-content; max-width: 100%; overflow: auto; border-spacing: 0; border-collapse: collapse; }
th, td { padding: 6px 13px; border: 1px solid #d0d7de; }
th { font-weight: 600; background: #f6f8fa; }
tr:nth-child(2n) { background: #f6f8fa; }
@media (prefers-color-scheme: dark) {
  body { color: #e6edf3; background: #0d1117; }
  .markdown-body { background: #0d1117; border-color: #30363d; }
  h1, h2, th, td { border-color: #30363d; }
  code, pre, th, tr:nth-child(2n) { background: #161b22; }
  a { color: #58a6ff; }
}
</style>
</head>
<body>
<div class="bar"><strong>README 미리보기</strong><span><code>README.md</code> 저장 시 자동 새로고침</span></div>
<main class="markdown-body">{{CONTENT}}</main>
<script>
let version = {{VERSION}};
setInterval(async () => {
  try {
    const next = Number(await (await fetch('/__mtime__', {cache: 'no-store'})).text());
    if (next !== version) location.reload();
  } catch (_) {}
}, 700);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        path = unquote(urlparse(self.path).path)
        if path == "/":
            version = README.stat().st_mtime_ns
            page = PAGE.replace("{{CONTENT}}", render_markdown(README.read_text(encoding="utf-8")))
            page = page.replace("{{VERSION}}", str(version))
            self.send_bytes(page.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/__mtime__":
            self.send_bytes(str(README.stat().st_mtime_ns).encode(), "text/plain")
            return

        target = (ROOT / path.lstrip("/")).resolve()
        try:
            target.relative_to(ROOT)
        except ValueError:
            self.send_error(403)
            return
        if not target.is_file():
            self.send_error(404)
            return
        content_type = "image/jpeg" if target.suffix.lower() in {".jpg", ".jpeg"} else "application/octet-stream"
        self.send_bytes(target.read_bytes(), content_type)

    def send_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="README.md local preview")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    url = f"http://127.0.0.1:{args.port}"
    print(f"README 미리보기: {url}")
    print("종료: Ctrl+C")
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
