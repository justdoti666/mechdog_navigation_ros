#!/usr/bin/env python3
# pi_report_view.py -- live report window: RGB + DEPTH + TRAVERSABILITY(2.5D) + status bar
# open http://<pi-ip>:8080/  (MJPEG live stream; record this pane)
# NOTE: all overlay text is ASCII on purpose (cv2.putText cannot draw CJK).
import threading, time, numpy as np, cv2
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

W_PANEL, H_PANEL = 360, 270
state = {"color": None, "depth": None, "terrain": None, "status": "waiting for data ..."}
lock = threading.Lock()

def be():
    q = QoSProfile(depth=1)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q

class Viewer(Node):
    def __init__(self):
        super().__init__("report_view")
        q = be()
        self.create_subscription(Image, "/camera/ir/image_raw", self.on_color, q)   # 彩色彩 HW 故障 → 用红外(同视场)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, q)
        self.create_subscription(Image, "/safety/terrain_map", self.on_terrain, q)
        self.create_subscription(String, "/safety/status_text", self.on_status, 10)

    def on_color(self, m):
        try:
            if m.encoding in ("rgb8", "bgr8"):
                a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == "rgb8":
                    a = a[:, :, ::-1]
            elif m.encoding == "mono8":
                g = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).astype(np.float32)
                lo, hi = float(np.percentile(g, 2)), float(np.percentile(g, 98))
                if hi > lo + 1.0:
                    g = np.clip((g - lo) / (hi - lo) * 255.0, 0, 255)      # 自动拉伸, 红外偏暗也能看清
                a = cv2.cvtColor(g.astype(np.uint8), cv2.COLOR_GRAY2BGR)
            elif m.encoding in ("mono16", "16UC1"):
                g = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32)
                g = np.clip(g / max(1.0, float(np.percentile(g, 99))) * 255.0, 0, 255).astype(np.uint8)
                a = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            else:
                return
            a = cv2.resize(a, (W_PANEL, H_PANEL))
            with lock: state["color"] = a
        except Exception:
            pass

    def on_depth(self, m):
        try:
            if m.encoding == "16UC1":
                d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32)
            elif m.encoding == "32FC1":
                d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width) * 1000.0
            else:
                return
            med = float(np.nanmedian(d))
            v = d.copy()
            v[(v < 200) | (v > 6000)] = np.nan
            v = np.clip((v - 300.0) / (4000.0 - 300.0), 0, 1)
            img = cv2.applyColorMap((np.nan_to_num(v) * 255).astype(np.uint8), cv2.COLORMAP_JET)
            img = cv2.resize(img, (W_PANEL, H_PANEL), interpolation=cv2.INTER_NEAREST)
            cv2.putText(img, f"median={med:.0f}mm", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
            with lock: state["depth"] = img
        except Exception:
            pass

    def on_terrain(self, m):
        try:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1].copy()
            img = cv2.resize(img, (W_PANEL, H_PANEL), interpolation=cv2.INTER_NEAREST)
            with lock: state["terrain"] = img
        except Exception:
            pass

    def on_status(self, m):
        with lock: state["status"] = str(m.data)

def panel_or_note(img, note):
    if img is not None:
        return img
    b = np.full((H_PANEL, W_PANEL, 3), 30, np.uint8)
    cv2.putText(b, note, (16, H_PANEL // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (90, 90, 230), 2)
    return b

def compose():
    with lock:
        c, d, t, s = state["color"], state["depth"], state["terrain"], state["status"]
    c = panel_or_note(c, "no IR / camera image yet")
    d = panel_or_note(d, "no depth yet")
    t = panel_or_note(t, "no terrain map yet (start safety_node)")
    blank = np.full((H_PANEL, W_PANEL, 3), 30, np.uint8)
    canvas = np.vstack([np.hstack([c, d]), np.hstack([t, blank])])
    titles = ["1) IR camera (mono, same FOV as depth)", "2) DEPTH  mm  (JET: blue=near, red=far)",
              "3) TRAVERSABILITY 2.5D  (green=GO, red=UP, blue=PIT, gray=unknown)", ""]
    for i, name in enumerate(titles):
        x = (i % 2) * W_PANEL + 10
        y = (i // 2) * H_PANEL + H_PANEL - 12
        cv2.putText(canvas, name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    bar = np.full((46, canvas.shape[1], 3), 45, np.uint8)
    cv2.putText(bar, s[:120], (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
    return np.vstack([canvas, bar])

REC = float(__import__("os").environ.get("RECORD_SECONDS", "0") or 0)
REC_STATE = {"writer": None, "t0": 0.0, "path": None, "done": False}

def record_frame(img):
    import os, time as _t
    if REC <= 0 or REC_STATE["done"]:
        return
    if REC_STATE["writer"] is None:
        os.makedirs("/home/chj", exist_ok=True)
        REC_STATE["path"] = "/home/chj/report_%s.mp4" % _t.strftime("%Y%m%d_%H%M%S")
        REC_STATE["writer"] = cv2.VideoWriter(REC_STATE["path"], cv2.VideoWriter_fourcc(*"mp4v"),
                                              12.0, (img.shape[1], img.shape[0]))
        REC_STATE["t0"] = _t.time()
        print("REC_START " + REC_STATE["path"], flush=True)
    cv2.circle(img, (26, 26), 9, (0, 0, 255), -1)
    cv2.putText(img, "REC", (44, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    REC_STATE["writer"].write(img)
    if _t.time() - REC_STATE["t0"] > REC:
        REC_STATE["writer"].release()
        REC_STATE["done"] = True
        print("REC_SAVED %s (%.1fs)" % (REC_STATE["path"], REC), flush=True)


def frames():
    while True:
        _img = compose()
        try:
            record_frame(_img)
        except Exception as _e:
            print("rec err", _e, flush=True)
        ok, buf = cv2.imencode(".jpg", _img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                   str(len(buf)).encode() + b"\r\n\r\n" + buf.tobytes() + b"\r\n")
        time.sleep(1.0 / 12.0)

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            html = (b"<html><head><title>mechdog live</title>"
                    b"<style>html,body{margin:0;height:100%;background:#111;overflow:hidden}"
                    b"img{width:100vw;height:100vh;object-fit:contain;display:block}</style>"
                    b"</head><body><img src='/stream.mjpg'></body></html>")
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html))); self.end_headers(); self.wfile.write(html)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                for ch in frames():
                    self.wfile.write(ch)
            except Exception:
                pass
        else:
            self.send_error(404)
    def log_message(self, *a):
        pass

def main():
    rclpy.init()
    node = Viewer()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    print("VIEW_READY http://0.0.0.0:8080/", flush=True)
    ThreadingHTTPServer(("0.0.0.0", 8080), Handler).serve_forever()

if __name__ == "__main__":
    main()
