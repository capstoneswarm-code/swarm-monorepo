from hashlib import sha256
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from threading import Thread

import numpy as np
from PIL import Image
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

PAGE = b"""<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot map</title>
<style>
body { font: 16px system-ui; background: #222; color: white; text-align: center; }
img { width: min(92vw, 900px); max-height: 80vh; object-fit: contain;
      image-rendering: pixelated; }
</style>
<h2>Robot map</h2>
<p id="state">Waiting for /map...</p>
<img id="map" alt="Robot occupancy map" hidden>
<script>
const image = document.getElementById("map");
const state = document.getElementById("state");
let tag = "", size = "";

async function refresh() {
  try {
    const response = await fetch("/map.png", {
      cache: "no-store",
      headers: tag ? {"If-None-Match": tag} : {}
    });

    if (response.status === 304) {
      state.textContent = "Latest map: " + size;
    } else if (response.status === 503) {
      state.textContent = "Waiting for /map...";
    } else if (response.ok) {
      const png = await response.blob();
      const old = image.src;
      image.src = URL.createObjectURL(png);
      image.hidden = false;
      if (old.startsWith("blob:")) URL.revokeObjectURL(old);
      tag = response.headers.get("ETag") || "";
      size = (png.size / 1024).toFixed(1) + " KiB per changed frame";
      state.textContent = "Latest map: " + size;
    } else {
      state.textContent = "Map request failed; retrying...";
    }
  } catch {
    state.textContent = "Connection interrupted; retrying...";
  } finally {
    setTimeout(refresh, 1000);
  }
}
refresh();
</script>
"""

latest = (None, None)
last_grid = None


def on_map(msg):
    global latest, last_grid

    w, h = msg.info.width, msg.info.height
    if not w or not h or len(msg.data) != w * h:
        return

    grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
    signature = (w, h, grid.tobytes())
    if signature == last_grid:
        return

    # Unknown = gray; free = white; occupied = black.
    # Flip rows because ROS grid rows start at the bottom.
    gray = np.where(grid < 0, 127, 255 - grid.astype(np.int16) * 255 // 100)
    output = BytesIO()
    Image.fromarray(gray[::-1].astype(np.uint8)).save(output, format="PNG")
    png = output.getvalue()
    latest = ('"' + sha256(png).hexdigest() + '"', png)
    last_grid = signature


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, body=b"", content_type="text/plain", etag=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        if status != 304:
            self.send_header("Content-Length", str(len(body)))
        if etag is not None:
            self.send_header("ETag", etag)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self.reply(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/map.png":
            tag, png = latest
            if png is None:
                self.reply(503)
            elif self.headers.get("If-None-Match") == tag:
                self.reply(304, etag=tag)
            else:
                self.reply(200, png, "image/png", tag)
        else:
            self.reply(404)

    def log_message(self, *_):
        pass


def main():
    rclpy.init()
    node = rclpy.create_node("web_map_view")
    subscription = node.create_subscription(
        OccupancyGrid, "/map", on_map,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )

    server = ThreadingHTTPServer(("127.0.0.1", 8765), Handler)
    Thread(target=server.serve_forever, daemon=True).start()
    print("Map viewer listening on port 8765", flush=True)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
