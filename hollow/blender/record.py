# Tiny recorder: serves hollow/ and saves frames POSTed by index.html?film&record, then encodes with ffmpeg.
# Usage: python3 blender/record.py OUT.mp4 [port]
import base64, http.server, os, subprocess, sys, threading

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
OUT = os.path.abspath(sys.argv[1]); PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8766
FRAMES = os.path.join(os.path.dirname(OUT), '_frames'); os.makedirs(FRAMES, exist_ok=True)
done = threading.Event()

class H(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k): super().__init__(*a, directory=ROOT, **k)
    def log_message(self, *a): pass
    def do_POST(self):
        body = self.rfile.read(int(self.headers['Content-Length'] or 0))
        if self.path.startswith('/frame'):
            i = int(self.path.split('i=')[1])
            with open(os.path.join(FRAMES, f'{i:05d}.png'), 'wb') as f: f.write(base64.b64decode(body.split(b',', 1)[1]))
            if i % 24 == 0: print('frame', i, flush=True)
        elif self.path == '/done': done.set()
        self.send_response(200); self.end_headers()

srv = http.server.ThreadingHTTPServer(('127.0.0.1', PORT), H)
threading.Thread(target=srv.serve_forever, daemon=True).start()
print('ready', PORT, flush=True)
done.wait(); srv.shutdown()
subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '24', '-i', os.path.join(FRAMES, '%05d.png'),
                '-c:v', 'libx264', '-crf', '16', '-preset', 'slow', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', OUT], check=True)
print('wrote', OUT, flush=True)
