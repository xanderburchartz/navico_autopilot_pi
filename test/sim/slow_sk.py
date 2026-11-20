"""A Signal K server that answers late, for the plugin's HTTP client.

Logs each request as it arrives, then waits before answering: GET by the
first argument (s), PUT by the second. Local only; sends nothing anywhere.

    python3 test/sim/slow_sk.py 2 4
"""
import http.server
import sys
import threading
import time

GET_DELAY = float(sys.argv[1]) if len(sys.argv) > 1 else 0
PUT_DELAY = float(sys.argv[2]) if len(sys.argv) > 2 else 0
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 3999


class Handler(http.server.BaseHTTPRequestHandler):
    def answer(self, delay, body):
        time.sleep(delay)
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        print(f"{time.monotonic():.2f} GET {self.path}", flush=True)
        self.answer(GET_DELAY, b"{}")

    def do_PUT(self):
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n).decode()
        print(f"{time.monotonic():.2f} PUT {self.path} {body}", flush=True)
        self.answer(PUT_DELAY, b'{"state":"COMPLETED","statusCode":200}')

    def log_message(self, *args):
        pass


http.server.ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
