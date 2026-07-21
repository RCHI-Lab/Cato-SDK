"""3D orientation visualizer — Python edition of Auli's imu_visualizer demo.

Streams fused orientation + linear acceleration from the SDK over a local
websocket to a three.js page. Requires the [examples] extra (websockets).

Usage: python examples/visualizer/serve.py
Then open http://localhost:8000 (opens automatically).
"""

import asyncio
import functools
import http.server
import json
import pathlib
import threading
import webbrowser

import websockets

from cato import Cato, DeviceNotFoundError

HTTP_PORT = 8000
WS_PORT = 8765
STATIC_DIR = pathlib.Path(__file__).parent / "static"


def start_http_server() -> http.server.ThreadingHTTPServer:
    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=str(STATIC_DIR)
    )
    handler.log_message = lambda *a, **k: None  # quiet
    httpd = http.server.ThreadingHTTPServer(("localhost", HTTP_PORT), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


async def handle_client(websocket, cato: Cato, counter: list) -> None:
    async def recv_loop() -> None:
        async for message in websocket:
            if message == "reset":
                cato.reset_orientation()

    recv_task = asyncio.create_task(recv_loop())
    try:
        while True:
            await asyncio.sleep(1 / 60)
            s = cato.latest()
            if s is None or s.quaternion is None:
                continue
            await websocket.send(
                json.dumps(
                    {
                        "q": s.quaternion,
                        "lin": s.linear_acc,
                        "acc": s.acc,
                        "gyro": s.gyro,
                        "n": counter[0],
                    }
                )
            )
    except websockets.ConnectionClosed:
        pass
    finally:
        recv_task.cancel()


async def main() -> None:
    try:
        cato = Cato()
        cato.connect()
    except DeviceNotFoundError as e:
        raise SystemExit(f"error: {e}")

    counter = [0]
    cato.on_sample(lambda _s: counter.__setitem__(0, counter[0] + 1))
    cato.on_disconnect(lambda exc: print(f"\nDevice disconnected: {exc or 'closed'}"))

    start_http_server()
    handler = functools.partial(handle_client, cato=cato, counter=counter)
    async with websockets.serve(handler, "localhost", WS_PORT):
        url = f"http://localhost:{HTTP_PORT}"
        print(f"Cato connected. Visualizer at {url} (Ctrl-C to stop)")
        webbrowser.open(url)
        try:
            await asyncio.Future()
        finally:
            cato.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nDone.")
