# Optional server integration

These files are integration patches for a compatible `xiaozhi-esp32-server`
deployment. They are not a complete standalone server.

- `http_server.py` exposes the StackChan MCP bridge and optional music helpers.
- `vision_handler.py` accepts authenticated camera uploads and forwards images
  to the configured vision model.
- `websocket_keepalive.patch` prevents compatible embedded clients from being
  disconnected by a missing WebSocket pong during long music playback.

The files intentionally contain no production address or secret. Configure all
credentials through environment variables and compare the files with the exact
server version you deploy before replacing anything.

Recommended minimum environment variables:

```dotenv
STACKCHAN_MCP_TOKEN=replace-with-a-long-random-value
```

Optional music integrations use additional `STACKCHAN_*` environment variables
referenced in `http_server.py`. Do not commit a populated `.env` file.

Deploy behind HTTPS and authentication. Back up the original server files and
perform a Python syntax check before restarting the service.

For long music playback, apply the keepalive patch from the root of the
compatible `xiaozhi-esp32-server` checkout:

```bash
git apply /path/to/server/websocket_keepalive.patch
```
