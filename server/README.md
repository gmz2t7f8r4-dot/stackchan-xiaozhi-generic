# Optional server integration

These files are integration patches for a compatible `xiaozhi-esp32-server`
deployment. They are not a complete standalone server.

- `http_server.py` exposes the StackChan MCP bridge and optional music helpers.
- `vision_handler.py` accepts authenticated camera uploads and forwards images
  to the configured vision model.

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
