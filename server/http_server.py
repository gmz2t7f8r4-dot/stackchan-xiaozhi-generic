import asyncio
import base64
import binascii
import difflib
import hashlib
import ipaddress
import json
import os
import random
import re
import socket
import time
import uuid
from datetime import datetime, timedelta
from pathlib import Path
from urllib.parse import urljoin, urlparse
from zoneinfo import ZoneInfo
import aiohttp
from aiohttp import web
from config.logger import setup_logging
from core.api.ota_handler import OTAHandler
from core.api.vision_handler import VisionHandler
from core.handle.sendAudioHandle import (
    _wait_for_audio_completion,
    sendAudioMessage,
    send_display_message,
    send_tts_message,
)
from core.providers.tools.device_mcp.mcp_handler import call_mcp_tool
from core.providers.tts.dto.dto import SentenceType
from plugins_func.functions.play_music import play_music

TAG = __name__
MUSIC_DIR = Path(os.environ.get("STACKCHAN_MUSIC_DIR", "/opt/xiaozhi-esp32-server/music"))
MUSIC_CACHE_DIR = Path(os.environ.get("STACKCHAN_MUSIC_CACHE_DIR", str(MUSIC_DIR / ".online-cache")))
MUSIC_RESOLVER_URL = os.environ.get("STACKCHAN_MUSIC_RESOLVER_URL", "").strip()
MUSIC_RESOLVER_TOKEN = os.environ.get("STACKCHAN_MUSIC_RESOLVER_TOKEN", "").strip()
NCM_API_URL = os.environ.get("STACKCHAN_NCM_API_URL", "").strip().rstrip("/")
TME_ACCESS_TOKEN = os.environ.get("STACKCHAN_TME_ACCESS_TOKEN", "").strip()
TME_DEVICE_ID = os.environ.get("STACKCHAN_TME_DEVICE_ID", "").strip()
TME_API_URL = "https://iot.cloud.tencent.com/api/exploreropen/tokenapi"
MUSIC_DOWNLOAD_LIMIT = int(os.environ.get("STACKCHAN_MUSIC_DOWNLOAD_LIMIT", str(30 * 1024 * 1024)))
LYRICS_LOOKUP_URL = os.environ.get("STACKCHAN_LYRICS_LOOKUP_URL", "https://lrclib.net/api/search").strip()
LYRICS_USER_AGENT = os.environ.get(
    "STACKCHAN_LYRICS_USER_AGENT", "StackChan-Xiaozhi-Generic/0.1"
).strip()
LYRICS_START_OFFSET = float(os.environ.get("STACKCHAN_LYRICS_START_OFFSET", "1.8"))
CAMERA_DIR = Path(os.environ.get("STACKCHAN_CAMERA_DIR", "/opt/xiaozhi-esp32-server/data/camera_mcp"))
CAMERA_LATEST_FILE = CAMERA_DIR / "latest_manual.txt"
FACE_DIR = Path(os.environ.get("STACKCHAN_FACE_DIR", "/opt/xiaozhi-esp32-server/data/stackchan_faces"))
MUSIC_ALARM_FILE = Path(os.environ.get(
    "STACKCHAN_MUSIC_ALARM_FILE",
    "/opt/xiaozhi-esp32-server/data/stackchan_music_alarms.json",
))
CHINA_TZ = ZoneInfo("Asia/Shanghai")
MUSIC_EXTENSIONS = {".mp3", ".wav", ".p3"}
ONLINE_CACHE_LOCK = asyncio.Lock()
LRC_TIMESTAMP = re.compile(r"\[(\d{1,3}):(\d{1,2})(?:[.:](\d{1,3}))?\]")


class SimpleHttpServer:
    SAFE_DEVICE_TOOL_PREFIXES = (
        "self_led_",
        "self_servo_",
        "self_motion_",
        "self_alarm_",
        "self_camera_",
        "self_face_",
        "self_audio_speaker_",
        "self_screen_",
        "self_music_",
    )
    # These tools are implemented by the currently deployed CoreS3 firmware.
    # Advertise their schemas even while the device's on-demand voice WebSocket
    # is idle, otherwise Operit validates the server while StackChan is asleep
    # and permanently sees only server-side tools.
    FIRMWARE_TOOLS = {
        "self_led_set_color",
        "self_led_turn_off",
        "self_led_auto",
        "self_motion_nod",
        "self_motion_shake_head",
        "self_motion_tilt_head",
        "self_motion_center",
        "self_motion_look_at",
        "self_motion_dance",
        "self_motion_stop_dance",
        "self_alarm_list",
        "self_alarm_cancel",
        "self_screen_set_emotion",
        "self_camera_take_photo",
    }
    SERVER_SIDE_TOOLS = {
        "self_music_list", "self_music_play", "self_music_stop",
        "self_music_next", "self_music_now_playing", "self_music_lyrics_status",
        "self_music_playlists", "self_music_playlist_tracks", "self_music_play_playlist",
        "self_alarm_create",
    }
    HIDDEN_DEVICE_TOOLS = {"self_alarm_set", "self_alarm_stop", "self_face_follow"}
    STATIC_TOOLS = [
        {"name": "self_led_set_color", "description": "Set StackChan's LED ring color.",
         "inputSchema": {"type": "object", "properties": {
             "r": {"type": "integer", "minimum": 0, "maximum": 255},
             "g": {"type": "integer", "minimum": 0, "maximum": 255},
             "b": {"type": "integer", "minimum": 0, "maximum": 255}},
             "required": ["r", "g", "b"]}},
        {"name": "self_led_turn_off", "description": "Turn off StackChan's LED ring.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_led_auto", "description": "Return StackChan's LED ring to automatic emotion colors.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_motion_nod", "description": "Make StackChan nod.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_motion_shake_head", "description": "Make StackChan shake its head.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_motion_tilt_head", "description": "Make StackChan tilt its head cutely.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_motion_center", "description": "Return StackChan's head to its neutral center position.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_motion_look_at", "description": "Turn StackChan's head to yaw and pitch angles.",
         "inputSchema": {"type": "object", "properties": {
             "yaw": {"type": "integer", "minimum": -45, "maximum": 45},
             "pitch": {"type": "integer", "minimum": 5, "maximum": 60}},
             "required": ["yaw", "pitch"]}},
        {"name": "self_motion_dance", "description": "让小机连续跳舞。可选预设，也可由当前助手编排逗号分隔的安全舞步。",
         "inputSchema": {"type": "object", "properties": {
             "sequence": {"type": "string", "default": "happy", "description": "预设 happy/cute/swing，或由 left/right/up/down/upper_left/upper_right/lower_left/lower_right/center 组成的逗号分隔序列"},
             "tempo_ms": {"type": "integer", "minimum": 180, "maximum": 1000, "default": 360},
             "repeat": {"type": "integer", "minimum": 1, "maximum": 5, "default": 2}}}},
        {"name": "self_motion_stop_dance", "description": "停止小机当前舞蹈并回正。",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_alarm_create", "description": "唯一的闹钟创建工具。普通铃声、本地音乐和在线音乐都用它；无论用户说具体日期还是几分钟后，都先换算成北京时间 YYYY-MM-DD HH:MM:SS。label 只负责屏幕文字，绝不决定是否响铃。",
         "inputSchema": {"type": "object", "properties": {
             "local_datetime": {"type": "string", "description": "北京时间 YYYY-MM-DD HH:MM:SS"},
             "sound_source": {"type": "string", "enum": ["alarm_tone", "local_music", "online_music"], "default": "alarm_tone"},
             "song_name": {"type": "string", "description": "本地或在线音乐名称；本地可用 random"},
             "label": {"type": "string", "default": "闹钟"},
             "repeat": {"type": "string", "enum": ["once", "daily", "weekdays", "weekly", "custom"], "default": "once"},
             "weekday_mask": {"type": "integer", "minimum": 0, "maximum": 127, "default": 0}},
             "required": ["local_datetime"]}},
        {"name": "self_alarm_list", "description": "List active alarms and timers.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_alarm_cancel", "description": "Cancel an alarm by id.",
         "inputSchema": {"type": "object", "properties": {"id": {"type": "integer"}}, "required": ["id"]}},
        {"name": "self_screen_set_emotion", "description": "Show a requested illustrated facial expression on StackChan. Use this when the user asks the assistant to look happy, sad, angry, surprised, sleepy, or thoughtful.",
         "inputSchema": {"type": "object", "properties": {
             "emotion": {"type": "string", "enum": ["neutral", "happy", "laughing", "funny", "loving", "embarrassed", "confident", "sad", "crying", "angry", "thinking", "surprised", "shocked", "confused", "sleepy"]}},
             "required": ["emotion"]}},
        {"name": "self_camera_take_photo", "description": "调用 StackChan 摄像头查看眼前内容。question 写清想从画面中了解什么，例如‘看看我手里是什么’。调用后 StackChan 会显示实时取景并等待用户按屏幕快门；18 秒内未按会自动拍摄，随后工具返回 JPEG 原图供当前 AI 查看。",
         "inputSchema": {"type": "object", "properties": {
              "question": {"type": "string", "default": "请简短描述这张照片。"}}}},
        {"name": "self_music_play", "description": "Play a named song through StackChan, or use random for a random local song.",
         "inputSchema": {"type": "object", "properties": {
             "song_name": {"type": "string", "default": "random"}}}},
        {"name": "self_music_list", "description": "List the local songs that StackChan can actually play.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_music_search", "description": "Search the configured licensed online music provider.",
         "inputSchema": {"type": "object", "properties": {
             "query": {"type": "string"},
             "limit": {"type": "integer", "minimum": 1, "maximum": 10, "default": 5}},
             "required": ["query"]}},
        {"name": "self_music_play_online", "description": "Search, cache and play a song from the configured licensed online provider.",
         "inputSchema": {"type": "object", "properties": {
             "query": {"type": "string"}}, "required": ["query"]}},
        {"name": "self_music_playlists", "description": "List the signed-in user's NetEase Cloud Music playlists. Present the playlist names and ask which one to play.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_music_playlist_tracks", "description": "List tracks from a NetEase Cloud Music playlist selected by exact name or playlist ID.",
         "inputSchema": {"type": "object", "properties": {
             "playlist": {"type": "string"},
             "limit": {"type": "integer", "minimum": 1, "maximum": 100, "default": 30}},
             "required": ["playlist"]}},
        {"name": "self_music_play_playlist", "description": "Load a NetEase Cloud Music playlist by exact name or playlist ID, start the first playable song, and automatically continue in list-loop order.",
         "inputSchema": {"type": "object", "properties": {
             "playlist": {"type": "string"}}, "required": ["playlist"]}},
        {"name": "self_music_stop", "description": "Stop the audio currently playing through StackChan.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_music_next", "description": "Play the next song in StackChan's current music queue.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_music_now_playing", "description": "Report StackChan's current or most recently selected song.",
         "inputSchema": {"type": "object", "properties": {}}},
        {"name": "self_music_lyrics_status", "description": "Report whether synchronized lyrics are available for the current song.",
         "inputSchema": {"type": "object", "properties": {}}},
    ]

    def __init__(self, config: dict, websocket_server=None):
        self.config = config
        self.websocket_server = websocket_server
        self.mcp_token = os.environ.get("STACKCHAN_MCP_TOKEN", "").strip()
        self.mcp_device_id = os.environ.get("STACKCHAN_DEVICE_ID", "").strip()
        self.logger = setup_logging()
        self.ota_handler = OTAHandler(config)
        self.vision_handler = VisionHandler(config)
        self.music_queue = []
        self.music_index = -1
        self.music_auto_advance = False
        self.music_lyrics = []
        self.music_lyrics_title = ""
        self.music_lyrics_source = "none"
        self._lyrics_generation = 0
        self._lyrics_future = None
        self._playback_generation = 0
        self._playback_finish_task = None
        self._music_switch_lock = asyncio.Lock()
        self._face_lock = asyncio.Lock()
        self.music_alarms = self._load_music_alarms()
        self._music_alarm_task = None

    def _mcp_authorized(self, request):
        if not self.mcp_token:
            return False
        return request.headers.get("Authorization", "") == f"Bearer {self.mcp_token}"

    @staticmethod
    def _jsonrpc_result(request_id, result):
        return web.json_response({"jsonrpc": "2.0", "id": request_id, "result": result})

    @staticmethod
    def _jsonrpc_error(request_id, code, message):
        return web.json_response(
            {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}}
        )

    @classmethod
    def _tool_allowed(cls, name):
        return any(name.startswith(prefix) for prefix in cls.SAFE_DEVICE_TOOL_PREFIXES)

    @staticmethod
    def _music_files():
        if not MUSIC_DIR.is_dir():
            return []
        return sorted(
            (
                path
                for path in MUSIC_DIR.rglob("*")
                if path.is_file()
                and path.suffix.lower() in MUSIC_EXTENSIONS
                # Online tracks are managed by the online queue.  Advertising
                # its cache as local music routes it through the legacy player
                # and can leave CoreS3 with lyrics but no audio.
                and MUSIC_CACHE_DIR not in path.parents
            ),
            key=lambda path: path.stem.casefold(),
        )

    @classmethod
    def _choose_music(cls, requested_name):
        files = cls._music_files()
        if not files:
            raise RuntimeError("服务器音乐目录中没有可播放的音源")
        requested = str(requested_name or "random").strip()
        if not requested or requested.casefold() in {"random", "随机", "随便"}:
            return random.choice(files)
        normalized = requested.casefold()
        for path in files:
            if normalized == path.stem.casefold() or normalized == path.name.casefold():
                return path
        for path in files:
            if normalized in path.stem.casefold() or path.stem.casefold() in normalized:
                return path
        matches = difflib.get_close_matches(normalized, [path.stem.casefold() for path in files], n=1, cutoff=0.35)
        if matches:
            return next(path for path in files if path.stem.casefold() == matches[0])
        available = "、".join(path.stem for path in files)
        raise RuntimeError(f"没有找到《{requested}》。可播放：{available}")

    @staticmethod
    async def _assert_public_https_url(url):
        parsed = urlparse(str(url))
        if parsed.scheme != "https" or not parsed.hostname:
            raise RuntimeError("音源地址必须是 HTTPS")
        try:
            infos = await asyncio.get_running_loop().getaddrinfo(
                parsed.hostname, parsed.port or 443, type=socket.SOCK_STREAM
            )
        except OSError as exc:
            raise RuntimeError("无法解析音源服务器") from exc
        if not infos:
            raise RuntimeError("无法解析音源服务器")
        for info in infos:
            address = ipaddress.ip_address(info[4][0])
            if not address.is_global:
                raise RuntimeError("拒绝访问内网或本机音源地址")

    @staticmethod
    def _resolver_headers():
        headers = {"Accept": "application/json"}
        if MUSIC_RESOLVER_TOKEN:
            headers["Authorization"] = f"Bearer {MUSIC_RESOLVER_TOKEN}"
        return headers

    @staticmethod
    def _online_music_available():
        return bool(NCM_API_URL or MUSIC_RESOLVER_URL or (TME_ACCESS_TOKEN and TME_DEVICE_ID))

    @staticmethod
    async def _ncm_get(path, params=None):
        if not NCM_API_URL:
            raise RuntimeError("尚未配置网易云音乐服务")
        timeout = aiohttp.ClientTimeout(total=25, connect=5)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.get(NCM_API_URL + path, params=params or {}) as response:
                payload = await response.json(content_type=None)
                if response.status != 200:
                    message = payload.get("error") if isinstance(payload, dict) else "未知错误"
                    raise RuntimeError(f"网易云音乐接口失败：{message}")
                return payload

    @classmethod
    async def _search_ncm_music(cls, query, limit):
        payload = await cls._ncm_get("/search", {
            "q": str(query).strip(),
            "limit": max(1, min(int(limit), 10)),
        })
        tracks = []
        for song in payload.get("songs", []) if isinstance(payload, dict) else []:
            if not isinstance(song, dict) or not str(song.get("id", "")).isdigit():
                continue
            tracks.append({
                "provider": "netease",
                "song_id": str(song["id"]),
                "title": str(song.get("name") or query).strip(),
                "artist": "、".join(str(value).strip() for value in song.get("artists", []) if str(value).strip()),
                "url": "",
                "headers": {},
            })
        if not tracks:
            raise RuntimeError(f"网易云没有找到《{query}》")
        return tracks

    @classmethod
    async def _ncm_playlists(cls):
        payload = await cls._ncm_get("/playlists", {"limit": 100})
        playlists = []
        for item in payload.get("playlists", []) if isinstance(payload, dict) else []:
            playlist_id = str(item.get("id") or "").strip()
            name = str(item.get("name") or "").strip()
            if not playlist_id or not name:
                continue
            playlists.append({
                "id": playlist_id,
                "name": name,
                "track_count": max(0, int(item.get("trackCount") or 0)),
            })
        if not playlists:
            raise RuntimeError("当前网易云账号没有可用歌单")
        return playlists

    @classmethod
    async def _select_ncm_playlist(cls, requested):
        requested = str(requested or "").strip()
        if not requested:
            raise RuntimeError("请提供歌单名称或歌单 ID")
        playlists = await cls._ncm_playlists()
        for item in playlists:
            if requested == item["id"] or requested.casefold() == item["name"].casefold():
                return item
        contained = [item for item in playlists if requested.casefold() in item["name"].casefold()]
        if len(contained) == 1:
            return contained[0]
        names = [item["name"] for item in playlists]
        matches = difflib.get_close_matches(requested, names, n=1, cutoff=0.45)
        if matches:
            return next(item for item in playlists if item["name"] == matches[0])
        raise RuntimeError("没有找到该歌单；可选歌单：" + "、".join(names))

    @classmethod
    async def _ncm_playlist_tracks(cls, playlist_id, limit=500):
        payload = await cls._ncm_get("/playlist", {
            "id": str(playlist_id),
            "limit": max(1, min(int(limit), 500)),
            "offset": 0,
        })
        tracks = []
        for song in payload.get("songs", []) if isinstance(payload, dict) else []:
            song_id = str(song.get("id") or "").strip()
            title = str(song.get("name") or "").strip()
            if not song_id.isdigit() or not title:
                continue
            tracks.append({
                "provider": "netease",
                "song_id": song_id,
                "title": title,
                "artist": "、".join(
                    str(value).strip() for value in song.get("artists", []) if str(value).strip()
                ),
                "url": "",
                "headers": {},
            })
        if not tracks:
            raise RuntimeError("该歌单中没有可读取的歌曲")
        return tracks

    @staticmethod
    def _load_music_alarms():
        try:
            payload = json.loads(MUSIC_ALARM_FILE.read_text(encoding="utf-8"))
            if isinstance(payload, list):
                return [item for item in payload if isinstance(item, dict)]
        except (OSError, ValueError):
            pass
        return []

    def _save_music_alarms(self):
        MUSIC_ALARM_FILE.parent.mkdir(parents=True, exist_ok=True)
        temporary = MUSIC_ALARM_FILE.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(self.music_alarms, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
        temporary.replace(MUSIC_ALARM_FILE)

    @staticmethod
    def _parse_china_datetime(value):
        try:
            parsed = datetime.strptime(str(value).strip(), "%Y-%m-%d %H:%M:%S")
        except ValueError as exc:
            raise RuntimeError("时间格式应为 YYYY-MM-DD HH:MM:SS") from exc
        parsed = parsed.replace(tzinfo=CHINA_TZ)
        if parsed <= datetime.now(CHINA_TZ):
            raise RuntimeError("闹钟时间必须晚于现在")
        return parsed

    @staticmethod
    def _next_music_alarm_time(record, after_timestamp):
        repeat = str(record.get("repeat") or "once")
        if repeat == "once":
            return 0
        current = datetime.fromtimestamp(float(record["next_at"]), CHINA_TZ)
        after = datetime.fromtimestamp(float(after_timestamp), CHINA_TZ)
        for days in range(1, 9):
            candidate = current + timedelta(days=days)
            matches = repeat == "daily"
            if repeat == "weekdays":
                matches = candidate.weekday() < 5
            elif repeat == "weekly":
                matches = candidate.weekday() == current.weekday()
            elif repeat == "custom":
                sunday_based = (candidate.weekday() + 1) % 7
                matches = bool(int(record.get("weekday_mask") or 0) & (1 << sunday_based))
            if matches and candidate > after:
                return int(candidate.timestamp())
        return 0

    async def _prepare_music_alarm_track(self, query):
        tracks = await self._search_online_music(str(query or "").strip(), 5)
        failures = []
        for track in tracks:
            try:
                cached = await self._download_online_track(track)
                prepared = dict(track)
                prepared["cached_path"] = str(cached)
                return prepared
            except Exception as exc:
                failures.append(str(exc))
        reason = failures[0] if failures else "没有可播放结果"
        raise RuntimeError(f"找到了歌曲，但无法作为闹铃缓存：{reason}")

    @staticmethod
    def _music_alarm_song_argument(arguments):
        """Accept argument names cached by different Operit MCP versions."""
        for key in ("song_name", "song", "title", "query"):
            value = str(arguments.get(key) or "").strip()
            if value:
                return value
        return ""

    async def _create_alarm(self, conn, arguments):
        repeat = str(arguments.get("repeat") or "once").strip()
        if repeat not in {"once", "daily", "weekdays", "weekly", "custom"}:
            raise RuntimeError("不支持的重复方式")
        weekday_mask = int(arguments.get("weekday_mask") or 0)
        if repeat == "custom" and weekday_mask == 0:
            raise RuntimeError("自选星期时必须提供 weekday_mask")
        when = self._parse_china_datetime(arguments.get("local_datetime", ""))
        source = str(arguments.get("sound_source") or "alarm_tone").strip()
        if source not in {"alarm_tone", "local_music", "online_music"}:
            raise RuntimeError("sound_source 只能是 alarm_tone、local_music 或 online_music")
        label = str(arguments.get("label") or "闹钟").strip()[:60]
        track = None
        song = self._music_alarm_song_argument(arguments)
        if source == "local_music":
            path = self._choose_music(song or "random")
            track = {
                "kind": "local", "title": path.stem, "artist": "",
                "cached_path": str(path),
            }
        elif source == "online_music":
            if not song:
                raise RuntimeError("在线音乐闹钟必须提供 song_name")
            track = await self._prepare_music_alarm_track(song)
            track["kind"] = "online"
        device_arguments = {
            "local_datetime": when.strftime("%Y-%m-%d %H:%M:%S"),
            "label": label,
            "repeat": repeat,
            "weekday_mask": weekday_mask,
            # Internal-only flag: wake the playback connection at fire time.
            "server_music": source != "alarm_tone",
        }
        device_result = await call_mcp_tool(
            conn, conn.mcp_client, "self_alarm_set", device_arguments
        )
        match = re.search(r"\bid\s*=?\s*(\d+)\b", str(device_result), re.IGNORECASE)
        if not match:
            raise RuntimeError(f"本地闹钟返回异常：{device_result}")
        alarm_id = int(match.group(1))

        if source == "alarm_tone":
            return f"闹钟已设置：{when.strftime('%Y-%m-%d %H:%M:%S')}，默认铃声，id {alarm_id}"
        self.music_alarms = [
            item for item in self.music_alarms if int(item.get("id") or -1) != alarm_id
        ]
        self.music_alarms.append({
            "id": alarm_id,
            "next_at": int(when.timestamp()),
            "label": label,
            "repeat": repeat,
            "weekday_mask": weekday_mask,
            "track": track,
        })
        self._save_music_alarms()
        title = track["title"] + (f" - {track['artist']}" if track.get("artist") else "")
        source_name = "本地音乐" if source == "local_music" else "在线音乐"
        return f"闹钟已设置：{when.strftime('%Y-%m-%d %H:%M:%S')}，{source_name}《{title}》，id {alarm_id}"

    async def _fire_music_alarm(self, record):
        track = dict(record.get("track") or {})
        try:
            cached = Path(str(track.get("cached_path") or ""))
            if track.get("kind") != "local" and (not cached.is_file() or cached.stat().st_size <= 1024):
                cached = await self._download_online_track(track)
            conn = None
            # The firmware opens its normally-idle WebSocket when a music alarm
            # fires. Allow that connection and MCP discovery time to complete.
            for _ in range(20):
                if self.websocket_server:
                    conn = await self.websocket_server.get_device_connection(
                        self.mcp_device_id or None
                    )
                if conn and getattr(conn, "mcp_client", None):
                    break
                await asyncio.sleep(0.5)
            if not conn or not getattr(conn, "mcp_client", None):
                self.logger.bind(tag=TAG).warning("音乐闹钟到点但设备离线，保留本地默认铃声")
                return
            title = track["title"] + (f" - {track['artist']}" if track.get("artist") else "")
            entries, source = await self._load_synced_lyrics(cached, title)
            async with self._music_switch_lock:
                await self._stop_audio(conn)
                # A device reconnect or a late MCP reply must never hold the
                # actual alarm song behind the tool's 30-second timeout.
                try:
                    await asyncio.wait_for(
                        call_mcp_tool(conn, conn.mcp_client, "self_alarm_stop", {}),
                        timeout=2.0,
                    )
                except Exception as exc:
                    self.logger.bind(tag=TAG).warning(
                        f"停止本地后备铃声超时，继续播放音乐闹铃: {exc}"
                    )
                if track.get("kind") == "local":
                    self.music_queue = [{"kind": "local", "path": cached, "title": title}]
                else:
                    self.music_queue = [{"kind": "online", "track": track}]
                self.music_index = 0
                await self._play_cached_file(conn, cached, title)
                self._start_lyrics(conn, entries, title, source)
        except Exception as exc:
            # Never dismiss the local fallback unless the online song is ready.
            self.logger.bind(tag=TAG).warning(f"音乐闹铃播放失败，保留本地铃声: {exc}")

    async def _music_alarm_loop(self):
        while True:
            await asyncio.sleep(1)
            now = int(time.time())
            due = []
            changed = False
            for record in list(self.music_alarms):
                next_at = int(record.get("next_at") or 0)
                if next_at <= 0 or next_at > now:
                    continue
                # A stale one-time mapping after a long outage must not ring late.
                if now - next_at <= 120:
                    due.append(dict(record))
                following = self._next_music_alarm_time(record, now)
                if following > 0:
                    record["next_at"] = following
                else:
                    self.music_alarms.remove(record)
                changed = True
            if changed:
                self._save_music_alarms()
            for record in due:
                asyncio.create_task(self._fire_music_alarm(record))

    @classmethod
    async def _resolve_ncm_track(cls, track):
        song, lyric = await asyncio.gather(
            cls._ncm_get("/song", {"id": track["song_id"]}),
            cls._ncm_get("/lyric", {"id": track["song_id"]}),
        )
        url = str(song.get("url") or "").strip()
        if not url:
            raise RuntimeError(f"《{track['title']}》当前账号没有完整播放权限")
        # NetEase CDN URLs are often returned as HTTP although the same host
        # supports HTTPS. Upgrade before the public-address safety check.
        if url.startswith("http://"):
            url = "https://" + url[len("http://"):]
        resolved = dict(track)
        resolved.update({
            "url": url,
            "lyrics": str(lyric.get("lyric") or ""),
            "translated_lyrics": str(lyric.get("translatedLyric") or ""),
        })
        return resolved

    @classmethod
    async def _tme_command(cls, command, params):
        """Call Tencent IoT Explorer's licensed Kugou/TME user command API."""
        if not TME_ACCESS_TOKEN or not TME_DEVICE_ID:
            raise RuntimeError("尚未配置腾讯音乐授权")
        import aiohttp
        timeout = aiohttp.ClientTimeout(total=20, connect=8)
        body = {
            "RequestId": str(uuid.uuid4()),
            "Action": "AppKugouUserCommand",
            "AccessToken": TME_ACCESS_TOKEN,
            "DeviceId": TME_DEVICE_ID,
            "KGCommand": command,
            "KugouParams": params,
        }
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(TME_API_URL, json=body) as response:
                if response.status != 200:
                    raise RuntimeError(f"腾讯音乐接口失败（HTTP {response.status}）")
                payload = await response.json(content_type=None)
        result = payload.get("Response", payload) if isinstance(payload, dict) else {}
        error_code = result.get("ErrorCode", -1)
        if str(error_code) != "0":
            message = str(result.get("ErrorMsg") or "未知错误").strip()
            if "AccessToken" in message or "token" in message.casefold():
                raise RuntimeError("腾讯音乐授权已失效，请重新授权")
            raise RuntimeError(f"腾讯音乐接口返回错误：{message}")
        data = result.get("Data")
        return data if isinstance(data, (dict, list)) else {}

    @classmethod
    async def _search_tme_music(cls, query, limit):
        data = await cls._tme_command("search_song", {
            "page": 1,
            "size": max(1, min(int(limit), 10)),
            "keyword": str(query).strip(),
        })
        songs = data.get("songs", []) if isinstance(data, dict) else []
        tracks = []
        for song in songs:
            if not isinstance(song, dict) or not song.get("song_id"):
                continue
            tracks.append({
                "provider": "tencent_tme",
                "song_id": str(song["song_id"]),
                "title": str(song.get("song_name") or query).strip(),
                "artist": str(song.get("singer_name") or "").strip(),
                "playable_code": int(song.get("playable_code", 9)),
                "url": "",
                "headers": {},
            })
        if not tracks:
            raise RuntimeError(f"腾讯音乐没有找到《{query}》")
        return tracks

    @classmethod
    async def _resolve_tme_track(cls, track):
        data = await cls._tme_command("song_url", {"song_id": track["song_id"]})
        if not isinstance(data, dict):
            raise RuntimeError("腾讯音乐没有返回歌曲信息")
        playable_code = int(data.get("playable_code", track.get("playable_code", 9)))
        if playable_code != 0:
            reasons = {
                1: "所在地区不可播放", 2: "歌曲暂无版权", 3: "需要音乐会员",
                4: "歌曲需要单独购买", 5: "当前账号无播放权限",
                6: "设备端版权受限", 9: "未知版权原因",
            }
            raise RuntimeError(f"《{track['title']}》无法完整播放：{reasons.get(playable_code, '无播放权限')}")
        # Prefer the standard stream for the CoreS3's small decoder and network buffer.
        url = str(data.get("song_url") or "").strip()
        if not url:
            raise RuntimeError(f"《{track['title']}》没有返回完整播放链接")
        resolved = dict(track)
        resolved.update({
            "title": str(data.get("song_name") or track["title"]).strip(),
            "artist": str(data.get("singer_name") or track.get("artist", "")).strip(),
            "url": url,
        })
        return resolved

    @classmethod
    async def _search_online_music(cls, query, limit=5):
        query = str(query).strip()
        if not query:
            raise RuntimeError("请说出歌曲名或歌手名")
        if NCM_API_URL:
            return await cls._search_ncm_music(query, limit)
        if TME_ACCESS_TOKEN and TME_DEVICE_ID:
            return await cls._search_tme_music(query, limit)
        if not MUSIC_RESOLVER_URL:
            raise RuntimeError("尚未配置有授权的在线音源服务")
        await cls._assert_public_https_url(MUSIC_RESOLVER_URL)
        import aiohttp
        timeout = aiohttp.ClientTimeout(total=20, connect=8)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(
                MUSIC_RESOLVER_URL,
                json={"query": str(query).strip(), "limit": max(1, min(int(limit), 10))},
                headers=cls._resolver_headers(),
            ) as response:
                if response.status != 200:
                    raise RuntimeError(f"在线音源搜索失败（HTTP {response.status}）")
                payload = await response.json(content_type=None)
        tracks = payload.get("tracks", payload if isinstance(payload, list) else [])
        normalized = []
        for item in tracks[:10]:
            if not isinstance(item, dict) or not item.get("url"):
                continue
            normalized.append({
                "title": str(item.get("title") or query).strip(),
                "artist": str(item.get("artist") or "").strip(),
                "url": str(item["url"]).strip(),
                "headers": item.get("headers") if isinstance(item.get("headers"), dict) else {},
            })
        if not normalized:
            raise RuntimeError(f"没有找到《{query}》的可播放正版音源")
        return normalized

    @classmethod
    async def _download_online_track(cls, track):
        import aiohttp
        if track.get("provider") == "netease":
            track = await cls._resolve_ncm_track(track)
        if track.get("provider") == "tencent_tme" and not track.get("url"):
            track = await cls._resolve_tme_track(track)
        url = track["url"]
        key = hashlib.sha256(
            (track["title"] + "\n" + track.get("artist", "") + "\n" + url).encode("utf-8")
        ).hexdigest()[:24]
        MUSIC_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        safe_title = re.sub(r"[^\w\u4e00-\u9fff -]+", "_", track["title"]).strip(" ._")[:60] or "在线歌曲"
        safe_artist = re.sub(r"[^\w\u4e00-\u9fff -]+", "_", track.get("artist", "")).strip(" ._")[:40]
        display_name = safe_title + (f" - {safe_artist}" if safe_artist else "")
        output = MUSIC_CACHE_DIR / f"{display_name} [{key}].mp3"
        sidecar = output.with_suffix(".lrc")
        synced_lyrics = str(track.get("lyrics") or "").strip()
        if synced_lyrics:
            try:
                sidecar.write_text(synced_lyrics + "\n", encoding="utf-8")
            except OSError:
                pass
        if output.is_file() and output.stat().st_size > 1024:
            return output
        async with ONLINE_CACHE_LOCK:
            if output.is_file() and output.stat().st_size > 1024:
                return output
            source = MUSIC_CACHE_DIR / f"{key}.download"
            current_url = url
            headers = {str(k): str(v) for k, v in track.get("headers", {}).items()}
            timeout = aiohttp.ClientTimeout(total=90, connect=10, sock_read=20)
            try:
                async with aiohttp.ClientSession(timeout=timeout) as session:
                    for _ in range(4):
                        await cls._assert_public_https_url(current_url)
                        async with session.get(current_url, headers=headers, allow_redirects=False) as response:
                            if response.status in {301, 302, 303, 307, 308}:
                                location = response.headers.get("Location")
                                if not location:
                                    raise RuntimeError("音源重定向缺少地址")
                                current_url = urljoin(current_url, location)
                                continue
                            if response.status != 200:
                                raise RuntimeError(f"下载音源失败（HTTP {response.status}）")
                            length = int(response.headers.get("Content-Length", "0") or 0)
                            if length > MUSIC_DOWNLOAD_LIMIT:
                                raise RuntimeError("歌曲文件超过缓存大小限制")
                            total = 0
                            with source.open("wb") as handle:
                                async for chunk in response.content.iter_chunked(64 * 1024):
                                    total += len(chunk)
                                    if total > MUSIC_DOWNLOAD_LIMIT:
                                        raise RuntimeError("歌曲文件超过缓存大小限制")
                                    handle.write(chunk)
                            break
                    else:
                        raise RuntimeError("音源重定向次数过多")
                process = await asyncio.create_subprocess_exec(
                    "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                    "-i", str(source), "-vn", "-ac", "1", "-ar", "16000",
                    "-b:a", "64k", str(output),
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.PIPE,
                )
                _, stderr = await process.communicate()
                if process.returncode != 0 or not output.is_file():
                    raise RuntimeError("在线歌曲转码失败：" + stderr.decode("utf-8", "ignore")[-200:])
                return output
            finally:
                try:
                    source.unlink(missing_ok=True)
                except OSError:
                    pass

    async def _play_queue_item(self, conn, item):
        if item.get("kind") == "local":
            entries, source = await self._load_synced_lyrics(item["path"], item["title"])
            await self._play_cached_file(conn, Path(item["path"]), item["title"])
            self._start_lyrics(conn, entries, item["title"], source)
            return item["title"]
        track = item["track"]
        cached = await self._download_online_track(track)
        title = track["title"] + (f" - {track['artist']}" if track.get("artist") else "")
        entries, source = await self._load_synced_lyrics(cached, title)
        await self._play_cached_file(conn, cached, title)
        self._start_lyrics(conn, entries, title, source)
        return title

    async def _play_available_queue_item(self, conn, start_index=0):
        if not self.music_queue:
            raise RuntimeError("当前没有播放队列，请先选择一首歌")
        failures = []
        for offset in range(len(self.music_queue)):
            index = (int(start_index) + offset) % len(self.music_queue)
            try:
                title = await self._play_queue_item(conn, self.music_queue[index])
                self.music_index = index
                return title
            except Exception as exc:
                failures.append(str(exc))
        reason = failures[0] if failures else "没有可播放结果"
        raise RuntimeError(f"搜索到了歌曲，但当前账号都无法完整播放：{reason}")

    async def _play_next_playlist_item(self, conn, finished_index):
        """Advance a playlist without falling back to the song that just ended."""
        queue_size = len(self.music_queue)
        if queue_size < 2:
            raise RuntimeError("歌单中没有下一首歌曲")
        failures = []
        for offset in range(1, queue_size):
            index = (int(finished_index) + offset) % queue_size
            try:
                title = await self._play_queue_item(conn, self.music_queue[index])
                self.music_index = index
                return title
            except Exception as exc:
                failures.append(str(exc))
        reason = failures[0] if failures else "后续歌曲均不可播放"
        raise RuntimeError(f"歌单后续歌曲均无法播放：{reason}")

    def _set_local_music_queue(self, selected):
        files = self._music_files()
        self.music_queue = [
            {"kind": "local", "title": path.stem, "path": str(path)} for path in files
        ]
        self.music_index = next(
            (index for index, item in enumerate(self.music_queue) if item["path"] == str(selected)),
            0,
        )

    @staticmethod
    def _parse_lrc(text):
        entries = []
        for raw_line in str(text or "").replace("\r", "").split("\n"):
            lyric = LRC_TIMESTAMP.sub("", raw_line).strip()
            if not lyric:
                continue
            for match in LRC_TIMESTAMP.finditer(raw_line):
                minutes = int(match.group(1))
                seconds = int(match.group(2))
                fraction = match.group(3) or "0"
                millis = int((fraction + "000")[:3])
                entries.append((minutes * 60 + seconds + millis / 1000.0, lyric))
        entries.sort(key=lambda item: item[0])
        deduped = []
        for timestamp, lyric in entries:
            if deduped and abs(deduped[-1][0] - timestamp) < 0.001 and deduped[-1][1] == lyric:
                continue
            deduped.append((timestamp, lyric))
        return deduped

    @classmethod
    async def _load_synced_lyrics(cls, audio_path, title):
        audio_path = Path(audio_path)
        sidecar = audio_path.with_suffix(".lrc")
        if sidecar.is_file():
            entries = cls._parse_lrc(sidecar.read_text(encoding="utf-8-sig", errors="replace"))
            if entries:
                return entries, "local-lrc"
        if not LYRICS_LOOKUP_URL:
            return [], "none"
        try:
            timeout = aiohttp.ClientTimeout(total=12)
            headers = {"User-Agent": LYRICS_USER_AGENT}
            async with aiohttp.ClientSession(timeout=timeout, headers=headers) as session:
                async with session.get(LYRICS_LOOKUP_URL, params={"q": title}) as response:
                    if response.status == 429:
                        return [], "rate-limited"
                    response.raise_for_status()
                    candidates = await response.json(content_type=None)
            normalized_title = re.sub(r"[\W_]+", "", title).casefold()
            best = None
            best_score = -1
            for candidate in candidates if isinstance(candidates, list) else []:
                synced = candidate.get("syncedLyrics")
                if not synced:
                    continue
                candidate_title = str(candidate.get("trackName") or "")
                normalized_candidate = re.sub(r"[\W_]+", "", candidate_title).casefold()
                score = difflib.SequenceMatcher(None, normalized_title, normalized_candidate).ratio()
                if normalized_title == normalized_candidate:
                    score += 1
                if score > best_score:
                    best_score, best = score, synced
            # Do not attach an unrelated song merely because it has a vaguely
            # similar name.  A manual sidecar remains the deterministic fallback.
            if best and best_score >= 0.72:
                entries = cls._parse_lrc(best)
                if entries:
                    try:
                        sidecar.write_text(best.strip() + "\n", encoding="utf-8")
                    except OSError:
                        pass
                    return entries, "online-cache"
        except Exception:
            pass
        return [], "none"

    def _cancel_lyrics(self):
        self._lyrics_generation += 1
        future = self._lyrics_future
        self._lyrics_future = None
        if future and not future.done():
            future.cancel()

    def _cancel_playback_finish(self):
        self._playback_generation += 1
        task = self._playback_finish_task
        self._playback_finish_task = None
        if task and not task.done():
            task.cancel()

    def _start_lyrics(self, conn, entries, title, source):
        self._cancel_lyrics()
        self.music_lyrics = entries
        self.music_lyrics_title = title
        self.music_lyrics_source = source
        generation = self._lyrics_generation

        async def runner():
            try:
                if not entries:
                    await send_display_message(conn, f"♪ {title}  暂无同步歌词")
                    return
                await send_display_message(conn, f"♪ {title}")
                await asyncio.sleep(max(0.0, LYRICS_START_OFFSET))
                started = time.monotonic()
                for timestamp, lyric in entries:
                    if generation != self._lyrics_generation or conn.stop_event.is_set():
                        return
                    delay = timestamp - (time.monotonic() - started)
                    if delay > 0:
                        await asyncio.sleep(delay)
                    if generation != self._lyrics_generation:
                        return
                    await send_display_message(conn, lyric[:80])
            except (asyncio.CancelledError, ConnectionError):
                return
            except Exception as exc:
                self.logger.bind(tag=TAG).warning(f"歌词同步停止: {exc}")

        self._lyrics_future = asyncio.run_coroutine_threadsafe(runner(), conn.loop)

    async def _stop_audio(self, conn):
        self.music_auto_advance = False
        self._cancel_lyrics()
        self._cancel_playback_finish()
        from core.handle.abortHandle import handleAbortMessage
        await handleAbortMessage(conn)
        return "已停止播放"

    async def _play_cached_file(self, conn, path, title):
        # A late completion callback from the previous song must never send a
        # LAST packet into the newly selected song's audio/lyric session.
        self._cancel_playback_finish()
        playback_generation = self._playback_generation
        conn.client_abort = False
        sentence_id = conn.sentence_id
        frames = []
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            lambda: conn.tts.audio_to_opus_data_stream(
                str(path), callback=frames.append
            ),
        )
        if not frames:
            raise RuntimeError("歌曲转码后没有可播放的音频帧")

        # External MCP playback bypasses send_stt_message(), which normally
        # moves the firmware from listening into speaking mode.  Without this
        # explicit start the device buffers/discards audible output until the
        # microphone detects speech, while lyrics continue advancing.
        await send_tts_message(conn, "start")
        conn.client_is_speaking = True

        # Send the first packets before returning.  _play_queue_item starts the
        # lyric clock only after this method returns, keeping lyrics aligned
        # with audible playback instead of with the earlier download request.
        await sendAudioMessage(
            conn, SentenceType.FIRST, frames, f"♪ {title}", sentence_id
        )

        async def finish_after_playback():
            current_task = asyncio.current_task()
            try:
                await _wait_for_audio_completion(conn)
                if (
                    playback_generation == self._playback_generation
                    and not conn.client_abort
                    and conn.sentence_id == sentence_id
                ):
                    await sendAudioMessage(
                        conn, SentenceType.LAST, [], None, sentence_id
                    )
                    finished_index = self.music_index
                    if self._playback_finish_task is current_task:
                        self._playback_finish_task = None
                    if self.music_auto_advance and len(self.music_queue) > 1:
                        async with self._music_switch_lock:
                            if (
                                playback_generation != self._playback_generation
                                or conn.client_abort
                                or conn.sentence_id != sentence_id
                                or not self.music_auto_advance
                                or self.music_index != finished_index
                            ):
                                return
                            self._cancel_lyrics()
                            try:
                                title = await self._play_next_playlist_item(
                                    conn, finished_index
                                )
                                self.logger.bind(tag=TAG).info(
                                    f"歌单自动续播：《{title}》"
                                )
                            except Exception as exc:
                                self.music_auto_advance = False
                                self.logger.bind(tag=TAG).warning(
                                    f"歌单自动续播停止: {exc}"
                                )
            except (asyncio.CancelledError, ConnectionError):
                return
            finally:
                if self._playback_finish_task is current_task:
                    self._playback_finish_task = None

        self._playback_finish_task = asyncio.create_task(finish_after_playback())

    @staticmethod
    def _walk_values(value):
        yield value
        if isinstance(value, dict):
            for child in value.values():
                yield from SimpleHttpServer._walk_values(child)
        elif isinstance(value, (list, tuple)):
            for child in value:
                yield from SimpleHttpServer._walk_values(child)
        elif isinstance(value, str):
            stripped = value.strip()
            if stripped.startswith(("{", "[")):
                try:
                    decoded = json.loads(stripped)
                except (TypeError, ValueError, json.JSONDecodeError):
                    return
                yield from SimpleHttpServer._walk_values(decoded)

    @classmethod
    def _camera_content(cls, result):
        payload = cls._camera_payload(result)
        content = []
        if payload and payload.get("response"):
            content.append({"type": "text", "text": str(payload["response"])})
        else:
            content.append({"type": "text", "text": str(result)})
        if not payload:
            return content

        inline_image = str(payload.get("camera_image_base64", ""))
        if inline_image:
            try:
                decoded = base64.b64decode(inline_image, validate=True)
            except (ValueError, binascii.Error):
                decoded = b""
            if decoded and len(decoded) <= 2 * 1024 * 1024:
                content.append({
                    "type": "image",
                    "data": inline_image,
                    "mimeType": str(payload.get("camera_mime_type") or "image/jpeg"),
                })
            return content

        image_id = str(payload.get("camera_image_id", ""))
        if not re.fullmatch(r"[0-9a-f]{32}", image_id):
            return content
        image_path = CAMERA_DIR / f"{image_id}.jpg"
        try:
            image_data = image_path.read_bytes()
            if image_data:
                content.append({
                    "type": "image",
                    "data": base64.b64encode(image_data).decode("ascii"),
                    "mimeType": "image/jpeg",
                })
        except OSError:
            pass
        return content

    @classmethod
    def _camera_payload(cls, result):
        for value in cls._walk_values(result):
            if isinstance(value, dict) and (
                value.get("camera_image_id") or value.get("camera_image_base64")
            ):
                return value
        return None

    @staticmethod
    def _face_name(value):
        name = str(value or "").strip()
        if not name or len(name) > 32 or any(ch in name for ch in "\\/:*?\"<>|"):
            raise RuntimeError("请输入 1 到 32 个字符的人名")
        return name

    @staticmethod
    def _detect_face(image_path):
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("VPS 尚未安装人脸识别组件") from exc
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError("摄像头照片读取失败")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detector = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        faces = detector.detectMultiScale(
            gray, scaleFactor=1.12, minNeighbors=5, minSize=(45, 45)
        )
        if len(faces) == 0:
            raise RuntimeError("照片中没有检测到清晰的正面人脸，请靠近一点并正对摄像头")
        x, y, w, h = max(faces, key=lambda item: item[2] * item[3])
        margin = max(4, int(min(w, h) * 0.08))
        x0, y0 = max(0, x - margin), max(0, y - margin)
        x1, y1 = min(gray.shape[1], x + w + margin), min(gray.shape[0], y + h + margin)
        face = cv2.resize(gray[y0:y1, x0:x1], (160, 160))
        return cv2.equalizeHist(face)

    async def _capture_face(self, conn):
        result = await call_mcp_tool(
            conn, conn.mcp_client, "self_camera_take_photo",
            {"question": "__stackchan_face_capture__"},
        )
        payload = self._camera_payload(result)
        image_id = str((payload or {}).get("camera_image_id", ""))
        if not re.fullmatch(r"[0-9a-f]{32}", image_id):
            raise RuntimeError("设备没有返回可用于人脸识别的照片")
        image_path = CAMERA_DIR / f"{image_id}.jpg"
        if not image_path.is_file():
            raise RuntimeError("VPS 上没有找到刚拍摄的照片")
        try:
            return self._detect_face(image_path)
        finally:
            image_path.unlink(missing_ok=True)

    @staticmethod
    def _face_samples():
        samples = []
        if not FACE_DIR.is_dir():
            return samples
        for person_dir in sorted(FACE_DIR.iterdir()):
            if not person_dir.is_dir():
                continue
            for image_path in sorted(person_dir.glob("*.png")):
                samples.append((person_dir.name, image_path))
        return samples

    async def _face_manage(self, conn, arguments):
        action = str(arguments.get("action", "")).strip().lower()
        if action in {"follow_on", "follow_off", "follow_status"}:
            device_action = {
                "follow_on": "on", "follow_off": "off", "follow_status": "status"
            }[action]
            return await call_mcp_tool(
                conn, conn.mcp_client, "self_face_follow", {"action": device_action}
            )
        if action == "list":
            counts = {}
            for person, _ in self._face_samples():
                counts[person] = counts.get(person, 0) + 1
            if not counts:
                return "还没有录入任何人脸"
            return "已录入：" + "、".join(f"{name}（{count}张）" for name, count in counts.items())
        if action == "latest_photo":
            try:
                image_id = CAMERA_LATEST_FILE.read_text(encoding="ascii").strip()
            except OSError:
                image_id = ""
            if not re.fullmatch(r"[0-9a-f]{32}", image_id):
                raise RuntimeError("设备最近没有可取回的照片")
            image_path = CAMERA_DIR / f"{image_id}.jpg"
            if not image_path.is_file():
                raise RuntimeError("设备最近没有可取回的照片")
            return {
                "success": True,
                "response": "这是 StackChan 最近拍摄的照片",
                "camera_image_id": image_id,
                "camera_mime_type": "image/jpeg",
            }
        if action == "delete":
            name = self._face_name(arguments.get("name"))
            person_dir = FACE_DIR / name
            removed = 0
            if person_dir.is_dir():
                for image_path in person_dir.glob("*.png"):
                    image_path.unlink()
                    removed += 1
                try:
                    person_dir.rmdir()
                except OSError:
                    pass
            return f"已删除{name}的{removed}张人脸样本" if removed else f"没有找到{name}的人脸样本"
        if action not in {"enroll", "recognize"}:
            raise RuntimeError("action 不正确")

        async with self._face_lock:
            face = await self._capture_face(conn)
            if action == "enroll":
                import cv2
                name = self._face_name(arguments.get("name"))
                person_dir = FACE_DIR / name
                person_dir.mkdir(parents=True, exist_ok=True)
                sample_path = person_dir / f"{int(time.time() * 1000)}.png"
                if not cv2.imwrite(str(sample_path), face):
                    raise RuntimeError("保存人脸样本失败")
                count = len(list(person_dir.glob("*.png")))
                suffix = "；建议再从稍微不同角度录入，累计至少 3 张" if count < 3 else ""
                return f"已录入{name}的第{count}张人脸样本{suffix}"

            samples = self._face_samples()
            if not samples:
                raise RuntimeError("还没有录入人脸，请先使用 enroll")
            import cv2
            import numpy as np
            names = sorted({person for person, _ in samples})
            label_by_name = {name: index for index, name in enumerate(names)}
            train_images, labels = [], []
            for person, image_path in samples:
                image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
                if image is not None:
                    train_images.append(image)
                    labels.append(label_by_name[person])
            if not train_images:
                raise RuntimeError("已录入的人脸样本无法读取")
            recognizer = cv2.face.LBPHFaceRecognizer_create()
            recognizer.train(train_images, np.asarray(labels, dtype=np.int32))
            label, distance = recognizer.predict(face)
            if distance > 72:
                return f"检测到人脸，但未能确认身份（距离 {distance:.1f}）"
            return f"识别为{names[label]}（匹配距离 {distance:.1f}，越低越相似）"

    async def handle_operit_mcp(self, request):
        if not self._mcp_authorized(request):
            return web.Response(status=401, text="Unauthorized")
        try:
            payload = await request.json()
        except Exception:
            return self._jsonrpc_error(None, -32700, "Invalid JSON")

        request_id = payload.get("id")
        method = payload.get("method", "")
        if method.startswith("notifications/"):
            return web.Response(status=202)
        if method == "initialize":
            params = payload.get("params") or {}
            requested_version = params.get("protocolVersion", "2025-06-18")
            supported_versions = {"2024-11-05", "2025-03-26", "2025-06-18"}
            protocol_version = (
                requested_version
                if requested_version in supported_versions
                else "2025-06-18"
            )
            return self._jsonrpc_result(request_id, {
                "protocolVersion": protocol_version,
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "stackchan-operit-bridge", "version": "1.0.0"},
            })

        conn = None
        if self.websocket_server:
            conn = await self.websocket_server.get_device_connection(self.mcp_device_id or None)

        if method == "tools/list":
            # Only advertise tools that the connected device actually reported.
            # Static schemas are documentation, not proof that the current firmware
            # implements a tool.  The sole exception is music playback, which is
            # implemented by the VPS itself.
            static_by_name = {tool["name"]: tool for tool in self.STATIC_TOOLS}
            tools_by_name = {
                name: static_by_name[name]
                for name in self.SERVER_SIDE_TOOLS | self.FIRMWARE_TOOLS
                if name in static_by_name
            }
            if self._online_music_available():
                for name in {
                    "self_music_search", "self_music_play_online",
                    "self_music_playlists", "self_music_playlist_tracks",
                    "self_music_play_playlist",
                }:
                    tools_by_name[name] = static_by_name[name]
            if conn and getattr(conn, "mcp_client", None):
                for sanitized_name, tool in conn.mcp_client.tools.items():
                    if not self._tool_allowed(sanitized_name):
                        continue
                    if sanitized_name in self.HIDDEN_DEVICE_TOOLS:
                        continue
                    live_tool = {
                        "name": sanitized_name,
                        "description": tool.get("description", ""),
                        "inputSchema": tool.get("inputSchema", {"type": "object", "properties": {}}),
                    }
                    # Prefer the clearer hand-written schema when it names the same
                    # live tool; otherwise preserve the device-provided definition.
                    tools_by_name[sanitized_name] = static_by_name.get(
                        sanitized_name, live_tool
                    )
            return self._jsonrpc_result(request_id, {"tools": list(tools_by_name.values())})

        if method == "tools/call":
            params = payload.get("params") or {}
            name = params.get("name", "")
            arguments = params.get("arguments") or {}
            if not self._tool_allowed(name):
                return self._jsonrpc_error(request_id, -32602, "Tool is not allowed")
            if name == "self_music_list":
                tracks = [path.stem for path in self._music_files()]
                text = "可播放曲目：" + ("、".join(tracks) if tracks else "暂无音源")
                return self._jsonrpc_result(request_id, {
                    "content": [{"type": "text", "text": text}],
                    "isError": not bool(tracks),
                })
            if name == "self_music_now_playing":
                if not self.music_queue or self.music_index < 0:
                    text = "当前没有已选择的歌曲"
                else:
                    item = self.music_queue[self.music_index]
                    if item.get("kind") == "local":
                        title = item["title"]
                    else:
                        track = item["track"]
                        title = track["title"] + (f" - {track['artist']}" if track.get("artist") else "")
                    text = f"当前曲目：《{title}》"
                return self._jsonrpc_result(request_id, {
                    "content": [{"type": "text", "text": text}], "isError": False,
                })
            if name == "self_music_lyrics_status":
                if not self.music_lyrics_title:
                    text = "当前没有已加载的歌词"
                elif self.music_lyrics:
                    source_name = {
                        "local-lrc": "本地LRC",
                        "online-cache": "在线同步歌词缓存",
                    }.get(self.music_lyrics_source, self.music_lyrics_source)
                    text = f"《{self.music_lyrics_title}》已加载{len(self.music_lyrics)}行同步歌词，来源：{source_name}"
                else:
                    text = f"《{self.music_lyrics_title}》暂无同步歌词；可放入同名 .lrc 文件后重播"
                return self._jsonrpc_result(request_id, {
                    "content": [{"type": "text", "text": text}], "isError": False,
                })
            if name == "self_music_search":
                try:
                    tracks = await self._search_online_music(
                        arguments.get("query", ""), arguments.get("limit", 5)
                    )
                    text = "搜索结果：" + "、".join(
                        track["title"] + (f" - {track['artist']}" if track["artist"] else "")
                        for track in tracks
                    )
                    return self._jsonrpc_result(request_id, {
                        "content": [{"type": "text", "text": text}], "isError": False,
                    })
                except Exception as exc:
                    return self._jsonrpc_result(request_id, {
                        "content": [{"type": "text", "text": str(exc)}], "isError": True,
                    })
            if name == "self_music_playlists":
                try:
                    playlists = await self._ncm_playlists()
                    text = "网易云歌单：" + "、".join(
                        f"《{item['name']}》（{item['track_count']}首）" for item in playlists
                    ) + "。请询问用户想播放哪一个歌单。"
                    return self._jsonrpc_result(request_id, {
                        "content": [{"type": "text", "text": text}], "isError": False,
                    })
                except Exception as exc:
                    return self._jsonrpc_result(request_id, {
                        "content": [{"type": "text", "text": str(exc)}], "isError": True,
                    })
            if name == "self_music_playlist_tracks":
                try:
                    playlist = await self._select_ncm_playlist(arguments.get("playlist", ""))
                    limit = max(1, min(int(arguments.get("limit", 30)), 100))
                    tracks = await self._ncm_playlist_tracks(playlist["id"], limit)
                    text = f"歌单《{playlist['name']}》曲目：" + "、".join(
                        track["title"] + (f" - {track['artist']}" if track["artist"] else "")
                        for track in tracks
                    )
                    return self._jsonrpc_result(request_id, {
                        "content": [{"type": "text", "text": text}], "isError": False,
                    })
                except Exception as exc:
                    return self._jsonrpc_result(request_id, {
                        "content": [{"type": "text", "text": str(exc)}], "isError": True,
                    })
            if not conn or not getattr(conn, "mcp_client", None):
                return self._jsonrpc_error(
                    request_id,
                    -32001,
                    "StackChan 当前未连接，请先唤醒当前助手并保持会话后再调用",
                )
            try:
                if name == "self_music_play":
                    async with self._music_switch_lock:
                        await self._stop_audio(conn)
                        selected = self._choose_music(arguments.get("song_name", "random"))
                        self._set_local_music_queue(selected)
                        title = await self._play_queue_item(conn, self.music_queue[self.music_index])
                    result = f"已发送播放指令：《{title}》"
                elif name == "self_music_play_online":
                    async with self._music_switch_lock:
                        await self._stop_audio(conn)
                        tracks = await self._search_online_music(arguments.get("query", ""), 5)
                        self.music_queue = [{"kind": "online", "track": track} for track in tracks]
                        title = await self._play_available_queue_item(conn, 0)
                    result = f"已缓存并开始播放：《{title}》"
                elif name == "self_music_play_playlist":
                    async with self._music_switch_lock:
                        await self._stop_audio(conn)
                        playlist = await self._select_ncm_playlist(arguments.get("playlist", ""))
                        tracks = await self._ncm_playlist_tracks(playlist["id"], 500)
                        self.music_queue = [{"kind": "online", "track": track} for track in tracks]
                        self.music_auto_advance = True
                        title = await self._play_available_queue_item(conn, 0)
                    result = (
                        f"已自动加载歌单《{playlist['name']}》（{len(tracks)}首）"
                        f"并开始播放：《{title}》"
                    )
                elif name == "self_music_stop":
                    async with self._music_switch_lock:
                        result = await self._stop_audio(conn)
                elif name == "self_music_next":
                    async with self._music_switch_lock:
                        if not self.music_queue:
                            raise RuntimeError("当前没有播放队列，请先选择一首歌")
                        next_index = (self.music_index + 1) % len(self.music_queue)
                        auto_advance = self.music_auto_advance
                        await self._stop_audio(conn)
                        self.music_auto_advance = auto_advance
                        title = await self._play_available_queue_item(conn, next_index)
                    result = f"已切换到下一首：《{title}》"
                elif name == "self_alarm_create":
                    result = await self._create_alarm(conn, arguments)
                elif name == "self_alarm_cancel":
                    result = await call_mcp_tool(conn, conn.mcp_client, name, arguments)
                    alarm_id = int(arguments.get("id") or -1)
                    previous_count = len(self.music_alarms)
                    self.music_alarms = [
                        item for item in self.music_alarms
                        if int(item.get("id") or -1) != alarm_id
                    ]
                    if len(self.music_alarms) != previous_count:
                        self._save_music_alarms()
                else:
                    if name == "self_camera_take_photo":
                        # Keep vision in Operit's current model/context. The
                        # device waits for the physical shutter, then this
                        # reserved question makes the vision endpoint return
                        # the JPEG itself instead of invoking a separate VPS
                        # VLLM provider.
                        result = await call_mcp_tool(
                            conn,
                            conn.mcp_client,
                            name,
                            {"question": "__stackchan_mcp_inline__"},
                            timeout=55,
                        )
                    else:
                        result = await call_mcp_tool(
                            conn, conn.mcp_client, name, arguments
                        )
                content = (
                    self._camera_content(result)
                    if name == "self_camera_take_photo"
                    else [{"type": "text", "text": str(result)}]
                )
                if name == "self_camera_take_photo" and content:
                    requested_view = str(arguments.get("question", "")).strip()
                    if requested_view:
                        content[0] = {
                            "type": "text",
                            "text": f"请根据随附的 StackChan 实拍照片回答：{requested_view}",
                        }
                return self._jsonrpc_result(request_id, {
                    "content": content,
                    "isError": False,
                })
            except Exception as exc:
                return self._jsonrpc_result(request_id, {
                    "content": [{"type": "text", "text": str(exc)}],
                    "isError": True,
                })
        return self._jsonrpc_error(request_id, -32601, "Method not found")

    async def handle_operit_mcp_get(self, request):
        if not self._mcp_authorized(request):
            return web.Response(status=401, text="Unauthorized")
        return web.Response(status=405, text="Use MCP Streamable HTTP POST")

    async def handle_health(self, request):
        """Public health response used by clients that probe the origin first."""
        return web.json_response({
            "status": "ok",
            "service": "stackchan-operit-mcp",
            "endpoint": "/mcp",
        })

    async def handle_operit_mcp_options(self, request):
        return web.Response(status=204, headers={
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
            "Access-Control-Allow-Headers": (
                "Authorization, Content-Type, Accept, MCP-Protocol-Version, "
                "Mcp-Session-Id"
            ),
        })

    def _get_websocket_url(self, local_ip: str, port: int) -> str:
        """获取websocket地址

        Args:
            local_ip: 本地IP地址
            port: 端口号

        Returns:
            str: websocket地址
        """
        server_config = self.config["server"]
        websocket_config = server_config.get("websocket")

        if websocket_config and "你" not in websocket_config:
            return websocket_config
        else:
            return f"ws://{local_ip}:{port}/xiaozhi/v1/"

    async def start(self):
        try:
            server_config = self.config["server"]
            read_config_from_api = self.config.get("read_config_from_api", False)
            host = server_config.get("ip", "0.0.0.0")
            port = int(server_config.get("http_port", 8003))

            if port:
                app = web.Application()

                if not read_config_from_api:
                    # 如果没有开启智控台，只是单模块运行，就需要再添加简单OTA接口，用于下发websocket接口
                    app.add_routes(
                        [
                            web.get("/xiaozhi/ota/", self.ota_handler.handle_get),
                            web.post("/xiaozhi/ota/", self.ota_handler.handle_post),
                            web.options(
                                "/xiaozhi/ota/", self.ota_handler.handle_options
                            ),
                            # 下载接口，仅提供 data/bin/*.bin 下载
                            web.get(
                                "/xiaozhi/ota/download/{filename}",
                                self.ota_handler.handle_download,
                            ),
                            web.options(
                                "/xiaozhi/ota/download/{filename}",
                                self.ota_handler.handle_options,
                            ),
                        ]
                    )
                # 添加路由
                app.add_routes(
                    [
                        web.get("/", self.handle_health),
                        web.get("/operit/mcp", self.handle_operit_mcp_get),
                        web.post("/operit/mcp", self.handle_operit_mcp),
                        web.options("/operit/mcp", self.handle_operit_mcp_options),
                        # Standard MCP path. Some mobile MCP runtimes discard a
                        # custom path after probing the host origin.
                        web.get("/mcp", self.handle_operit_mcp_get),
                        web.post("/mcp", self.handle_operit_mcp),
                        web.options("/mcp", self.handle_operit_mcp_options),
                        web.get("/mcp/vision/explain", self.vision_handler.handle_get),
                        web.post(
                            "/mcp/vision/explain", self.vision_handler.handle_post
                        ),
                        web.options(
                            "/mcp/vision/explain", self.vision_handler.handle_options
                        ),
                    ]
                )

                # 运行服务
                runner = web.AppRunner(app)
                await runner.setup()
                site = web.TCPSite(runner, host, port)
                await site.start()

                # 保持服务运行
                self._music_alarm_task = asyncio.create_task(self._music_alarm_loop())
                try:
                    while True:
                        await asyncio.sleep(3600)  # 每隔 1 小时检查一次
                finally:
                    if self._music_alarm_task and not self._music_alarm_task.done():
                        self._music_alarm_task.cancel()
        except Exception as e:
            self.logger.bind(tag=TAG).error(f"HTTP服务器启动失败: {e}")
            import traceback

            self.logger.bind(tag=TAG).error(f"错误堆栈: {traceback.format_exc()}")
            raise
