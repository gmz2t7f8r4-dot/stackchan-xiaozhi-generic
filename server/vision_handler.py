import json
import copy
import os
import time
import uuid
from pathlib import Path
from aiohttp import web
from config.logger import setup_logging
from core.api.base_handler import BaseHandler
from core.utils.util import get_vision_url, is_valid_image_file
from core.utils.vllm import create_instance
from config.config_loader import get_private_config_from_api
from core.utils.auth import AuthToken
import base64
from typing import Tuple, Optional
from plugins_func.register import Action

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

TAG = __name__

# 设置最大文件大小为5MB
MAX_FILE_SIZE = 5 * 1024 * 1024
CAMERA_MCP_DIR = Path(
    os.environ.get(
        "STACKCHAN_CAMERA_DIR",
        "/opt/xiaozhi-esp32-server/data/camera_mcp",
    )
)
CAMERA_TTL_SECONDS = 10 * 60
CAMERA_MAX_FILES = 20
CAMERA_LATEST_FILE = CAMERA_MCP_DIR / "latest_manual.txt"
FACE_DIR = Path(
    os.environ.get(
        "STACKCHAN_FACE_DIR",
        "/opt/xiaozhi-esp32-server/data/stackchan_faces",
    )
)
YUNET_MODEL_PATH = Path(
    "/opt/xiaozhi-esp32-server/data/models/face_detection_yunet_2023mar.onnx"
)
_HAAR_DETECTOR = None
_YUNET_DETECTOR = None
if cv2 is not None:
    _HAAR_DETECTOR = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )
    if YUNET_MODEL_PATH.is_file() and hasattr(cv2, "FaceDetectorYN"):
        # Preload the DNN while the service starts.  Lazy loading on the first
        # device request can exceed the CoreS3's 30-second HTTP timeout.
        _YUNET_DETECTOR = cv2.FaceDetectorYN.create(
            str(YUNET_MODEL_PATH), "", (320, 240), 0.35, 0.3, 5000
        )
LOCAL_CAMERA_DEVICE_ID = os.environ.get(
    "STACKCHAN_LOCAL_CAMERA_DEVICE_ID", "7c:4f:ad:af:91:50"
)
LOCAL_CAMERA_CLIENT_ID = os.environ.get(
    "STACKCHAN_LOCAL_CAMERA_CLIENT_ID", "1b93975e-89d1-4622-9f9f-2c27428def06"
)


class VisionHandler(BaseHandler):
    def __init__(self, config: dict):
        super().__init__(config)
        # 初始化认证工具
        self.auth = AuthToken(config["server"]["auth_key"])

    def _create_error_response(self, message: str) -> dict:
        """创建统一的错误响应格式"""
        return {"success": False, "message": message}

    def _create_raw_camera_response(self, message: str, success: bool = True):
        """Return a bodyless response that the low-memory CoreS3 can finish immediately."""
        encoded = base64.b64encode(message.encode("utf-8")).decode("ascii")
        response = web.Response(
            status=200,
            headers={
                "X-StackChan-Result": encoded,
                "X-StackChan-Success": "1" if success else "0",
            },
        )
        self._add_cors_headers(response)
        return response

    @staticmethod
    def _save_camera_image(image_data: bytes, mark_latest: bool = True) -> str:
        """Persist an MCP image and optionally make it the latest manual photo."""
        CAMERA_MCP_DIR.mkdir(parents=True, exist_ok=True)
        try:
            latest_id = CAMERA_LATEST_FILE.read_text(encoding="ascii").strip()
        except OSError:
            latest_id = ""
        now = time.time()
        files = []
        for path in CAMERA_MCP_DIR.glob("*.jpg"):
            try:
                stat = path.stat()
                if path.stem != latest_id and now - stat.st_mtime > CAMERA_TTL_SECONDS:
                    path.unlink(missing_ok=True)
                else:
                    files.append((stat.st_mtime, path))
            except OSError:
                continue
        removable = [item for item in sorted(files) if item[1].stem != latest_id]
        for _, old_path in removable[:-CAMERA_MAX_FILES + 1]:
            try:
                old_path.unlink(missing_ok=True)
            except OSError:
                pass

        image_id = uuid.uuid4().hex
        (CAMERA_MCP_DIR / f"{image_id}.jpg").write_bytes(image_data)
        if mark_latest:
            CAMERA_LATEST_FILE.write_text(image_id, encoding="ascii")
        return image_id

    async def _analyze_camera_image(
        self,
        device_id: str,
        client_id: str,
        question: str,
        image_data: bytes,
    ) -> str:
        """Run the configured vision model for both MCP and local camera flows."""
        image_base64 = base64.b64encode(image_data).decode("utf-8")
        current_config = copy.deepcopy(self.config)
        if current_config.get("read_config_from_api", False):
            current_config = await get_private_config_from_api(
                current_config,
                device_id,
                client_id,
            )

        select_vllm_module = current_config["selected_module"].get("VLLM")
        if not select_vllm_module:
            raise ValueError("您还未设置默认的视觉分析模块")
        vllm_type = (
            select_vllm_module
            if "type" not in current_config["VLLM"][select_vllm_module]
            else current_config["VLLM"][select_vllm_module]["type"]
        )
        if not vllm_type:
            raise ValueError(f"无法找到VLLM模块对应的供应器{vllm_type}")
        vllm = create_instance(
            vllm_type,
            current_config["VLLM"][select_vllm_module],
        )
        return vllm.response(question, image_base64)

    def _verify_auth_token(self, request) -> Tuple[bool, Optional[str], bool]:
        """验证认证token"""
        # 测试模式：允许特定测试令牌或跳过验证
        auth_header = request.headers.get("Authorization", "")
        client_id = request.headers.get("Client-Id", "")

        # 允许测试客户端跳过认证
        if client_id == "web_test_client":
            device_id = request.headers.get("Device-Id", "test_device")
            return True, device_id, False

        # The local full-screen camera app must work without first opening a
        # conversational MCP session.  Permit this one physical device to use
        # only the reserved local-camera commands; handle_post enforces that
        # command allow-list after reading the multipart question field.
        device_id = request.headers.get("Device-Id", "")
        if (
            device_id == LOCAL_CAMERA_DEVICE_ID
            and client_id == LOCAL_CAMERA_CLIENT_ID
            and not auth_header
        ):
            return True, device_id, True

        if not auth_header.startswith("Bearer "):
            return False, None, False

        token = auth_header[7:]  # 移除"Bearer "前缀
        valid, token_device_id = self.auth.verify_token(token)
        return valid, token_device_id, False

    @staticmethod
    def _detect_face_bytes(image_data: bytes):
        if cv2 is None or np is None:
            raise ValueError("VPS 尚未安装人脸识别组件")
        image = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("摄像头照片读取失败")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Haar detection is unnecessarily expensive on a frame that contains
        # no usable light at all.  More importantly, report the physical cause
        # to the device instead of making it look like an upload failure.
        if float(np.mean(gray)) < 18.0 or float(np.percentile(gray, 90)) < 30.0:
            raise ValueError("画面过暗，请检查屏幕下方的摄像头是否被遮挡")
        # GC0308 is only QVGA/0.3 MP.  Equalization plus a slightly smaller
        # minimum window is materially more reliable than desktop-camera Haar
        # defaults while still requiring a compact multi-neighbour detection.
        detection_gray = cv2.resize(
            cv2.equalizeHist(gray), None, fx=2.0, fy=2.0,
            interpolation=cv2.INTER_CUBIC,
        )
        haar_faces = _HAAR_DETECTOR.detectMultiScale(
            detection_gray, scaleFactor=1.20, minNeighbors=3, minSize=(60, 60)
        )
        face_boxes = [
            (int(x // 2), int(y // 2), int(w // 2), int(h // 2))
            for x, y, w, h in haar_faces
        ]
        if not face_boxes and _YUNET_DETECTOR is not None:
            # YuNet is a compact official OpenCV model and is much more robust
            # than Haar on the CoreS3's underexposed 0.3 MP frames.
            enhanced = cv2.cvtColor(cv2.equalizeHist(gray), cv2.COLOR_GRAY2BGR)
            image_h, image_w = enhanced.shape[:2]
            _YUNET_DETECTOR.setInputSize((image_w, image_h))
            _, yunet_faces = _YUNET_DETECTOR.detect(enhanced)
            if yunet_faces is not None:
                face_boxes = [
                    (int(face[0]), int(face[1]), int(face[2]), int(face[3]))
                    for face in yunet_faces
                ]
        if not face_boxes:
            raise ValueError("照片中没有检测到清晰的正面人脸")
        x, y, w, h = max(face_boxes, key=lambda item: item[2] * item[3])
        margin = max(4, int(min(w, h) * 0.08))
        x0, y0 = max(0, x - margin), max(0, y - margin)
        x1, y1 = min(gray.shape[1], x + w + margin), min(gray.shape[0], y + h + margin)
        return cv2.equalizeHist(cv2.resize(gray[y0:y1, x0:x1], (160, 160)))

    @staticmethod
    def _face_samples():
        samples = []
        if not FACE_DIR.is_dir():
            return samples
        for person_dir in sorted(FACE_DIR.iterdir()):
            if person_dir.is_dir():
                for image_path in sorted(person_dir.glob("*.png")):
                    samples.append((person_dir.name, image_path))
        return samples

    @classmethod
    def _handle_local_face_action(cls, question: str, image_data: bytes):
        enroll_prefix = "__stackchan_face_enroll__:"
        if question == "__stackchan_face_recognize__":
            action, name = "recognize", ""
        elif question.startswith(enroll_prefix):
            action, name = "enroll", question[len(enroll_prefix):].strip()
            if not name or len(name) > 32 or any(ch in name for ch in '\\/:*?"<>|'):
                raise ValueError("录入的人名无效")
        else:
            return None

        face = cls._detect_face_bytes(image_data)
        import cv2
        if action == "enroll":
            person_dir = FACE_DIR / name
            person_dir.mkdir(parents=True, exist_ok=True)
            sample_path = person_dir / f"{int(time.time() * 1000)}.png"
            if not cv2.imwrite(str(sample_path), face):
                raise ValueError("保存人脸样本失败")
            count = len(list(person_dir.glob("*.png")))
            return f"已录入{name}的第{count}张人脸"

        samples = cls._face_samples()
        if not samples:
            raise ValueError("还没有录入人脸，请先录入用户")
        import numpy as np
        names = sorted({person for person, _ in samples})
        label_by_name = {person: index for index, person in enumerate(names)}
        train_images, labels = [], []
        for person, image_path in samples:
            sample = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if sample is not None:
                train_images.append(sample)
                labels.append(label_by_name[person])
        if not train_images:
            raise ValueError("已录入的人脸样本无法读取")
        recognizer = cv2.face.LBPHFaceRecognizer_create()
        recognizer.train(train_images, np.asarray(labels, dtype=np.int32))
        label, distance = recognizer.predict(face)
        if distance > 100:
            return f"检测到人脸，但未能确认身份（距离 {distance:.1f}）"
        if distance > 72:
            return f"可能是{names[label]}（光线较暗，匹配距离 {distance:.1f}）"
        return f"识别为{names[label]}（匹配距离 {distance:.1f}）"

    async def handle_post(self, request):
        """处理 MCP Vision POST 请求"""
        response = None  # 初始化response变量
        try:
            # 验证token
            is_valid, token_device_id, local_camera_auth = self._verify_auth_token(request)
            if not is_valid:
                response = web.Response(
                    text=json.dumps(
                        self._create_error_response("无效的认证token或token已过期")
                    ),
                    content_type="application/json",
                    status=401,
                )
                return response

            # 获取请求头信息
            device_id = request.headers.get("Device-Id", "")
            client_id = request.headers.get("Client-Id", "")
            if device_id != token_device_id:
                raise ValueError("设备ID与token不匹配")

            # The full-screen camera app uses a deliberately small raw-JPEG
            # protocol.  It avoids multipart/chunked edge cases on the CoreS3
            # when internal RAM is almost exhausted.  This branch is reachable
            # only through the physical-device allow-list above.
            raw_camera_action = request.headers.get("X-StackChan-Camera-Action", "")
            if raw_camera_action:
                self.logger.bind(tag=TAG).info(
                    f"本机相机请求开始: action={raw_camera_action} "
                    f"content_length={request.content_length} device={device_id}"
                )
                if not local_camera_auth or raw_camera_action not in {
                    "look", "photo", "recognize", "enroll_qiqi"
                }:
                    response = web.Response(
                        text=json.dumps(
                            self._create_error_response("无权使用本机相机接口"),
                            ensure_ascii=False,
                        ),
                        content_type="application/json",
                        status=403,
                    )
                    return response
                try:
                    image_data = await request.read()
                    self.logger.bind(tag=TAG).info(
                        f"本机相机图片接收完成: action={raw_camera_action} "
                        f"bytes={len(image_data)}"
                    )
                    if not image_data:
                        raise ValueError("图片数据为空")
                    if len(image_data) > MAX_FILE_SIZE:
                        raise ValueError("图片大小超过限制")
                    if not is_valid_image_file(image_data):
                        raise ValueError("摄像头上传的照片格式无效")

                    if raw_camera_action == "look":
                        self._save_camera_image(image_data, mark_latest=True)
                        # The VPS currently has no valid VLLM credential.
                        # Keep local capture reliable and expose the saved JPEG
                        # to MCP/Operit instead of invoking a placeholder key.
                        message = "照片已拍好"
                    elif raw_camera_action == "photo":
                        self._save_camera_image(image_data, mark_latest=True)
                        message = "拍照完成，已保存为最近照片"
                    else:
                        question = (
                            "__stackchan_face_recognize__"
                            if raw_camera_action == "recognize"
                            else "__stackchan_face_enroll__:用户"
                        )
                        message = self._handle_local_face_action(question, image_data)
                    response = self._create_raw_camera_response(message, True)
                    return response
                except ValueError as exc:
                    self.logger.bind(tag=TAG).error(f"本机相机请求异常: {exc}")
                    response = self._create_raw_camera_response(str(exc), False)
                    return response

            # 解析multipart/form-data请求
            reader = await request.multipart()

            # 读取question字段
            question_field = await reader.next()
            if question_field is None:
                raise ValueError("缺少问题字段")
            question = await question_field.text()
            self.logger.bind(tag=TAG).debug(f"Question: {question}")

            if local_camera_auth and not (
                question in {
                    "__stackchan_manual_photo__",
                    "__stackchan_face_recognize__",
                }
                or question.startswith("__stackchan_face_enroll__:")
            ):
                response = web.Response(
                    text=json.dumps(
                        self._create_error_response("本机相机仅允许拍照、录入和识别人脸"),
                        ensure_ascii=False,
                    ),
                    content_type="application/json",
                    status=403,
                )
                return response

            # 读取图片文件
            image_field = await reader.next()
            if image_field is None:
                raise ValueError("缺少图片文件")

            # 读取图片数据
            image_data = await image_field.read()
            if not image_data:
                raise ValueError("图片数据为空")

            # 检查文件大小
            if len(image_data) > MAX_FILE_SIZE:
                raise ValueError(
                    f"图片大小超过限制，最大允许{MAX_FILE_SIZE/1024/1024}MB"
                )

            # 检查文件格式
            if not is_valid_image_file(image_data):
                raise ValueError(
                    "不支持的文件格式，请上传有效的图片文件（支持JPEG、PNG、GIF、BMP、TIFF、WEBP格式）"
                )

            # The on-device CAMERA app uses authenticated reserved questions
            # so it can enroll/recognize locally without saving the source frame.
            local_face_response = self._handle_local_face_action(question, image_data)
            if local_face_response is not None:
                return_json = {
                    "success": True,
                    "action": Action.RESPONSE.name,
                    "response": local_face_response,
                }
                response = web.Response(
                    text=json.dumps(return_json, ensure_ascii=False, separators=(",", ":")),
                    content_type="application/json",
                )
                self._add_cors_headers(response)
                return response

            # Operit-initiated face management needs the source frame only long
            # enough to detect/crop it. The bridge deletes this transient file in
            # a finally block, and it never replaces the latest manual photo.
            if question == "__stackchan_face_capture__":
                camera_image_id = self._save_camera_image(image_data, mark_latest=False)
                return_json = {
                    "success": True,
                    "action": Action.RESPONSE.name,
                    "response": "人脸临时照片已拍摄",
                    "camera_image_id": camera_image_id,
                    "camera_mime_type": "image/jpeg",
                    "camera_transient": True,
                }
                response = web.Response(
                    text=json.dumps(return_json, ensure_ascii=False, separators=(",", ":")),
                    content_type="application/json",
                )
                self._add_cors_headers(response)
                return response

            # A screen-button photo or a direct camera MCP call is a manual photo.
            # Keep it as Operit's latest retrievable photo (without a public URL).
            camera_image_id = self._save_camera_image(image_data, mark_latest=True)
            if question in {
                "__stackchan_manual_photo__",
                "__stackchan_mcp_photo__",
            }:
                return_json = {
                    "success": True,
                    "action": Action.RESPONSE.name,
                    "response": (
                        "照片已发送给当前助手"
                        if question == "__stackchan_mcp_photo__"
                        else "拍照完成，已保存为最近照片"
                    ),
                    "camera_image_id": camera_image_id,
                    "camera_mime_type": "image/jpeg",
                }
                response = web.Response(
                    text=json.dumps(return_json, ensure_ascii=False, separators=(",", ":")),
                    content_type="application/json",
                )
                self._add_cors_headers(response)
                return response

            result = await self._analyze_camera_image(
                device_id,
                client_id,
                question,
                image_data,
            )

            return_json = {
                "success": True,
                "action": Action.RESPONSE.name,
                "response": result,
                "camera_image_id": camera_image_id,
                "camera_mime_type": "image/jpeg",
            }

            response = web.Response(
                text=json.dumps(return_json, separators=(",", ":")),
                content_type="application/json",
            )
        except ValueError as e:
            self.logger.bind(tag=TAG).error(f"MCP Vision POST请求异常: {e}")
            return_json = self._create_error_response(str(e))
            response = web.Response(
                text=json.dumps(return_json, separators=(",", ":")),
                content_type="application/json",
            )
        except Exception as e:
            self.logger.bind(tag=TAG).error(f"MCP Vision POST请求异常: {e}")
            return_json = self._create_error_response("处理请求时发生错误")
            response = web.Response(
                text=json.dumps(return_json, separators=(",", ":")),
                content_type="application/json",
            )
        finally:
            if response:
                self._add_cors_headers(response)
            return response

    async def handle_get(self, request):
        """处理 MCP Vision GET 请求"""
        try:
            vision_explain = get_vision_url(self.config)
            if vision_explain and len(vision_explain) > 0 and "null" != vision_explain:
                message = (
                    f"MCP Vision 接口运行正常，视觉解释接口地址是：{vision_explain}"
                )
            else:
                message = "MCP Vision 接口运行不正常，请打开data目录下的.config.yaml文件，找到【server.vision_explain】，设置好地址"

            response = web.Response(text=message, content_type="text/plain")
        except Exception as e:
            self.logger.bind(tag=TAG).error(f"MCP Vision GET请求异常: {e}")
            return_json = self._create_error_response("服务器内部错误")
            response = web.Response(
                text=json.dumps(return_json, separators=(",", ":")),
                content_type="application/json",
            )
        finally:
            self._add_cors_headers(response)
            return response
