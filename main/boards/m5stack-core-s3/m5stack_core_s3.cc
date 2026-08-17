#include "wifi_board.h"
#include "cores3_audio_codec.h"
#include "display/lcd_display.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "application.h"
#include "config.h"
#include "power_save_timer.h"
#include "i2c_device.h"
#include "axp2101.h"
#include "mcp_server.h"
#include "settings.h"
#include "wifi_manager.h"
#include "assets/lang_config.h"
#include "stackchan_ble_compat.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9341.h>
#include <esp_timer.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <mutex>
#include <sstream>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "esp_sntp.h"
#include "esp_video.h"
#include "lvgl.h"
#include "SCSCL.h"
#include "i2c_bus.h"
#include "bmi270_api.h"
#include "bmi2.h"

// BMI270 SDK 在 .a 里有这些 public 符号但头文件未暴露——自己声明用来绕过 bmi270_sensor_create 硬编码 0x68 的限制
extern "C" {
    int8_t bmi270_init(struct bmi2_dev *dev);
    extern const uint8_t bmi270_config_file[];
}

// BMI270 自定义 I2C read/write（地址 0x69）
static int8_t Bmi270I2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    auto dev = (i2c_master_dev_handle_t)intf_ptr;
    return i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, 200) == ESP_OK ? 0 : -1;
}

static int8_t Bmi270I2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    static uint8_t big_buf[8300];  // BMI270 config blob ~8KB
    if (len + 1 > sizeof(big_buf)) return -1;
    auto dev = (i2c_master_dev_handle_t)intf_ptr;
    big_buf[0] = reg_addr;
    memcpy(big_buf + 1, data, len);
    return i2c_master_transmit(dev, big_buf, len + 1, 500) == ESP_OK ? 0 : -1;
}

static void Bmi270DelayUs(uint32_t period_us, void *intf_ptr) {
    if (period_us < 1000) {
        esp_rom_delay_us(period_us);
    } else {
        vTaskDelay(pdMS_TO_TICKS((period_us + 999) / 1000));
    }
}

#define TAG "M5StackCoreS3Board"

class FaceTracker;

class StackChanServo {
public:
    bool Begin() {
        if (!bus_.begin(UART_NUM_1, 1000000, 6, 7)) {
            ESP_LOGE("Servo", "SCS bus begin failed");
            return false;
        }
        ESP_LOGI("Servo", "SCS bus init OK on UART1 GPIO6/7 @1Mbps");
        anim_queue_ = xQueueCreate(8, sizeof(ServoAnimation));
        if (!anim_queue_) {
            ESP_LOGE("Servo", "Failed to create animation queue");
            return false;
        }
        const BaseType_t task_result = xTaskCreatePinnedToCore(
            AnimationTaskFunc, "servo_anim", 4096, this, 2, &anim_task_, 1);
        if (task_result != pdPASS) {
            ESP_LOGE("Servo", "Failed to create animation task: %ld", (long)task_result);
            vQueueDelete(anim_queue_);
            anim_queue_ = nullptr;
            return false;
        }
        ESP_LOGI("Servo", "Animation worker ready (queue=8, stack=4096)");
        MoveTo(0, 30, 1500);
        // Keep the head stationary by default.  Motion is only performed by
        // explicit MCP commands (nod/shake/tilt/center/look_at).
        scan_running_ = false;
        ESP_LOGI("Servo", "Automatic idle scan disabled");
        return true;
    }

    void MoveTo(int yaw_deg, int pitch_deg, int time_ms) {
        if (yaw_deg < -45) yaw_deg = -45;
        if (yaw_deg > 45) yaw_deg = 45;
        if (pitch_deg < 5) pitch_deg = 5;
        if (pitch_deg > 60) pitch_deg = 60;
        int yaw_pos = 460 + yaw_deg * 16 / 5;
        int pitch_pos = 620 + pitch_deg * 16 / 5;
        bus_.WritePos(1, yaw_pos, time_ms, 0);
        bus_.WritePos(2, pitch_pos, time_ms, 0);
    }

    void PauseScan() {
        if (scan_running_ && idle_timer_) {
            esp_timer_stop(idle_timer_);
            scan_running_ = false;
        }
    }

    void ResumeScan() {
        // Intentionally disabled: waking from power save or pausing face
        // tracking must not restart random head movement.
    }

    void Center() { MoveTo(0, 30, 600); }

    void SetFaceTracker(FaceTracker* ft) { tracker_ = ft; }

    void Nod();
    void Shake();
    void Tilt();
    void Pet();
    bool StartDance(const std::string& sequence, int tempo_ms, int repeat);
    void StopDance();
    bool IsDancing() const { return dance_running_.load(); }

    bool IsAnimating() const { return anim_running_.load(); }

private:
    enum class ServoAnimation : uint8_t { Nod, Shake, Tilt, Pet };
    bool EnqueueAnimation(ServoAnimation animation);
    static void AnimationTaskFunc(void* arg);
    void AnimationLoop();
    void RunAnimation(ServoAnimation animation);
    struct DanceStep { int8_t yaw; int8_t pitch; };
    struct DanceContext {
        StackChanServo* servo;
        DanceStep steps[24];
        uint8_t count;
        uint8_t repeat;
        uint16_t tempo_ms;
    };
    static void DanceTaskFunc(void* arg);
    void DanceLoop(DanceContext* context);
    SCSCL bus_;
    esp_timer_handle_t idle_timer_ = nullptr;
    FaceTracker* tracker_ = nullptr;
    bool scan_running_ = false;
    std::atomic<bool> anim_running_{false};
    QueueHandle_t anim_queue_ = nullptr;
    TaskHandle_t anim_task_ = nullptr;
    std::atomic<bool> dance_running_{false};
    std::atomic<bool> dance_stop_{false};
    TaskHandle_t dance_task_ = nullptr;
};

class FaceTracker {
    static constexpr int DS_W = 40;
    static constexpr int DS_H = 30;
public:
    void Start(EspVideo* camera, StackChanServo* servo) {
        camera_ = camera;
        servo_ = servo;
        if (!camera_ || !servo_) return;

        paused_ = true;
        const BaseType_t result = xTaskCreatePinnedToCoreWithCaps(
            TaskFunc, "face_track", 8192, this, 1, &task_, 1,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (result != pdPASS) {
            task_ = nullptr;
            ESP_LOGE("FaceTrack", "Failed to create PSRAM tracking task: %ld", (long)result);
            return;
        }
        ESP_LOGI("FaceTrack", "Started (skin-region face following)");
    }

    void Pause(bool resume_scan = true) {
        if (!paused_) {
            paused_ = true;
            tracking_ = false;
            if (resume_scan) servo_->ResumeScan();
            ESP_LOGI("FaceTrack", "Paused (scan=%d)", resume_scan);
        }
    }

    void Resume() {
        if (enabled_ && paused_) {
            paused_ = false;
            servo_->PauseScan();
            ESP_LOGI("FaceTrack", "Resumed");
        }
    }

    void SetEnabled(bool enabled) {
        enabled_ = enabled;
        if (enabled) {
            Resume();
        } else {
            Pause(false);
            if (servo_) servo_->Center();
        }
    }

    bool IsPaused() const { return paused_; }
    bool IsEnabled() const { return enabled_; }
    float GetYaw() const { return yaw_; }
    float GetPitch() const { return pitch_; }

private:
    static void TaskFunc(void* arg) {
        auto* self = static_cast<FaceTracker*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (self->paused_ || !self->camera_->IsOk()) continue;
            self->Track();
        }
    }

    void Track() {
        uint8_t skin[DS_W * DS_H] = {};

        bool ok = camera_->PeekFrame([&](const uint8_t* data, size_t len, uint16_t w, uint16_t h) {
            if (w == 0 || h == 0) return;
            int sx = w / DS_W;
            int sy = h / DS_H;
            const uint32_t format = camera_->SensorFormat();
            for (int dy = 0; dy < DS_H; dy++) {
                for (int dx = 0; dx < DS_W; dx++) {
                    const int px_x = dx * sx;
                    const int px_y = dy * sy;
                    int y = 0, cb = 0, cr = 0, r = 0, g = 0, b = 0;
                    if (format == V4L2_PIX_FMT_YUV422P || format == V4L2_PIX_FMT_YUYV) {
                        size_t pair = ((size_t)px_y * w + (px_x & ~1)) * 2;
                        if (pair + 3 >= len) continue;
                        y = data[pair + ((px_x & 1) ? 2 : 0)];
                        cb = data[pair + 1];
                        cr = data[pair + 3];
                        r = y + ((359 * (cr - 128)) >> 8);
                        g = y - ((88 * (cb - 128) + 183 * (cr - 128)) >> 8);
                        b = y + ((454 * (cb - 128)) >> 8);
                    } else if (format == V4L2_PIX_FMT_RGB24) {
                        size_t offset = ((size_t)px_y * w + px_x) * 3;
                        if (offset + 2 >= len) continue;
                        r = data[offset]; g = data[offset + 1]; b = data[offset + 2];
                        y = (77 * r + 150 * g + 29 * b) >> 8;
                        cb = ((-43 * r - 85 * g + 128 * b) >> 8) + 128;
                        cr = ((128 * r - 107 * g - 21 * b) >> 8) + 128;
                    } else {
                        size_t offset = ((size_t)px_y * w + px_x) * 2;
                        if (offset + 1 >= len) continue;
                        uint16_t px = (uint16_t)data[offset] |
                                      ((uint16_t)data[offset + 1] << 8);
                        r = ((px >> 11) & 0x1f) * 255 / 31;
                        g = ((px >> 5) & 0x3f) * 255 / 63;
                        b = (px & 0x1f) * 255 / 31;
                        y = (77 * r + 150 * g + 29 * b) >> 8;
                        cb = ((-43 * r - 85 * g + 128 * b) >> 8) + 128;
                        cr = ((128 * r - 107 * g - 21 * b) >> 8) + 128;
                    }
                    if (y > 45 && cb >= 72 && cb <= 142 && cr >= 125 && cr <= 185 &&
                        r > b && r + 18 > g) {
                        skin[dy * DS_W + dx] = 1;
                    }
                }
            }
        });
        if (!ok) return;

        uint8_t visited[DS_W * DS_H] = {};
        uint16_t queue[DS_W * DS_H];
        int best_count = 0, best_sum_x = 0, best_sum_y = 0;
        for (int sy = 2; sy < DS_H - 1; ++sy) {
            for (int sx = 1; sx < DS_W - 1; ++sx) {
                const int start = sy * DS_W + sx;
                if (!skin[start] || visited[start]) continue;
                int head = 0, tail = 0, count = 0, sum_x = 0, sum_y = 0;
                int min_x = sx, max_x = sx, min_y = sy, max_y = sy;
                visited[start] = 1;
                queue[tail++] = start;
                while (head < tail) {
                    int idx = queue[head++];
                    int x = idx % DS_W, y = idx / DS_W;
                    ++count; sum_x += x; sum_y += y;
                    min_x = std::min(min_x, x); max_x = std::max(max_x, x);
                    min_y = std::min(min_y, y); max_y = std::max(max_y, y);
                    const int neighbors[4] = {idx - 1, idx + 1, idx - DS_W, idx + DS_W};
                    for (int n : neighbors) {
                        if (n >= 0 && n < DS_W * DS_H && skin[n] && !visited[n]) {
                            visited[n] = 1;
                            queue[tail++] = (uint16_t)n;
                        }
                    }
                }
                int bw = max_x - min_x + 1, bh = max_y - min_y + 1;
                bool face_shape = count >= 10 && bw >= 3 && bh >= 4 &&
                                  bw * 10 >= bh * 4 && bw * 10 <= bh * 18;
                if (face_shape && count > best_count) {
                    best_count = count; best_sum_x = sum_x; best_sum_y = sum_y;
                }
            }
        }

        if (best_count == 0) {
            no_move_count_++;
            if (no_move_count_ > 8) tracking_ = false;
            return;
        }

        no_move_count_ = 0;
        if (!tracking_) {
            servo_->PauseScan();
            tracking_ = true;
        }

        float cx = (float)best_sum_x / best_count;
        float cy = (float)best_sum_y / best_count;
        float target_x = (cx - DS_W / 2.0f) / (DS_W / 2.0f);
        float target_y = (cy - DS_H / 2.0f) / (DS_H / 2.0f);

        smooth_x_ = smooth_x_ * 0.65f + target_x * 0.35f;
        smooth_y_ = smooth_y_ * 0.65f + target_y * 0.35f;

        if (fabsf(smooth_x_) < 0.03f) smooth_x_ = 0;
        if (fabsf(smooth_y_) < 0.03f) smooth_y_ = 0;

        yaw_ -= smooth_x_ * 6.0f;
        pitch_ -= smooth_y_ * 4.0f;
        if (yaw_ < -45) yaw_ = -45;
        if (yaw_ > 45) yaw_ = 45;
        if (pitch_ < 5) pitch_ = 5;
        if (pitch_ > 60) pitch_ = 60;

        servo_->MoveTo((int)yaw_, (int)pitch_, 150);
    }

    EspVideo* camera_ = nullptr;
    StackChanServo* servo_ = nullptr;
    TaskHandle_t task_ = nullptr;
    volatile bool paused_ = false;
    std::atomic<bool> enabled_{false};
    bool tracking_ = false;
    int no_move_count_ = 0;
    float yaw_ = 0.0f;
    float pitch_ = 30.0f;
    float smooth_x_ = 0.0f;
    float smooth_y_ = 0.0f;
};

bool StackChanServo::StartDance(const std::string& requested_sequence,
                                int tempo_ms, int repeat) {
    StopDance();
    for (int i = 0; i < 30 && dance_running_.load(); ++i) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (dance_running_.load() || anim_running_.load()) {
        ESP_LOGW("Servo", "Dance rejected: another animation is active");
        return false;
    }

    std::string sequence = requested_sequence;
    std::transform(sequence.begin(), sequence.end(), sequence.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    if (sequence.empty() || sequence == "happy") {
        sequence = "left,upper_right,right,upper_left,center,down,center";
    } else if (sequence == "cute") {
        sequence = "upper_left,center,upper_right,center,down,center";
    } else if (sequence == "swing") {
        sequence = "left,right,left,right,upper_left,upper_right,center";
    }

    auto* context = static_cast<DanceContext*>(heap_caps_calloc(
        1, sizeof(DanceContext), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!context) return false;
    context->servo = this;
    context->tempo_ms = (uint16_t)std::clamp(tempo_ms, 180, 1000);
    context->repeat = (uint8_t)std::clamp(repeat, 1, 5);

    auto append_step = [&](const std::string& token) -> bool {
        DanceStep step{0, 30};
        if (token == "left" || token == "l") step = {-28, 30};
        else if (token == "right" || token == "r") step = {28, 30};
        else if (token == "up" || token == "u") step = {0, 14};
        else if (token == "down" || token == "d") step = {0, 48};
        else if (token == "upper_left" || token == "ul") step = {-22, 16};
        else if (token == "upper_right" || token == "ur") step = {22, 16};
        else if (token == "lower_left" || token == "dl") step = {-22, 46};
        else if (token == "lower_right" || token == "dr") step = {22, 46};
        else if (token != "center" && token != "c") return false;
        if (context->count >= 24) return false;
        context->steps[context->count++] = step;
        return true;
    };

    std::stringstream stream(sequence);
    std::string token;
    while (std::getline(stream, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(),
            [](unsigned char c) { return std::isspace(c) != 0; }), token.end());
        if (token.empty()) continue;
        if (!append_step(token)) {
            ESP_LOGW("Servo", "Invalid or excessive dance step: %s", token.c_str());
            heap_caps_free(context);
            return false;
        }
    }
    if (context->count == 0) {
        heap_caps_free(context);
        return false;
    }

    dance_stop_.store(false);
    dance_running_.store(true);
    const BaseType_t result = xTaskCreatePinnedToCoreWithCaps(
        DanceTaskFunc, "servo_dance", 4096, context, 2, &dance_task_, 1,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (result != pdPASS) {
        dance_running_.store(false);
        dance_task_ = nullptr;
        heap_caps_free(context);
        return false;
    }
    ESP_LOGI("Servo", "Dance started: steps=%u repeat=%u tempo=%u",
             context->count, context->repeat, context->tempo_ms);
    return true;
}

void StackChanServo::StopDance() {
    if (dance_running_.load()) dance_stop_.store(true);
}

void StackChanServo::DanceTaskFunc(void* arg) {
    auto* context = static_cast<DanceContext*>(arg);
    context->servo->DanceLoop(context);
    heap_caps_free(context);
    vTaskDeleteWithCaps(nullptr);
}

void StackChanServo::DanceLoop(DanceContext* context) {
    if (tracker_) tracker_->Pause(false);
    for (int round = 0; round < context->repeat && !dance_stop_.load(); ++round) {
        for (int i = 0; i < context->count && !dance_stop_.load(); ++i) {
            MoveTo(context->steps[i].yaw, context->steps[i].pitch,
                   context->tempo_ms);
            int remaining = context->tempo_ms + 70;
            while (remaining > 0 && !dance_stop_.load()) {
                const int slice = std::min(remaining, 50);
                vTaskDelay(pdMS_TO_TICKS(slice));
                remaining -= slice;
            }
        }
    }
    MoveTo(0, 30, 450);
    vTaskDelay(pdMS_TO_TICKS(470));
    dance_task_ = nullptr;
    dance_running_.store(false);
    dance_stop_.store(false);
    if (tracker_) tracker_->Resume();
    ESP_LOGI("Servo", "Dance finished");
}

void StackChanServo::Nod() {
    StopDance();
    EnqueueAnimation(ServoAnimation::Nod);
}

void StackChanServo::Shake() {
    StopDance();
    EnqueueAnimation(ServoAnimation::Shake);
}

void StackChanServo::Tilt() {
    StopDance();
    EnqueueAnimation(ServoAnimation::Tilt);
}

void StackChanServo::Pet() {
    StopDance();
    EnqueueAnimation(ServoAnimation::Pet);
}

bool StackChanServo::EnqueueAnimation(ServoAnimation animation) {
    if (!anim_queue_) {
        ESP_LOGE("Servo", "Animation queue is unavailable");
        return false;
    }
    if (xQueueSend(anim_queue_, &animation, 0) != pdPASS) {
        ESP_LOGW("Servo", "Animation queue full; dropping request %u",
                 (unsigned)animation);
        return false;
    }
    anim_running_.store(true);
    ESP_LOGI("Servo", "Animation queued: %u (waiting=%u)",
             (unsigned)animation, (unsigned)uxQueueMessagesWaiting(anim_queue_));
    return true;
}

void StackChanServo::AnimationTaskFunc(void* arg) {
    static_cast<StackChanServo*>(arg)->AnimationLoop();
    vTaskDelete(nullptr);
}

void StackChanServo::AnimationLoop() {
    ServoAnimation animation;
    while (true) {
        if (xQueueReceive(anim_queue_, &animation, portMAX_DELAY) != pdPASS) continue;
        anim_running_.store(true);
        if (tracker_) tracker_->Pause(false);
        ESP_LOGI("Servo", "Animation start: %u", (unsigned)animation);
        RunAnimation(animation);
        ESP_LOGI("Servo", "Animation done: %u", (unsigned)animation);
        if (uxQueueMessagesWaiting(anim_queue_) == 0) {
            anim_running_.store(false);
            if (tracker_) tracker_->Resume();
        }
    }
}

void StackChanServo::RunAnimation(ServoAnimation animation) {
    const int y = tracker_ ? (int)tracker_->GetYaw() : 0;
    const int p = tracker_ ? (int)tracker_->GetPitch() : 30;
    switch (animation) {
    case ServoAnimation::Nod:
        MoveTo(y, p - 10, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        MoveTo(y, p + 5, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        MoveTo(y, p - 8, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        MoveTo(y, p, 300);
        vTaskDelay(pdMS_TO_TICKS(300));
        break;
    case ServoAnimation::Shake:
        MoveTo(y - 15, p, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        MoveTo(y + 15, p, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        MoveTo(y - 10, p, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        MoveTo(y, p, 300);
        vTaskDelay(pdMS_TO_TICKS(300));
        break;
    case ServoAnimation::Tilt:
        MoveTo(y + 10, p - 10, 400);
        vTaskDelay(pdMS_TO_TICKS(1500));
        MoveTo(y, p, 500);
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
    case ServoAnimation::Pet: {
        // Cycle instead of relying on random choice.  The previous 2-up/1-side
        // distribution could easily produce several lifts in a row and made
        // the affectionate side motion look missing on the real device.
        static uint8_t pet_action = 0;
        const int action = pet_action++ % 3;
        if (action == 0) {
            // Clearly visible affectionate turn to the left.
            MoveTo(y - 28, p + 4, 450);
        } else if (action == 1) {
            // Happy upward motion.
            MoveTo(y, p + 28, 450);
        } else {
            // Clearly visible affectionate turn to the right.
            MoveTo(y + 28, p + 4, 450);
        }
        ESP_LOGI("Servo", "Pet motion variant: %d (left/up/right)", action);
        vTaskDelay(pdMS_TO_TICKS(1300));
        MoveTo(y, p, 350);
        vTaskDelay(pdMS_TO_TICKS(400));
        break;
    }
    }
}

static bool EnableServoPowerViaPy32(i2c_master_bus_handle_t i2c_bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x6F,
        .scl_speed_hz = 100000,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = 0 },
    };
    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PY32: failed to add device: %s", esp_err_to_name(err));
        return false;
    }

    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        uint8_t reg = 0x02;
        uint8_t ver = 0;
        err = i2c_master_transmit_receive(dev, &reg, 1, &ver, 1, 200);
        if (err == ESP_OK && ver != 0 && ver != 0xFF) {
            ESP_LOGI(TAG, "PY32 found! version=%d, enabling VM_EN", ver);
            uint8_t buf[2];
            reg = 0x03; i2c_master_transmit_receive(dev, &reg, 1, &buf[1], 1, 200);
            buf[0] = 0x03; buf[1] |= 0x01; i2c_master_transmit(dev, buf, 2, 200);
            reg = 0x09; i2c_master_transmit_receive(dev, &reg, 1, &buf[1], 1, 200);
            buf[0] = 0x09; buf[1] |= 0x01; i2c_master_transmit(dev, buf, 2, 200);
            reg = 0x05; i2c_master_transmit_receive(dev, &reg, 1, &buf[1], 1, 200);
            buf[0] = 0x05; buf[1] |= 0x01; i2c_master_transmit(dev, buf, 2, 200);
            ESP_LOGI(TAG, "Servo power enabled (VM_EN)");
            return true;
        }
        ESP_LOGD(TAG, "PY32 attempt %d: err=%s ver=0x%02X", i, esp_err_to_name(err), ver);
    }

    ESP_LOGW(TAG, "PY32 not found after 10 attempts");
    i2c_master_bus_rm_device(dev);
    return false;
}

namespace shizhou_avatar {

enum class Expression {
    Neutral, Happy, Angry, Sad, Sleepy,
    Loving, Crying,
    Kissy, Cool, Confident,
    Shocked, Thinking, Surprised, Confused,
    Embarrassed, Silly, Winking, Laughing, Funny, Relaxed, Delicious
};

struct Overlay {
    bool tear = false;
    bool heart_eyes = false;
    bool kiss_heart = false;
    bool cheek_blush = false;
    bool cool_glasses = false;
    bool excl_mark = false;
    bool think_bubble = false;
    bool star_burst = false;
    bool wave_squiggle = false;
    bool drool = false;
    bool laugh_lines = false;
    bool question_mark = false;
    bool zzz = false;
};

class LvglAvatar {
public:
    LvglAvatar() = default;
    ~LvglAvatar() { Destroy(); }

    bool Init(lv_obj_t* parent, int w, int h,
              const std::shared_ptr<EmojiCollection>& collection = nullptr) {
        if (canvas_ || image_) return true;
        w_ = w; h_ = h;

        // Prefer the illustrated child avatar stored in the assets partition.
        // The CoreS3 UI must never briefly replace the configured avatar with the generic
        // vector face.  Callers therefore wait until the complete illustrated
        // collection is mounted instead of accepting the vector fallback.
        if (LoadImageFrames(collection)) {
            // Keep the collection alive for as long as the avatar uses the
            // frame pointers.  Assets::Apply() replaces the display theme and
            // may otherwise destroy the old EmojiCollection, leaving every
            // cached LvglImage pointer below dangling.
            image_collection_ = collection;
            image_ = lv_image_create(parent);
            lv_image_set_src(image_, frame_neutral_->image_dsc());
            lv_obj_align(image_, LV_ALIGN_TOP_LEFT, 0, 0);
            lv_obj_clear_flag(image_, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_move_background(image_);
            current_frame_ = frame_neutral_;
            image_mode_ = true;
            timer_ = lv_timer_create(&LvglAvatar::TimerCb, 50, this);
            next_blink_ms_ = lv_tick_get() + 2600;
            return true;
        }

        size_t bytes = (size_t)w * h * 2;
        buf_ = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
        if (!buf_) return false;
        canvas_ = lv_canvas_create(parent);
        lv_canvas_set_buffer(canvas_, buf_, w, h, LV_COLOR_FORMAT_RGB565);
        lv_obj_align(canvas_, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_clear_flag(canvas_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_move_background(canvas_);
        timer_ = lv_timer_create(&LvglAvatar::TimerCb, 50, this);
        next_blink_ms_ = 3000;
        last_saccade_ms_ = 0;
        Draw();
        return true;
    }

    void Destroy() {
        if (timer_) { lv_timer_delete(timer_); timer_ = nullptr; }
        if (image_) { lv_obj_delete(image_); image_ = nullptr; }
        if (canvas_) { lv_obj_delete(canvas_); canvas_ = nullptr; }
        if (buf_)   { heap_caps_free(buf_); buf_ = nullptr; }
        current_frame_ = nullptr;
        frame_neutral_ = nullptr;
        frame_blink_half_ = nullptr;
        frame_blink_closed_ = nullptr;
        frame_talk_small_ = nullptr;
        frame_talk_medium_ = nullptr;
        frame_talk_wide_ = nullptr;
        frame_happy_ = nullptr;
        frame_angry_ = nullptr;
        frame_sad_ = nullptr;
        frame_thinking_ = nullptr;
        frame_surprised_ = nullptr;
        image_collection_.reset();
    }

    bool IsReady() const { return canvas_ != nullptr || image_ != nullptr; }
    bool IsImageMode() const { return image_mode_; }
    void SetVisible(bool visible) {
        if (image_) {
            if (visible) lv_obj_remove_flag(image_, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(image_, LV_OBJ_FLAG_HIDDEN);
        }
        if (canvas_) {
            if (visible) lv_obj_remove_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void SetExpression(Expression e) { expression_ = e; }
    void SetOverlay(const Overlay& o) { overlay_ = o; }
    Expression GetExpression() const { return expression_; }
    Overlay GetOverlay() const { return overlay_; }
    void StartSpeaking(uint32_t duration_ms) {
        speaking_until_ms_ = lv_tick_get() + duration_ms;
    }
    void StopSpeaking() { speaking_until_ms_ = 0; }

private:
    static void TimerCb(lv_timer_t* t) {
        static_cast<LvglAvatar*>(lv_timer_get_user_data(t))->OnTick();
    }

    void UpdateBreathParams() {
        breath_amp_ = 3.0f;
        breath_period_steps_ = 100;
        breath_paused_ = false;
        switch (expression_) {
            case Expression::Relaxed:
                breath_amp_ = 7.0f;
                breath_period_steps_ = 160;
                break;
            case Expression::Shocked:
                breath_paused_ = true;
                break;
            default: break;
        }
    }

    bool BlinkAllowed() const {
        switch (expression_) {
            case Expression::Cool:
            case Expression::Confident:
            case Expression::Shocked:
            case Expression::Winking:
            case Expression::Kissy:
                return false;
            default:
                return true;
        }
    }

    bool SlowBlink() const {
        return expression_ == Expression::Thinking || expression_ == Expression::Relaxed;
    }

    bool SaccadeEnabled() const {
        switch (expression_) {
            case Expression::Cool:
            case Expression::Confident:
            case Expression::Shocked:
            case Expression::Thinking:
            case Expression::Embarrassed:
            case Expression::Winking:
                return false;
            default:
                return true;
        }
    }

    void GetGazeOverride(float* gh, float* gv) const {
        switch (expression_) {
            case Expression::Thinking:
                *gh = 0; *gv = -1.0f; break;
            case Expression::Embarrassed:
                *gh = 0; *gv = 0.7f; break;
            default:
                *gh = gaze_h_; *gv = gaze_v_;
        }
    }

    void OnTick() {
        tick_count_++;
        uint32_t now = lv_tick_get();

        UpdateBreathParams();
        if (breath_paused_) {
            breath_ = 0;
        } else {
            breath_ = sinf((tick_count_ % breath_period_steps_) * 2.0f * 3.14159265f / breath_period_steps_);
        }

        if (BlinkAllowed()) {
            if (now >= next_blink_ms_) {
                uint32_t mult = SlowBlink() ? 2 : 1;
                if (eye_closed_) {
                    eye_open_ratio_ = 1.0f;
                    next_blink_ms_ = now + mult * (2500 + (rand() % 2000));
                    eye_closed_ = false;
                } else {
                    eye_open_ratio_ = 0.0f;
                    next_blink_ms_ = now + 150 + (rand() % 200);
                    eye_closed_ = true;
                }
            }
        } else {
            eye_open_ratio_ = 1.0f;
            eye_closed_ = false;
        }

        if (SaccadeEnabled() && now - last_saccade_ms_ > 1500) {
            gaze_h_ = (rand() % 21 - 10) / 10.0f;
            gaze_v_ = (rand() % 21 - 10) / 10.0f;
            last_saccade_ms_ = now;
        }

        bool speaking = (speaking_until_ms_ != 0 && now < speaking_until_ms_);
        if (!speaking && speaking_until_ms_ != 0) speaking_until_ms_ = 0;
        if (speaking) {
            mouth_open_ = 0.2f + (rand() % 80) / 100.0f;
        } else {
            mouth_open_ = 0.0f;
        }

        if (image_mode_) {
            UpdateImageFrame(now, speaking);
        } else {
            Draw();
        }
    }

    bool LoadImageFrames(const std::shared_ptr<EmojiCollection>& collection) {
        if (!collection) return false;
        frame_neutral_ = collection->GetEmojiImage("neutral");
        frame_blink_half_ = collection->GetEmojiImage("blink_half");
        frame_blink_closed_ = collection->GetEmojiImage("blink_closed");
        frame_talk_small_ = collection->GetEmojiImage("talk_small");
        frame_talk_medium_ = collection->GetEmojiImage("talk_medium");
        frame_talk_wide_ = collection->GetEmojiImage("talk_wide");
        frame_happy_ = collection->GetEmojiImage("happy");
        frame_loving_ = collection->GetEmojiImage("loving");
        if (!frame_loving_) frame_loving_ = frame_happy_;
        frame_angry_ = collection->GetEmojiImage("angry");
        frame_sad_ = collection->GetEmojiImage("sad");
        frame_thinking_ = collection->GetEmojiImage("thinking");
        frame_surprised_ = collection->GetEmojiImage("surprised");
        return frame_neutral_ && frame_blink_half_ && frame_blink_closed_
            && frame_talk_small_ && frame_talk_medium_ && frame_talk_wide_
            && frame_happy_ && frame_angry_ && frame_sad_
            && frame_thinking_ && frame_surprised_;
    }

    const LvglImage* ExpressionImage() const {
        switch (expression_) {
            case Expression::Happy:
            case Expression::Kissy:
            case Expression::Confident:
            case Expression::Silly:
            case Expression::Winking:
            case Expression::Laughing:
            case Expression::Funny:
            case Expression::Delicious:
                return frame_happy_;
            case Expression::Loving:
                return frame_loving_;
            case Expression::Angry:
                return frame_angry_;
            case Expression::Sad:
            case Expression::Crying:
            case Expression::Embarrassed:
                return frame_sad_;
            case Expression::Thinking:
            case Expression::Confused:
            case Expression::Cool:
            case Expression::Relaxed:
                return frame_thinking_;
            case Expression::Shocked:
            case Expression::Surprised:
                return frame_surprised_;
            case Expression::Sleepy:
                return frame_blink_closed_;
            default:
                return frame_neutral_;
        }
    }

    void SetImageFrame(const LvglImage* frame) {
        if (!image_ || !frame || frame == current_frame_) return;
        lv_image_set_src(image_, frame->image_dsc());
        current_frame_ = frame;
    }

    void UpdateImageFrame(uint32_t now, bool speaking) {
        if (speaking) {
            switch (tick_count_ % 4) {
                case 0: SetImageFrame(frame_talk_small_); break;
                case 1: SetImageFrame(frame_talk_medium_); break;
                case 2: SetImageFrame(frame_talk_wide_); break;
                default: SetImageFrame(frame_talk_medium_); break;
            }
            return;
        }

        if (expression_ != Expression::Neutral) {
            SetImageFrame(ExpressionImage());
            return;
        }

        // The existing blink scheduler closes the eyes for 150-350 ms.  Use
        // the half-closed keyframe at both edges to avoid a hard two-frame pop.
        if (eye_closed_) {
            SetImageFrame((tick_count_ & 1) ? frame_blink_half_ : frame_blink_closed_);
        } else {
            SetImageFrame(frame_neutral_);
        }
    }

    void Draw() {
        if (!canvas_) return;
        const lv_color_t bg = lv_color_make(0x00, 0x00, 0x00);
        const lv_color_t fg = lv_color_make(0xFF, 0xFF, 0xFF);

        lv_canvas_fill_bg(canvas_, bg, LV_OPA_COVER);
        lv_layer_t layer;
        lv_canvas_init_layer(canvas_, &layer);

        DrawMouth(&layer, fg, bg);
        DrawEye(&layer, fg, bg, false);
        DrawEye(&layer, fg, bg, true);
        DrawOverlay(&layer, fg, bg);

        lv_canvas_finish_layer(canvas_, &layer);
    }

    void FillRect(lv_layer_t* layer, int x, int y, int w, int h, lv_color_t c) {
        if (w <= 0 || h <= 0) return;
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = c;
        d.bg_opa = LV_OPA_COVER;
        d.radius = 0;
        d.border_width = 0;
        lv_area_t a = {x, y, x + w - 1, y + h - 1};
        lv_draw_rect(layer, &d, &a);
    }

    void FillCircle(lv_layer_t* layer, int cx, int cy, int r, lv_color_t c) {
        if (r <= 0) return;
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = c;
        d.bg_opa = LV_OPA_COVER;
        d.radius = LV_RADIUS_CIRCLE;
        d.border_width = 0;
        lv_area_t a = {cx - r, cy - r, cx + r - 1, cy + r - 1};
        lv_draw_rect(layer, &d, &a);
    }

    void FillTriangle(lv_layer_t* layer, int x0, int y0, int x1, int y1, int x2, int y2, lv_color_t c) {
        lv_draw_triangle_dsc_t d;
        lv_draw_triangle_dsc_init(&d);
        d.p[0].x = (float)x0; d.p[0].y = (float)y0;
        d.p[1].x = (float)x1; d.p[1].y = (float)y1;
        d.p[2].x = (float)x2; d.p[2].y = (float)y2;
        d.color = c;
        d.opa = LV_OPA_COVER;
        lv_draw_triangle(layer, &d);
    }

    void FillRoundRect(lv_layer_t* layer, int x, int y, int w, int h, int radius, lv_color_t c) {
        if (w <= 0 || h <= 0) return;
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = c;
        d.bg_opa = LV_OPA_COVER;
        d.radius = radius;
        d.border_width = 0;
        lv_area_t a = {x, y, x + w - 1, y + h - 1};
        lv_draw_rect(layer, &d, &a);
    }

    void DrawArc(lv_layer_t* layer, int cx, int cy, int r, int start_deg, int end_deg, int width, lv_color_t c, bool rounded = false) {
        lv_draw_arc_dsc_t d;
        lv_draw_arc_dsc_init(&d);
        d.color = c;
        d.opa = LV_OPA_COVER;
        d.width = width;
        d.center.x = cx;
        d.center.y = cy;
        d.radius = r;
        d.start_angle = start_deg;
        d.end_angle = end_deg;
        d.rounded = rounded ? 1 : 0;
        lv_draw_arc(layer, &d);
    }

    void DrawLine(lv_layer_t* layer, int x1, int y1, int x2, int y2, int width, bool round, lv_color_t c) {
        lv_draw_line_dsc_t d;
        lv_draw_line_dsc_init(&d);
        d.color = c;
        d.opa = LV_OPA_COVER;
        d.width = width;
        d.round_start = round ? 1 : 0;
        d.round_end = round ? 1 : 0;
        d.p1.x = (float)x1; d.p1.y = (float)y1;
        d.p2.x = (float)x2; d.p2.y = (float)y2;
        lv_draw_line(layer, &d);
    }

    void DrawMouth(lv_layer_t* layer, lv_color_t fg, lv_color_t bg) {
        const int cx = 163;
        const int cy = 148 + (int)(breath_ * 3.0f);
        const int y_off = (int)(breath_ * 2.0f);

        switch (expression_) {
            case Expression::Cool:
                DrawLine(layer, cx - 12, cy + y_off + 2, cx + 12, cy + y_off, 3, true, fg);
                return;
            case Expression::Confident:
                DrawLine(layer, cx - 14, cy + y_off + 3, cx + 14, cy + y_off, 3, true, fg);
                return;
            case Expression::Silly:
                DrawLine(layer, cx - 13, cy + y_off + 4, cx + 13, cy + y_off, 3, true, fg);
                return;
            case Expression::Embarrassed:
                DrawLine(layer, cx - 12, cy + y_off, cx + 12, cy + y_off, 3, true, fg);
                return;
            case Expression::Kissy: {
                int my = cy + y_off;
                DrawArc(layer, cx, my - 6, 6, 270, 450, 3, fg, false);
                DrawArc(layer, cx, my + 6, 6, 270, 450, 3, fg, false);
                FillCircle(layer, cx, my, 2, fg);
                return;
            }
            case Expression::Winking:
                DrawArc(layer, cx, cy + y_off - 5, 12, 0, 180, 3, fg);
                return;
            case Expression::Laughing: {
                int y_top = cy + y_off - 28;
                FillRoundRect(layer, cx - 40, y_top, 80, 30, 12, fg);
                return;
            }
            case Expression::Funny:
                FillRoundRect(layer, cx - 25, cy + y_off - 10, 50, 20, 8, fg);
                return;
            case Expression::Relaxed:
                DrawLine(layer, cx - 20, cy + y_off, cx + 20, cy + y_off, 4, true, fg);
                return;
            case Expression::Delicious: {
                int h = 4 + (int)((60 - 4) * 0.3f);
                int w = 50 + (int)((90 - 50) * 0.7f);
                FillRoundRect(layer, cx - w / 2, cy + y_off - h / 2, w, h, 7, fg);
                return;
            }
            case Expression::Shocked: {
                FillRoundRect(layer, cx - 25, cy + y_off - 30, 50, 60, 10, fg);
                return;
            }
            case Expression::Surprised: {
                int h = 4 + (int)((60 - 4) * 0.5f);
                int w = 50 + (int)((90 - 50) * 0.5f);
                FillRoundRect(layer, cx - w / 2, cy + y_off - h / 2, w, h, 12, fg);
                return;
            }
            case Expression::Thinking: {
                DrawLine(layer, cx - 15, cy + y_off, cx + 15, cy + y_off, 3, true, fg);
                return;
            }
            case Expression::Confused: {
                DrawLine(layer, cx - 15, cy + y_off, cx + 15, cy + y_off + 2, 3, true, fg);
                return;
            }
            default: {
                int h = 4 + (int)((60 - 4) * mouth_open_);
                int w = 50 + (int)((90 - 50) * (1.0f - mouth_open_));
                int radius = (int)(mouth_open_ * 10);
                FillRoundRect(layer, cx - w / 2, cy + y_off - h / 2, w, h, radius, fg);
                return;
            }
        }
    }

    void DrawEye(lv_layer_t* layer, lv_color_t fg, lv_color_t bg, bool is_left) {
        if (expression_ == Expression::Cool) return;

        const int cx_base = is_left ? 230 : 90;
        const int cy_base_y = is_left ? 96 : 93;
        const int cy = cy_base_y + (int)(breath_ * 3.0f);

        float gh, gv;
        GetGazeOverride(&gh, &gv);
        const int off_x = (int)(gh * 3.0f);
        const int off_y = (int)(gv * 3.0f);

        if (overlay_.heart_eyes) {
            const lv_color_t red = lv_color_make(0xFF, 0x40, 0x70);
            int hcx = cx_base + off_x;
            int hcy = cy + off_y;
            FillCircle(layer, hcx - 6, hcy - 3, 7, red);
            FillCircle(layer, hcx + 6, hcy - 3, 7, red);
            FillTriangle(layer, hcx - 12, hcy + 1, hcx + 12, hcy + 1, hcx, hcy + 13, red);
            return;
        }

        if (expression_ == Expression::Shocked) {
            FillCircle(layer, cx_base, cy, 13, fg);
            FillCircle(layer, cx_base, cy, 3, bg);
            return;
        }

        if (expression_ == Expression::Surprised) {
            FillCircle(layer, cx_base, cy, 10, fg);
            return;
        }

        if (expression_ == Expression::Confused) {
            int r = is_left ? 8 : 6;
            FillCircle(layer, cx_base + off_x, cy + off_y, r, fg);
            return;
        }

        if (expression_ == Expression::Winking) {
            if (is_left) {
                DrawLine(layer, cx_base + 8, cy - 4, cx_base - 8, cy, 5, true, fg);
                DrawLine(layer, cx_base - 8, cy, cx_base + 8, cy + 4, 5, true, fg);
            } else {
                FillCircle(layer, cx_base, cy, 8, fg);
            }
            return;
        }

        if (expression_ == Expression::Silly) {
            int r = 8;
            FillCircle(layer, cx_base + off_x, cy + off_y, r, fg);
            int x0 = cx_base + off_x - r;
            int y0 = cy + off_y;
            int w = r * 2 + 4;
            int h = r + 2;
            if (!is_left) h += 2;
            FillCircle(layer, cx_base + off_x, cy + off_y, (int)(r / 1.5f), bg);
            FillRect(layer, x0, y0, w, h, bg);
            return;
        }

        if (expression_ == Expression::Laughing) {
            int r = 8;
            FillCircle(layer, cx_base + off_x, cy + off_y, r, fg);
            int x0 = cx_base + off_x - r - 2;
            int y0 = cy + off_y;
            int w = r * 2 + 8;
            int h = r + 4;
            FillCircle(layer, cx_base + off_x, cy + off_y, (int)(r / 1.5f), bg);
            FillRect(layer, x0, y0, w, h, bg);
            return;
        }

        if (expression_ == Expression::Sleepy) {
            if (is_left) {
                DrawLine(layer, cx_base - 8 + off_x, cy - 2 + off_y,
                                cx_base + 8 + off_x, cy + 2 + off_y, 4, true, fg);
            } else {
                DrawLine(layer, cx_base - 8 + off_x, cy + 2 + off_y,
                                cx_base + 8 + off_x, cy - 2 + off_y, 4, true, fg);
            }
            return;
        }

        if (expression_ == Expression::Relaxed) {
            int r = 8;
            FillCircle(layer, cx_base + off_x, cy + off_y, r, fg);
            int x0 = cx_base + off_x - r - 1;
            int y0 = cy + off_y - 1;
            int w = r * 2 + 6;
            int h = r + 3;
            FillCircle(layer, cx_base + off_x, cy + off_y, (int)(r / 1.5f), bg);
            FillRect(layer, x0, y0, w, h, bg);
            return;
        }

        const int r = 8;

        if (eye_open_ratio_ > 0) {
            FillCircle(layer, cx_base + off_x, cy + off_y, r, fg);

            if (expression_ == Expression::Angry || expression_ == Expression::Sad || expression_ == Expression::Crying) {
                int x0 = cx_base + off_x - r;
                int y0 = cy + off_y - r;
                int x1 = x0 + r * 2;
                int y1 = y0;
                bool sad = (expression_ == Expression::Sad || expression_ == Expression::Crying);
                int x2 = ((!is_left) != (!sad)) ? x0 : x1;
                int y2 = y0 + r;
                FillTriangle(layer, x0, y0, x1, y1, x2, y2, bg);
            }

            if (expression_ == Expression::Happy
                || expression_ == Expression::Kissy || expression_ == Expression::Funny
                || expression_ == Expression::Delicious) {
                FillCircle(layer, cx_base + off_x, cy + off_y, r + 2, bg);
                DrawArc(layer, cx_base + off_x, cy + off_y + r,
                        r, 180, 360, 3, fg, true);
            }
        } else {
            FillRect(layer, cx_base - r + off_x, cy - 2 + off_y, r * 2, 4, fg);
        }
    }

    void DrawOverlay(lv_layer_t* layer, lv_color_t fg, lv_color_t bg) {
        if (overlay_.tear) {
            const lv_color_t blue = lv_color_make(0x40, 0xA0, 0xFF);
            int tx = 90;
            int ty = 115 + (int)(breath_ * 3.0f);
            FillCircle(layer, tx, ty, 7, blue);
            FillTriangle(layer, tx - 6, ty - 2, tx + 6, ty - 2, tx, ty - 15, blue);
        }

        if (overlay_.cheek_blush) {
            const lv_color_t pink = lv_color_make(0xFF, 0x64, 0x82);
            for (int i = 0; i < 3; i++) {
                int x_start = 47 + i * 8;
                int x_end = x_start + 6;
                DrawLine(layer, x_start, 138, x_end, 130, 3, true, pink);
            }
            for (int i = 0; i < 3; i++) {
                int x_start = 251 + i * 8;
                int x_end = x_start + 6;
                DrawLine(layer, x_start, 138, x_end, 130, 3, true, pink);
            }
        }

        if (overlay_.cool_glasses) {
            FillRoundRect(layer, 85, 84, 50, 24, 5, fg);
            FillRoundRect(layer, 185, 84, 50, 24, 5, fg);
            DrawLine(layer, 85, 84, 235, 84, 2, false, fg);
        }

        if (overlay_.excl_mark) {
            DrawLine(layer, 291, 50, 291, 68, 4, true, fg);
            FillCircle(layer, 291, 76, 2, fg);
        }

        if (overlay_.think_bubble) {
            FillRoundRect(layer, 245, 47, 50, 25, 12, fg);
            FillCircle(layer, 258, 60, 3, bg);
            FillCircle(layer, 270, 60, 3, bg);
            FillCircle(layer, 282, 60, 3, bg);
            FillCircle(layer, 273, 85, 6, fg);
            FillCircle(layer, 258, 110, 4, fg);
        }

        if (overlay_.star_burst) {
            const int cx_s = 290, cy_s = 60;
            FillRect(layer, cx_s - 3, cy_s - 3, 6, 6, fg);
            FillTriangle(layer, cx_s, cy_s - 18, cx_s - 3, cy_s - 3, cx_s + 3, cy_s - 3, fg);
            FillTriangle(layer, cx_s, cy_s + 18, cx_s - 3, cy_s + 3, cx_s + 3, cy_s + 3, fg);
            FillTriangle(layer, cx_s - 18, cy_s, cx_s - 3, cy_s - 3, cx_s - 3, cy_s + 3, fg);
            FillTriangle(layer, cx_s + 18, cy_s, cx_s + 3, cy_s - 3, cx_s + 3, cy_s + 3, fg);
        }

        if (overlay_.wave_squiggle) {
            DrawLine(layer, 148, 28, 154, 24, 2, true, fg);
            DrawLine(layer, 154, 24, 160, 28, 2, true, fg);
            DrawLine(layer, 160, 28, 166, 24, 2, true, fg);
            DrawLine(layer, 166, 24, 172, 28, 2, true, fg);
        }

        if (overlay_.drool) {
            const lv_color_t blue = lv_color_make(0x40, 0xA0, 0xFF);
            int dx = 143;
            int dy = 168 + (int)(breath_ * 3.0f);
            FillCircle(layer, dx, dy, 4, blue);
            FillTriangle(layer, dx - 3, dy - 2, dx + 3, dy - 2, dx, dy - 8, blue);
        }

        if (overlay_.laugh_lines) {
            DrawLine(layer, 210, 150, 220, 142, 3, true, fg);
            DrawLine(layer, 218, 156, 228, 148, 3, true, fg);
        }

        if (overlay_.question_mark) {
            DrawArc(layer, 290, 50, 7, 180, 90, 4, fg, true);
            FillCircle(layer, 290, 67, 3, fg);
        }

        if (overlay_.zzz) {
            auto draw_z = [&](int cx_z, int cy_z, int size, int w) {
                int h = size / 2;
                DrawLine(layer, cx_z - h, cy_z - h, cx_z + h, cy_z - h, w, false, fg);
                DrawLine(layer, cx_z + h, cy_z - h, cx_z - h, cy_z + h, w, false, fg);
                DrawLine(layer, cx_z - h, cy_z + h, cx_z + h, cy_z + h, w, false, fg);
            };
            draw_z(258, 80, 8, 2);
            draw_z(270, 70, 10, 3);
            draw_z(286, 55, 14, 3);
        }

        if (overlay_.kiss_heart) {
            const lv_color_t red = lv_color_make(0xFF, 0x40, 0x70);
            int hx = 195;
            int hy = 130;
            FillCircle(layer, hx - 3, hy - 1, 4, red);
            FillCircle(layer, hx + 3, hy - 1, 4, red);
            FillTriangle(layer, hx - 6, hy + 1, hx + 6, hy + 1, hx, hy + 8, red);
        }
    }

    lv_obj_t* canvas_ = nullptr;
    lv_obj_t* image_ = nullptr;
    uint8_t* buf_ = nullptr;
    lv_timer_t* timer_ = nullptr;
    int w_ = 320, h_ = 240;

    Expression expression_ = Expression::Neutral;
    Overlay overlay_;

    uint32_t tick_count_ = 0;
    uint32_t next_blink_ms_ = 0;
    uint32_t last_saccade_ms_ = 0;
    uint32_t speaking_until_ms_ = 0;
    bool eye_closed_ = false;
    float eye_open_ratio_ = 1.0f;
    float mouth_open_ = 0.0f;
    float breath_ = 0.0f;
    float gaze_h_ = 0.0f;
    float gaze_v_ = 0.0f;

    float breath_amp_ = 3.0f;
    uint32_t breath_period_steps_ = 100;
    bool breath_paused_ = false;
    bool image_mode_ = false;
    std::shared_ptr<EmojiCollection> image_collection_;
    const LvglImage* current_frame_ = nullptr;
    const LvglImage* frame_neutral_ = nullptr;
    const LvglImage* frame_blink_half_ = nullptr;
    const LvglImage* frame_blink_closed_ = nullptr;
    const LvglImage* frame_talk_small_ = nullptr;
    const LvglImage* frame_talk_medium_ = nullptr;
    const LvglImage* frame_talk_wide_ = nullptr;
    const LvglImage* frame_happy_ = nullptr;
    const LvglImage* frame_loving_ = nullptr;
    const LvglImage* frame_angry_ = nullptr;
    const LvglImage* frame_sad_ = nullptr;
    const LvglImage* frame_thinking_ = nullptr;
    const LvglImage* frame_surprised_ = nullptr;
};

static Expression MapEmotion(const char* e) {
    if (!e) return Expression::Neutral;
    if (!strcmp(e, "neutral"))     return Expression::Neutral;
    if (!strcmp(e, "happy"))       return Expression::Happy;
    if (!strcmp(e, "laughing"))    return Expression::Laughing;
    if (!strcmp(e, "funny"))       return Expression::Funny;
    if (!strcmp(e, "sad"))         return Expression::Sad;
    if (!strcmp(e, "crying"))      return Expression::Crying;
    if (!strcmp(e, "angry"))       return Expression::Angry;
    if (!strcmp(e, "loving"))      return Expression::Loving;
    if (!strcmp(e, "embarrassed")) return Expression::Embarrassed;
    if (!strcmp(e, "surprised"))   return Expression::Surprised;
    if (!strcmp(e, "shocked"))     return Expression::Shocked;
    if (!strcmp(e, "thinking"))    return Expression::Thinking;
    if (!strcmp(e, "winking"))     return Expression::Winking;
    if (!strcmp(e, "cool"))        return Expression::Cool;
    if (!strcmp(e, "relaxed"))     return Expression::Relaxed;
    if (!strcmp(e, "delicious"))   return Expression::Delicious;
    if (!strcmp(e, "kissy"))       return Expression::Kissy;
    if (!strcmp(e, "confident"))   return Expression::Confident;
    if (!strcmp(e, "sleepy"))      return Expression::Sleepy;
    if (!strcmp(e, "silly"))       return Expression::Silly;
    if (!strcmp(e, "confused"))    return Expression::Confused;
    return Expression::Neutral;
}

static Overlay OverlayFor(const char* e) {
    Overlay o;
    if (!e) return o;
    if (!strcmp(e, "crying"))      o.tear = true;
    if (!strcmp(e, "loving"))      o.heart_eyes = true;
    if (!strcmp(e, "kissy"))       o.kiss_heart = true;
    if (!strcmp(e, "embarrassed")) o.cheek_blush = true;
    if (!strcmp(e, "cool"))        o.cool_glasses = true;
    if (!strcmp(e, "shocked"))     o.excl_mark = true;
    if (!strcmp(e, "thinking"))    o.think_bubble = true;
    if (!strcmp(e, "surprised"))   o.star_burst = true;
    // silly: no overlay
    if (!strcmp(e, "delicious"))   o.drool = true;
    if (!strcmp(e, "confused"))    o.question_mark = true;
    if (!strcmp(e, "sleepy"))      o.zzz = true;
    return o;
}

}  // namespace shizhou_avatar

class M5StackAvatarDisplay : public SpiLcdDisplay {
public:
    M5StackAvatarDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                          int width, int height, int offset_x, int offset_y,
                          bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy),
          canvas_w_(width), canvas_h_(height) {
        esp_timer_create_args_t args = {};
        args.callback = &M5StackAvatarDisplay::InitTimerCallback;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "avatar_init";
        args.skip_unhandled_events = true;
        esp_timer_create(&args, &avatar_init_timer_);
        esp_timer_start_periodic(avatar_init_timer_, 500000);

        esp_timer_create_args_t idle_args = {};
        idle_args.callback = &M5StackAvatarDisplay::IdleTimerCallback;
        idle_args.arg = this;
        idle_args.dispatch_method = ESP_TIMER_TASK;
        idle_args.name = "avatar_idle";
        idle_args.skip_unhandled_events = true;
        esp_timer_create(&idle_args, &idle_timer_);

        esp_timer_create_args_t reaction_restore_args = {};
        reaction_restore_args.callback = &M5StackAvatarDisplay::ReactionRestoreTimerCallback;
        reaction_restore_args.arg = this;
        reaction_restore_args.dispatch_method = ESP_TIMER_TASK;
        reaction_restore_args.name = "avatar_react_restore";
        reaction_restore_args.skip_unhandled_events = true;
        esp_timer_create(&reaction_restore_args, &reaction_restore_timer_);

        esp_timer_create_args_t reaction_anim_args = {};
        reaction_anim_args.callback = &M5StackAvatarDisplay::ReactionAnimTimerCallback;
        reaction_anim_args.arg = this;
        reaction_anim_args.dispatch_method = ESP_TIMER_TASK;
        reaction_anim_args.name = "avatar_react_anim";
        reaction_anim_args.skip_unhandled_events = true;
        esp_timer_create(&reaction_anim_args, &reaction_anim_timer_);
    }

    void SetFaceTracker(FaceTracker* ft) { face_tracker_ = ft; }
    void SetServo(StackChanServo* s) { servo_ = s; }
    void SetLedUpdater(std::function<void(const char*)> fn) { led_updater_ = std::move(fn); }

    void SetupUI() override {
        SpiLcdDisplay::SetupUI();
        DisplayLockGuard lock(this);
        ConfigureSingleLineSubtitleLocked();
        // Never expose the stock vector face, even during the short interval
        // before the illustrated asset partition has been mounted.
        HideEmojiBoxLocked();
        CreateSpeechModeUiLocked();
        UpdateSpeechModeUiLocked();
    }

    // Local app action codes: -1 avatar UI, -2 consumed, 0..2 speech mode,
    // 10 let the assistant take/look automatically, 11 press the manual shutter,
    // 12 open the manual viewfinder, 13 toggle local face following.
    int HandleLocalAppTap(int x, int y) {
        DisplayLockGuard lock(this);
        if (local_app_page_ == LocalAppPage::Avatar) return -1;

        // In viewfinder mode the bottom control is a real shutter.  Handle it
        // before the normal bottom-strip "home" gesture.
        if (local_app_page_ == LocalAppPage::Camera &&
            camera_viewfinder_active_.load()) {
            if (x >= 12 && x <= 308 && y >= 188 && y <= 239) return 11;
            return -2;
        }

        // A generous bottom strip returns directly to the avatar.
        if (y >= 200) {
            CloseLocalAppsLocked();
            return -2;
        }

        if (local_app_page_ == LocalAppPage::Launcher) {
            if (y >= 54 && y <= 190) {
                if (x >= 14 && x <= 154) {
                    ShowSpeechModeMenuLocked();
                    return -2;
                }
                if (x >= 166 && x <= 306) {
                    ShowCameraAppLocked();
                    return -2;
                }
            }
            return -2;
        }

        if (local_app_page_ == LocalAppPage::Camera) {
            if (x >= 12 && x <= 308 && y >= 82 && y <= 144) return 13;
            return -2;
        }

        if (local_app_page_ == LocalAppPage::Dance) return -2;

        int selected = -2;
        if (x >= 20 && x <= 300) {
            if (y >= 44 && y <= 88) selected = 1;        // quiet
            else if (y >= 96 && y <= 140) selected = 0;  // normal
            else if (y >= 148 && y <= 192) selected = 2; // text only
        }
        return selected;
    }

    bool HandleLocalAppSwipe(int dx, int dy) {
        DisplayLockGuard lock(this);
        if (abs(dx) <= abs(dy)) return false;

        if (camera_viewfinder_active_.load()) {
            ExitCameraViewfinderLocked();
        }

        // Pages form one continuous carousel:
        // avatar -> sound -> camera -> dance -> avatar.
        int page = 0;
        if (local_app_page_ == LocalAppPage::Speech) page = 1;
        else if (local_app_page_ == LocalAppPage::Camera) page = 2;
        else if (local_app_page_ == LocalAppPage::Dance) page = 3;
        else if (local_app_page_ != LocalAppPage::Avatar) return false;

        constexpr int kPageCount = 4;
        page = (page + (dx > 0 ? 1 : -1) + kPageCount) % kPageCount;
        if (page == 0) CloseLocalAppsLocked();
        else if (page == 1) ShowSpeechModeMenuLocked();
        else if (page == 2) ShowCameraAppLocked();
        else ShowDanceAppLocked();
        ESP_LOGI(TAG, "Local page swipe -> %d/%d", page + 1, kPageCount);
        return true;
    }

    void SetSpeechModeUi(int mode) {
        DisplayLockGuard lock(this);
        speech_mode_ = std::clamp(mode, 0, 2);
        UpdateSpeechModeUiLocked();
    }

    void SetFaceFollowUi(bool enabled) {
        DisplayLockGuard lock(this);
        face_follow_enabled_ = enabled;
        UpdateCameraAppLocked();
    }

    void SetCameraAppStatus(const char* text) {
        DisplayLockGuard lock(this);
        if (camera_status_label_) lv_label_set_text(camera_status_label_, text ? text : "");
    }

    void SetDanceStatus(const char* text) {
        DisplayLockGuard lock(this);
        if (dance_status_label_) lv_label_set_text(dance_status_label_, text ? text : "");
    }

    bool IsDancePage() const {
        return local_app_page_ == LocalAppPage::Dance;
    }

    void ApplyDanceAvatar(int eye_weight, int mouth_weight) {
        DisplayLockGuard lock(this);
        if (mouth_weight >= 65) {
            avatar_.SetExpression(shizhou_avatar::Expression::Laughing);
        } else if (eye_weight <= 60 || mouth_weight >= 25) {
            avatar_.SetExpression(shizhou_avatar::Expression::Happy);
        } else {
            avatar_.SetExpression(shizhou_avatar::Expression::Neutral);
        }
    }

    bool IsCameraViewfinderActive() const {
        return camera_viewfinder_active_.load();
    }

    void SetCameraViewfinderActive(bool active) {
        if (!active) {
            // Clear the repeating preview before repainting the normal camera
            // page.  Call the base implementation to avoid re-entering this
            // class's viewfinder-aware SetPreviewImage override.
            SpiLcdDisplay::SetPreviewImage(nullptr);
        }
        DisplayLockGuard lock(this);
        camera_viewfinder_active_.store(active);
        UpdateCameraViewfinderUiLocked(active);
    }

    void OnPetted() {
        if (!avatar_.IsReady()) return;
        DisplayLockGuard lock(this);
        // Match the official HeadPetModifier: happy face, heart/blush overlay,
        // and a small head movement.  The servo movement also produces the
        // familiar physical chirp; it is not a canned notification sound.
        BeginReactionLocked(3000, false);
        avatar_.SetExpression(shizhou_avatar::Expression::Loving);
        shizhou_avatar::Overlay o;
        o.heart_eyes = true;
        o.cheek_blush = true;
        avatar_.SetOverlay(o);
        SetActiveLocked(true);
        BumpIdleTimerLocked();
        if (servo_) servo_->Pet();
    }

    void OnShaken() {
        if (!avatar_.IsReady()) return;
        DisplayLockGuard lock(this);
        // The official reaction is animated for four seconds: hidden/dizzy
        // eyes, blush and a rocking mouth.  The illustrated avatar has fixed
        // frames, so cycle its closest three frames instead of freezing one.
        BeginReactionLocked(4000, true);
        reaction_step_ = 0;
        ApplyShakeFrameLocked();
        if (reaction_anim_timer_) {
            esp_timer_start_periodic(reaction_anim_timer_, 600000);
        }
        SetActiveLocked(true);
        BumpIdleTimerLocked();
    }

    void SetTheme(Theme* theme) override {
        // The display is created before the assets partition is mounted. The
        // first avatar may therefore be the vector fallback; rebuild it when
        // AssetsManager refreshes the theme with the illustrated frames.
        SpiLcdDisplay::SetTheme(theme);
        auto lvgl_theme = static_cast<LvglTheme*>(theme);
        auto collection = lvgl_theme ? lvgl_theme->emoji_collection() : nullptr;
        if (!collection || avatar_.IsImageMode()) return;

        DisplayLockGuard lock(this);
        lv_obj_t* screen = lv_screen_active();
        if (!screen || !container_) return;
        avatar_.Destroy();
        if (!avatar_.Init(screen, canvas_w_, canvas_h_, collection)) {
            ESP_LOGW("StackChanAvatar", "Avatar rebuild after theme refresh failed");
            return;
        }
        ApplyAvatarLayoutLocked();
        ESP_LOGI("StackChanAvatar", "Avatar rebuilt after assets refresh in %s mode",
                 avatar_.IsImageMode() ? "illustrated" : "vector fallback");
    }

    void SetEmotion(const char* emotion) override {
        SpiLcdDisplay::SetEmotion(emotion);
        DisplayLockGuard lock(this);
        CancelReactionLocked(false);
        HideEmojiBoxLocked();
        if (!avatar_.IsReady()) return;
        avatar_.SetExpression(shizhou_avatar::MapEmotion(emotion));
        avatar_.SetOverlay(shizhou_avatar::OverlayFor(emotion));
        if (emotion && strcmp(emotion, "sleepy") == 0) {
            SetActiveLocked(false);
            if (face_tracker_) face_tracker_->Pause(false);
            if (servo_) servo_->PauseScan();
        }
        // Emotions update the screen and LEDs only.  Do not turn the head
        // automatically; explicit MCP motion commands remain available.
        // 情绪灯联动
        if (led_updater_) led_updater_(emotion);
    }

    void SetPreviewImage(std::unique_ptr<LvglImage> image) override {
        const bool showing = image != nullptr;
        const bool viewfinder = camera_viewfinder_active_.load();
        SpiLcdDisplay::SetPreviewImage(std::move(image));
        DisplayLockGuard lock(this);
        HideEmojiBoxLocked();
        if (viewfinder) {
            avatar_.SetVisible(false);
            if (launcher_panel_) lv_obj_add_flag(launcher_panel_, LV_OBJ_FLAG_HIDDEN);
            if (speech_mode_panel_) lv_obj_add_flag(speech_mode_panel_, LV_OBJ_FLAG_HIDDEN);
            if (dance_panel_) lv_obj_add_flag(dance_panel_, LV_OBJ_FLAG_HIDDEN);
            if (preview_image_ && preview_image_cached_ && showing) {
                const auto* dsc = preview_image_cached_->image_dsc();
                if (dsc && dsc->header.w > 0 && dsc->header.h > 0) {
                    const int32_t scale_x = 256 * 320 / dsc->header.w;
                    const int32_t scale_y = 256 * 184 / dsc->header.h;
                    lv_image_set_scale(preview_image_, std::min(scale_x, scale_y));
                }
                lv_obj_set_size(preview_image_, 320, 184);
                lv_obj_align(preview_image_, LV_ALIGN_TOP_MID, 0, 2);
                lv_obj_move_foreground(preview_image_);
            }
            // The full-screen camera panel is transparent in viewfinder mode;
            // moving it above the preview keeps the bottom shutter clickable.
            if (camera_panel_) {
                lv_obj_remove_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);
                lv_obj_move_foreground(camera_panel_);
            }
            return;
        }
        avatar_.SetVisible(!showing);
        if (showing) {
            // EspVideo::Capture already converts the captured GC0308 frame to
            // RGB565 in PSRAM.  Show that exact still frame full-screen so the
            // owner can see what the assistant just looked at while the vision request
            // is being processed.
            if (launcher_panel_) lv_obj_add_flag(launcher_panel_, LV_OBJ_FLAG_HIDDEN);
            if (speech_mode_panel_) lv_obj_add_flag(speech_mode_panel_, LV_OBJ_FLAG_HIDDEN);
            if (camera_panel_) lv_obj_add_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);
            if (dance_panel_) lv_obj_add_flag(dance_panel_, LV_OBJ_FLAG_HIDDEN);
            if (preview_image_ && preview_image_cached_) {
                const auto* dsc = preview_image_cached_->image_dsc();
                if (dsc && dsc->header.w > 0 && dsc->header.h > 0) {
                    const int32_t scale_x = 256 * 320 / dsc->header.w;
                    const int32_t scale_y = 256 * 240 / dsc->header.h;
                    lv_image_set_scale(preview_image_, std::min(scale_x, scale_y));
                }
                lv_obj_set_size(preview_image_, 320, 240);
                lv_obj_align(preview_image_, LV_ALIGN_CENTER, 0, 0);
                lv_obj_move_foreground(preview_image_);
            }
        } else {
            lv_obj_t* panel = nullptr;
            if (local_app_page_ == LocalAppPage::Launcher) panel = launcher_panel_;
            else if (local_app_page_ == LocalAppPage::Speech) panel = speech_mode_panel_;
            else if (local_app_page_ == LocalAppPage::Camera) panel = camera_panel_;
            else if (local_app_page_ == LocalAppPage::Dance) panel = dance_panel_;
            if (panel) {
                lv_obj_remove_flag(panel, LV_OBJ_FLAG_HIDDEN);
                lv_obj_move_foreground(panel);
            }
        }
    }

    void SetChatMessage(const char* role, const char* content) override {
        SpiLcdDisplay::SetChatMessage(role, content);
        DisplayLockGuard lock(this);
        // LVGL deliberately keeps the previous scroll animation position when
        // label text changes.  That is useful for counters, but wrong for chat:
        // a new sentence could start at the old sentence's far-right offset and
        // appear as only its final Chinese character.  Re-applying the long mode
        // deletes the old animation and resets both label offsets to zero.
        ResetSubtitleScrollLocked();
        UpdateSubtitleDurationLocked(content);
        // LcdDisplay may reveal its generic centered vector/emoji when it
        // creates system messages (Wi-Fi setup, version checks, OTA, etc.).
        // Keep the configured avatar as the only face on every system screen.
        HideEmojiBoxLocked();
        if (!avatar_.IsReady()) return;
        bool meaningful = role && content && content[0] != '\0'
            && (strcmp(role, "user") == 0 || strcmp(role, "assistant") == 0);
        if (meaningful) {
            SetActiveLocked(true);
            BumpIdleTimerLocked();
        }
        if (role && content && content[0] != '\0' && strcmp(role, "assistant") == 0) {
            size_t n = strlen(content);
            uint32_t ms = (uint32_t)(n * 120);
            if (ms < 800) ms = 800;
            if (ms > 15000) ms = 15000;
            avatar_.StartSpeaking(ms);
        } else if (role && (strcmp(role, "user") == 0 || strcmp(role, "system") == 0)) {
            avatar_.StopSpeaking();
        }
    }

    uint32_t GetChatMessageDisplayDurationMs() const override {
        return subtitle_display_duration_ms_.load();
    }

    void SetStatus(const char* status) override {
        SpiLcdDisplay::SetStatus(status);
        if (!status) return;
        DisplayLockGuard lock(this);
        HideEmojiBoxLocked();
        if (!avatar_.IsReady()) return;
        // Face following is controlled explicitly and must survive normal
        // listening/thinking/speaking status changes.
        bool is_active = (strstr(status, "聆听")
                       || strstr(status, "说话")
                       || strstr(status, "思考")
                       || strstr(status, "连接")
                       || strstr(status, "Listening")
                       || strstr(status, "Speaking")
                       || strstr(status, "Thinking")
                       || strstr(status, "Connecting"));
        if (is_active) {
            SetActiveLocked(true);
            BumpIdleTimerLocked();
        }
    }

private:
    shizhou_avatar::LvglAvatar avatar_;
    FaceTracker* face_tracker_ = nullptr;
    StackChanServo* servo_ = nullptr;
    std::function<void(const char*)> led_updater_;
    esp_timer_handle_t avatar_init_timer_ = nullptr;
    esp_timer_handle_t idle_timer_ = nullptr;
    esp_timer_handle_t reaction_restore_timer_ = nullptr;
    esp_timer_handle_t reaction_anim_timer_ = nullptr;
    bool reaction_active_ = false;
    bool shake_reaction_ = false;
    int reaction_step_ = 0;
    shizhou_avatar::Expression reaction_prev_expression_ = shizhou_avatar::Expression::Neutral;
    shizhou_avatar::Overlay reaction_prev_overlay_;
    bool active_mode_ = false;
    int canvas_w_ = 320;
    int canvas_h_ = 240;
    int speech_mode_ = 0;
    enum class LocalAppPage { Avatar, Launcher, Speech, Camera, Dance };
    LocalAppPage local_app_page_ = LocalAppPage::Avatar;
    bool face_follow_enabled_ = false;
    lv_obj_t* speech_mode_button_ = nullptr;
    lv_obj_t* speech_mode_button_label_ = nullptr;
    lv_obj_t* launcher_panel_ = nullptr;
    lv_obj_t* speech_mode_panel_ = nullptr;
    lv_obj_t* speech_mode_rows_[3] = {};
    lv_obj_t* camera_panel_ = nullptr;
    lv_obj_t* camera_title_ = nullptr;
    lv_obj_t* camera_look_button_ = nullptr;
    lv_obj_t* camera_look_label_ = nullptr;
    lv_obj_t* camera_manual_button_ = nullptr;
    lv_obj_t* camera_manual_label_ = nullptr;
    lv_obj_t* camera_follow_button_ = nullptr;
    lv_obj_t* camera_follow_label_ = nullptr;
    lv_obj_t* camera_status_label_ = nullptr;
    lv_obj_t* camera_home_label_ = nullptr;
    lv_obj_t* dance_panel_ = nullptr;
    lv_obj_t* dance_status_label_ = nullptr;
    std::atomic<bool> camera_viewfinder_active_{false};
    lv_anim_t subtitle_scroll_anim_template_ = {};
    std::atomic<uint32_t> subtitle_display_duration_ms_{4000};
    static constexpr uint64_t IDLE_TIMEOUT_US = 8 * 1000 * 1000;

    void CreateSpeechModeUiLocked() {
        if (speech_mode_button_) return;
        lv_obj_t* screen = lv_screen_active();
        if (!screen) return;

        speech_mode_button_ = lv_obj_create(screen);
        lv_obj_set_size(speech_mode_button_, 72, 34);
        lv_obj_align(speech_mode_button_, LV_ALIGN_TOP_RIGHT, -4, 4);
        lv_obj_set_style_radius(speech_mode_button_, 12, 0);
        lv_obj_set_style_bg_color(speech_mode_button_, lv_color_hex(0x262A38), 0);
        lv_obj_set_style_bg_opa(speech_mode_button_, LV_OPA_80, 0);
        lv_obj_set_style_border_width(speech_mode_button_, 0, 0);
        lv_obj_set_style_pad_all(speech_mode_button_, 0, 0);
        lv_obj_remove_flag(speech_mode_button_, LV_OBJ_FLAG_SCROLLABLE);
        speech_mode_button_label_ = lv_label_create(speech_mode_button_);
        lv_obj_set_style_text_color(speech_mode_button_label_, lv_color_white(), 0);
        lv_label_set_text(speech_mode_button_label_, "应用");
        lv_obj_center(speech_mode_button_label_);
        // Navigation is gesture-first: keep the illustrated face completely
        // unobstructed instead of placing an "应用" button over it.
        lv_obj_add_flag(speech_mode_button_, LV_OBJ_FLAG_HIDDEN);

        launcher_panel_ = lv_obj_create(screen);
        lv_obj_set_pos(launcher_panel_, 0, 0);
        lv_obj_set_size(launcher_panel_, 320, 240);
        lv_obj_set_style_radius(launcher_panel_, 0, 0);
        lv_obj_set_style_bg_color(launcher_panel_, lv_color_hex(0xF3F0FA), 0);
        lv_obj_set_style_bg_opa(launcher_panel_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(launcher_panel_, 0, 0);
        lv_obj_set_style_pad_all(launcher_panel_, 0, 0);
        lv_obj_remove_flag(launcher_panel_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t* launcher_title = lv_label_create(launcher_panel_);
        lv_label_set_text(launcher_title, "应用");
        lv_obj_set_style_text_color(launcher_title, lv_color_hex(0x2E2742), 0);
        lv_obj_align(launcher_title, LV_ALIGN_TOP_MID, 0, 14);

        auto create_app_tile = [&](int x, uint32_t color, const char* icon, const char* name) {
            lv_obj_t* tile = lv_obj_create(launcher_panel_);
            lv_obj_set_pos(tile, x, 54);
            lv_obj_set_size(tile, 140, 136);
            lv_obj_set_style_radius(tile, 24, 0);
            lv_obj_set_style_bg_color(tile, lv_color_hex(color), 0);
            lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(tile, 0, 0);
            lv_obj_set_style_pad_all(tile, 0, 0);
            lv_obj_remove_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t* icon_label = lv_label_create(tile);
            lv_label_set_text(icon_label, icon);
            lv_obj_set_style_text_color(icon_label, lv_color_white(), 0);
            lv_obj_align(icon_label, LV_ALIGN_CENTER, 0, -20);
            lv_obj_t* name_label = lv_label_create(tile);
            lv_label_set_text(name_label, name);
            lv_obj_set_style_text_color(name_label, lv_color_white(), 0);
            lv_obj_align(name_label, LV_ALIGN_BOTTOM_MID, 0, -18);
        };
        create_app_tile(14, 0x7867A8, "VOICE", "声音模式");
        create_app_tile(166, 0x4F8DA8, "CAMERA", "相机");
        lv_obj_t* launcher_home = lv_label_create(launcher_panel_);
        lv_label_set_text(launcher_home, "━━━━  返回主页");
        lv_obj_set_style_text_color(launcher_home, lv_color_hex(0x6A617A), 0);
        lv_obj_align(launcher_home, LV_ALIGN_BOTTOM_MID, 0, -5);
        lv_obj_add_flag(launcher_panel_, LV_OBJ_FLAG_HIDDEN);

        speech_mode_panel_ = lv_obj_create(screen);
        lv_obj_set_pos(speech_mode_panel_, 0, 0);
        lv_obj_set_size(speech_mode_panel_, 320, 240);
        lv_obj_set_style_radius(speech_mode_panel_, 0, 0);
        lv_obj_set_style_bg_color(speech_mode_panel_, lv_color_hex(0xF7F3FF), 0);
        lv_obj_set_style_bg_opa(speech_mode_panel_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(speech_mode_panel_, 0, 0);
        lv_obj_set_style_pad_all(speech_mode_panel_, 0, 0);
        lv_obj_remove_flag(speech_mode_panel_, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* title = lv_label_create(speech_mode_panel_);
        lv_label_set_text(title, "声音模式");
        lv_obj_set_style_text_color(title, lv_color_hex(0x2E2742), 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

        const char* labels[] = {"轻声说话", "正常说话", "无声 · 只显示文字"};
        for (int i = 0; i < 3; ++i) {
            speech_mode_rows_[i] = lv_obj_create(speech_mode_panel_);
            lv_obj_set_pos(speech_mode_rows_[i], 20, 44 + i * 52);
            lv_obj_set_size(speech_mode_rows_[i], 280, 44);
            lv_obj_set_style_radius(speech_mode_rows_[i], 10, 0);
            lv_obj_set_style_border_width(speech_mode_rows_[i], 0, 0);
            lv_obj_set_style_pad_all(speech_mode_rows_[i], 0, 0);
            lv_obj_remove_flag(speech_mode_rows_[i], LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t* label = lv_label_create(speech_mode_rows_[i]);
            lv_label_set_text(label, labels[i]);
            lv_obj_set_style_text_color(label, lv_color_hex(0x2E2742), 0);
            lv_obj_center(label);
        }
        lv_obj_t* speech_home = lv_label_create(speech_mode_panel_);
        lv_label_set_text(speech_home, "━━━━  返回主页");
        lv_obj_set_style_text_color(speech_home, lv_color_hex(0x6A617A), 0);
        lv_obj_align(speech_home, LV_ALIGN_BOTTOM_MID, 0, -5);
        lv_obj_add_flag(speech_mode_panel_, LV_OBJ_FLAG_HIDDEN);

        camera_panel_ = lv_obj_create(screen);
        lv_obj_set_pos(camera_panel_, 0, 0);
        lv_obj_set_size(camera_panel_, 320, 240);
        lv_obj_set_style_radius(camera_panel_, 0, 0);
        lv_obj_set_style_bg_color(camera_panel_, lv_color_hex(0xEAF5F8), 0);
        lv_obj_set_style_bg_opa(camera_panel_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(camera_panel_, 0, 0);
        lv_obj_set_style_pad_all(camera_panel_, 0, 0);
        lv_obj_remove_flag(camera_panel_, LV_OBJ_FLAG_SCROLLABLE);
        camera_title_ = lv_label_create(camera_panel_);
        lv_label_set_text(camera_title_, "相机");
        lv_obj_set_style_text_color(camera_title_, lv_color_hex(0x173E4D), 0);
        lv_obj_align(camera_title_, LV_ALIGN_TOP_MID, 0, 10);

        auto create_camera_button = [&](int x, int y, const char* text) -> lv_obj_t* {
            lv_obj_t* button = lv_obj_create(camera_panel_);
            lv_obj_set_pos(button, x, y);
            lv_obj_set_size(button, 296, 44);
            lv_obj_set_style_radius(button, 16, 0);
            lv_obj_set_style_bg_color(button, lv_color_hex(0xBFE2EC), 0);
            lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(button, 0, 0);
            lv_obj_set_style_pad_all(button, 0, 0);
            lv_obj_remove_flag(button, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t* label = lv_label_create(button);
            lv_label_set_text(label, text);
            lv_obj_set_style_text_color(label, lv_color_hex(0x173E4D), 0);
            lv_obj_center(label);
            return label;
        };
        camera_look_label_ = create_camera_button(12, 42, "让助手看看");
        camera_look_button_ = lv_obj_get_parent(camera_look_label_);
        camera_manual_label_ = create_camera_button(12, 94, "自己拍照");
        camera_manual_button_ = lv_obj_get_parent(camera_manual_label_);
        // Normal camera pages no longer expose duplicate local photo actions.
        // The manual button becomes visible only while an AI camera MCP call
        // is waiting for the user to confirm the frame.
        lv_obj_add_flag(camera_look_button_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(camera_manual_button_, LV_OBJ_FLAG_HIDDEN);
        camera_follow_label_ = create_camera_button(12, 82, "人脸追踪：关");
        camera_follow_button_ = lv_obj_get_parent(camera_follow_label_);
        lv_obj_set_size(camera_follow_button_, 296, 62);
        camera_status_label_ = lv_label_create(camera_panel_);
        lv_label_set_text(camera_status_label_, "助手调用相机后会在这里取景");
        lv_obj_set_width(camera_status_label_, 290);
        lv_obj_set_style_text_align(camera_status_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(camera_status_label_, lv_color_hex(0x416775), 0);
        lv_obj_align(camera_status_label_, LV_ALIGN_BOTTOM_MID, 0, -25);
        camera_home_label_ = lv_label_create(camera_panel_);
        lv_label_set_text(camera_home_label_, "━━━━  返回主页");
        lv_obj_set_style_text_color(camera_home_label_, lv_color_hex(0x416775), 0);
        lv_obj_align(camera_home_label_, LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_add_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);

        dance_panel_ = lv_obj_create(screen);
        lv_obj_set_pos(dance_panel_, 0, 0);
        lv_obj_set_size(dance_panel_, 320, 240);
        lv_obj_set_style_radius(dance_panel_, 0, 0);
        lv_obj_set_style_bg_color(dance_panel_, lv_color_hex(0xFFF1F5), 0);
        lv_obj_set_style_bg_opa(dance_panel_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dance_panel_, 0, 0);
        lv_obj_set_style_pad_all(dance_panel_, 0, 0);
        lv_obj_remove_flag(dance_panel_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t* dance_title = lv_label_create(dance_panel_);
        lv_label_set_text(dance_title, "原厂 App 自设舞蹈");
        lv_obj_set_style_text_color(dance_title, lv_color_hex(0x672A46), 0);
        lv_obj_align(dance_title, LV_ALIGN_TOP_MID, 0, 9);

        lv_obj_t* dance_help = lv_label_create(dance_panel_);
        lv_label_set_text(dance_help,
            "打开 StackChan 手机 App\n"
            "进入 Music & Dance\n"
            "用摇杆录制连续动作");
        lv_obj_set_style_text_align(dance_help, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(dance_help, lv_color_hex(0x672A46), 0);
        lv_obj_align(dance_help, LV_ALIGN_CENTER, 0, -20);
        dance_status_label_ = lv_label_create(dance_panel_);
        lv_label_set_text(dance_status_label_, "进入此页后开启原厂舞蹈蓝牙");
        lv_obj_set_width(dance_status_label_, 300);
        lv_obj_set_style_text_align(dance_status_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(dance_status_label_, lv_color_hex(0x8A536B), 0);
        lv_obj_align(dance_status_label_, LV_ALIGN_BOTTOM_MID, 0, -32);
        lv_obj_t* dance_home = lv_label_create(dance_panel_);
        lv_label_set_text(dance_home, "━━━━  返回主页");
        lv_obj_set_style_text_color(dance_home, lv_color_hex(0x8A536B), 0);
        lv_obj_align(dance_home, LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_add_flag(dance_panel_, LV_OBJ_FLAG_HIDDEN);
    }

    void UpdateSpeechModeUiLocked() {
        // The board restores the persisted mode before SetupUI() creates the
        // local app widgets.  Keep the state now and paint it after creation.
        if (!speech_mode_rows_[0] || !speech_mode_rows_[1] || !speech_mode_rows_[2]) return;
        // UI row order is quiet, normal, text-only.
        const int row_modes[] = {1, 0, 2};
        for (int i = 0; i < 3; ++i) {
            bool selected = row_modes[i] == speech_mode_;
            lv_obj_set_style_bg_color(speech_mode_rows_[i],
                lv_color_hex(selected ? 0xD8CDF7 : 0xEEEAF5), 0);
            lv_obj_set_style_bg_opa(speech_mode_rows_[i], LV_OPA_COVER, 0);
        }
    }

    void ShowSpeechModeMenuLocked() {
        if (!speech_mode_panel_) return;
        if (camera_viewfinder_active_.load()) ExitCameraViewfinderLocked();
        HideLocalAppPanelsLocked();
        local_app_page_ = LocalAppPage::Speech;
        UpdateSpeechModeUiLocked();
        lv_obj_remove_flag(speech_mode_panel_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(speech_mode_panel_);
    }

    void HideLocalAppPanelsLocked() {
        if (launcher_panel_) lv_obj_add_flag(launcher_panel_, LV_OBJ_FLAG_HIDDEN);
        if (speech_mode_panel_) lv_obj_add_flag(speech_mode_panel_, LV_OBJ_FLAG_HIDDEN);
        if (camera_panel_) lv_obj_add_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);
        if (dance_panel_) lv_obj_add_flag(dance_panel_, LV_OBJ_FLAG_HIDDEN);
    }

    void ShowLauncherLocked() {
        if (!launcher_panel_) return;
        if (camera_viewfinder_active_.load()) ExitCameraViewfinderLocked();
        HideLocalAppPanelsLocked();
        local_app_page_ = LocalAppPage::Launcher;
        lv_obj_remove_flag(launcher_panel_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(launcher_panel_);
    }

    void ShowCameraAppLocked() {
        if (!camera_panel_) return;
        HideLocalAppPanelsLocked();
        local_app_page_ = LocalAppPage::Camera;
        UpdateCameraAppLocked();
        lv_obj_remove_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(camera_panel_);
    }

    void ShowDanceAppLocked() {
        if (!dance_panel_) return;
        HideLocalAppPanelsLocked();
        local_app_page_ = LocalAppPage::Dance;
        lv_obj_remove_flag(dance_panel_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(dance_panel_);
    }

    void CloseLocalAppsLocked() {
        if (camera_viewfinder_active_.load()) ExitCameraViewfinderLocked();
        HideLocalAppPanelsLocked();
        local_app_page_ = LocalAppPage::Avatar;
        if (speech_mode_button_) lv_obj_move_foreground(speech_mode_button_);
    }

    void UpdateCameraAppLocked() {
        if (camera_follow_label_) {
            lv_label_set_text(camera_follow_label_,
                              face_follow_enabled_ ? "人脸追踪：开" : "人脸追踪：关");
        }
    }

    void UpdateCameraViewfinderUiLocked(bool active) {
        if (!camera_panel_ || !camera_manual_button_ || !camera_manual_label_) return;
        if (active) {
            local_app_page_ = LocalAppPage::Camera;
            lv_obj_set_style_bg_opa(camera_panel_, LV_OPA_TRANSP, 0);
            if (camera_title_) lv_obj_add_flag(camera_title_, LV_OBJ_FLAG_HIDDEN);
            if (camera_look_button_) lv_obj_add_flag(camera_look_button_, LV_OBJ_FLAG_HIDDEN);
            if (camera_follow_button_) lv_obj_add_flag(camera_follow_button_, LV_OBJ_FLAG_HIDDEN);
            if (camera_status_label_) {
                lv_label_set_text(camera_status_label_, "");
                lv_obj_add_flag(camera_status_label_, LV_OBJ_FLAG_HIDDEN);
            }
            if (camera_home_label_) lv_obj_add_flag(camera_home_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_pos(camera_manual_button_, 12, 190);
            lv_obj_set_size(camera_manual_button_, 296, 44);
            lv_label_set_text(camera_manual_label_, "拍照");
            lv_obj_remove_flag(camera_manual_button_, LV_OBJ_FLAG_HIDDEN);
            avatar_.SetVisible(false);
            lv_obj_remove_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(camera_panel_);
        } else {
            lv_obj_set_style_bg_color(camera_panel_, lv_color_hex(0xEAF5F8), 0);
            lv_obj_set_style_bg_opa(camera_panel_, LV_OPA_COVER, 0);
            if (camera_title_) lv_obj_remove_flag(camera_title_, LV_OBJ_FLAG_HIDDEN);
            if (camera_look_button_) lv_obj_add_flag(camera_look_button_, LV_OBJ_FLAG_HIDDEN);
            if (camera_follow_button_) lv_obj_remove_flag(camera_follow_button_, LV_OBJ_FLAG_HIDDEN);
            if (camera_status_label_) lv_obj_remove_flag(camera_status_label_, LV_OBJ_FLAG_HIDDEN);
            if (camera_home_label_) lv_obj_remove_flag(camera_home_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_pos(camera_manual_button_, 12, 94);
            lv_obj_set_size(camera_manual_button_, 296, 44);
            lv_label_set_text(camera_manual_label_, "自己拍照");
            lv_obj_add_flag(camera_manual_button_, LV_OBJ_FLAG_HIDDEN);
            avatar_.SetVisible(true);
            UpdateCameraAppLocked();
            if (local_app_page_ == LocalAppPage::Camera) {
                lv_obj_remove_flag(camera_panel_, LV_OBJ_FLAG_HIDDEN);
                lv_obj_move_foreground(camera_panel_);
            }
        }
    }

    void ExitCameraViewfinderLocked() {
        camera_viewfinder_active_.store(false);
        if (preview_timer_) esp_timer_stop(preview_timer_);
        if (preview_image_) lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
        preview_image_cached_.reset();
        UpdateCameraViewfinderUiLocked(false);
    }

    void ConfigureSingleLineSubtitleLocked() {
        if (!bottom_bar_ || !chat_message_label_) return;

        const lv_font_t* font = lv_obj_get_style_text_font(chat_message_label_, LV_PART_MAIN);
        const int line_height = font ? lv_font_get_line_height(font) : 20;
        constexpr int horizontal_padding = 4;
        constexpr int vertical_padding = 4;

        // Keep the original compact one-line subtitle bar.  The board-wide
        // multiline option is useful on larger displays, but on this 320x240
        // avatar screen it can wrap behind adjacent layers and expose only a
        // suffix of the sentence.
        lv_obj_set_size(bottom_bar_, canvas_w_, line_height + vertical_padding * 2);
        lv_obj_set_style_pad_all(bottom_bar_, 0, 0);
        lv_obj_set_style_pad_left(bottom_bar_, horizontal_padding, 0);
        lv_obj_set_style_pad_right(bottom_bar_, horizontal_padding, 0);
        lv_obj_align(bottom_bar_, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_set_size(chat_message_label_, canvas_w_ - horizontal_padding * 2, line_height);
        lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_CENTER, 0);
        // Short captions remain still. LVGL starts SCROLL only when the text
        // exceeds this line; the animation is one forward reading pass.
        lv_anim_init(&subtitle_scroll_anim_template_);
        lv_anim_set_delay(&subtitle_scroll_anim_template_, 700);
        lv_anim_set_repeat_count(&subtitle_scroll_anim_template_, 0);
        lv_obj_set_style_anim(chat_message_label_, &subtitle_scroll_anim_template_, 0);
        lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_SCROLL);
        lv_obj_align(chat_message_label_, LV_ALIGN_CENTER, 0, 0);
    }

    void ResetSubtitleScrollLocked() {
        if (!chat_message_label_) return;

        // Reset the previous offset, then start a fresh one-shot pass only if
        // the new caption is wider than the available line.
        lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_CLIP);
        lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_SCROLL);
        lv_obj_align(chat_message_label_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_update_layout(chat_message_label_);

        const char* stored = lv_label_get_text(chat_message_label_);
        ESP_LOGI("StackChanSubtitle", "role text stored intact: '%s' (%dx%d)",
                 stored ? stored : "", lv_obj_get_width(chat_message_label_),
                 lv_obj_get_height(chat_message_label_));
    }

    void UpdateSubtitleDurationLocked(const char* content) {
        if (!chat_message_label_ || !content || content[0] == '\0') {
            subtitle_display_duration_ms_.store(4000);
            return;
        }
        const lv_font_t* font = lv_obj_get_style_text_font(chat_message_label_, LV_PART_MAIN);
        lv_point_t text_size = {};
        lv_text_get_size(&text_size, content, font,
            lv_obj_get_style_text_letter_space(chat_message_label_, LV_PART_MAIN),
            lv_obj_get_style_text_line_space(chat_message_label_, LV_PART_MAIN),
            LV_COORD_MAX, LV_TEXT_FLAG_NONE);
        const int32_t available = lv_obj_get_content_width(chat_message_label_);
        const int32_t travel = std::max<int32_t>(0, text_size.x - available);
        // Default label speed is 40 px/s, clamped by LVGL to 0.3..10 s.
        // Include the 0.7 s starting pause and leave the final characters
        // visible for 2 s before natural clearing.
        const uint32_t travel_ms = std::clamp<uint32_t>(
            static_cast<uint32_t>((travel * 1000LL + 39) / 40), 300U, 10000U);
        const uint32_t duration = travel > 0
            ? 700U + travel_ms + 2000U
            : 4000U;
        subtitle_display_duration_ms_.store(duration);
    }

    void HideEmojiBoxLocked() {
        if (emoji_box_) lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        if (emoji_label_) lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
        if (emoji_image_) lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    }

    void ApplyAvatarLayoutLocked() {
        lv_obj_set_style_bg_opa(container_, LV_OPA_TRANSP, 0);
        if (content_) lv_obj_set_style_bg_opa(content_, LV_OPA_TRANSP, 0);
        HideEmojiBoxLocked();
        if (top_bar_) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
        if (status_bar_) lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        if (speech_mode_button_) lv_obj_move_foreground(speech_mode_button_);
        if (local_app_page_ == LocalAppPage::Launcher && launcher_panel_) {
            lv_obj_move_foreground(launcher_panel_);
        } else if (local_app_page_ == LocalAppPage::Speech && speech_mode_panel_) {
            lv_obj_move_foreground(speech_mode_panel_);
        } else if (local_app_page_ == LocalAppPage::Camera && camera_panel_) {
            lv_obj_move_foreground(camera_panel_);
        } else if (local_app_page_ == LocalAppPage::Dance && dance_panel_) {
            lv_obj_move_foreground(dance_panel_);
        }
    }

    void SetActiveLocked(bool active) {
        if (active == active_mode_) return;
        active_mode_ = active;
        if (active) {
            if (top_bar_)    lv_obj_remove_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
            if (status_bar_) lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
            if (face_tracker_) face_tracker_->Pause(false);
        } else {
            if (top_bar_)    lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
            if (status_bar_) lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void BumpIdleTimerLocked() {
        if (!idle_timer_) return;
        esp_timer_stop(idle_timer_);
        esp_timer_start_once(idle_timer_, IDLE_TIMEOUT_US);
    }

    void BeginReactionLocked(uint32_t duration_ms, bool shake) {
        if (!reaction_active_) {
            reaction_prev_expression_ = avatar_.GetExpression();
            reaction_prev_overlay_ = avatar_.GetOverlay();
        }
        if (reaction_restore_timer_) esp_timer_stop(reaction_restore_timer_);
        if (reaction_anim_timer_) esp_timer_stop(reaction_anim_timer_);
        reaction_active_ = true;
        shake_reaction_ = shake;
        if (reaction_restore_timer_) {
            esp_timer_start_once(reaction_restore_timer_, (uint64_t)duration_ms * 1000);
        }
    }

    void ApplyShakeFrameLocked() {
        static constexpr shizhou_avatar::Expression frames[] = {
            shizhou_avatar::Expression::Sleepy,
            shizhou_avatar::Expression::Surprised,
            shizhou_avatar::Expression::Embarrassed,
        };
        avatar_.SetExpression(frames[reaction_step_ % 3]);
        avatar_.SetOverlay(shizhou_avatar::Overlay{});
        reaction_step_++;
    }

    void CancelReactionLocked(bool restore) {
        if (!reaction_active_) return;
        if (reaction_restore_timer_) esp_timer_stop(reaction_restore_timer_);
        if (reaction_anim_timer_) esp_timer_stop(reaction_anim_timer_);
        if (restore) {
            avatar_.SetExpression(reaction_prev_expression_);
            avatar_.SetOverlay(reaction_prev_overlay_);
        }
        reaction_active_ = false;
        shake_reaction_ = false;
    }

    static void ReactionRestoreTimerCallback(void* arg) {
        auto self = static_cast<M5StackAvatarDisplay*>(arg);
        DisplayLockGuard lock(self);
        self->CancelReactionLocked(true);
    }

    static void ReactionAnimTimerCallback(void* arg) {
        auto self = static_cast<M5StackAvatarDisplay*>(arg);
        DisplayLockGuard lock(self);
        if (self->reaction_active_ && self->shake_reaction_) {
            self->ApplyShakeFrameLocked();
        }
    }

    static void IdleTimerCallback(void* arg) {
        auto self = static_cast<M5StackAvatarDisplay*>(arg);
        DisplayLockGuard lock(self);
        self->SetActiveLocked(false);
        // face_tracker 由 SetStatus 跟设备状态联动控制，这里只管 UI 顶栏隐藏
    }

    static void InitTimerCallback(void* arg) {
        auto self = static_cast<M5StackAvatarDisplay*>(arg);
        self->TryInitAvatar();
    }

    void TryInitAvatar() {
        DisplayLockGuard lock(this);
        if (avatar_.IsReady()) return;
        lv_obj_t* screen = lv_screen_active();
        if (screen == nullptr) return;
        if (container_ == nullptr) return;

        auto collection = static_cast<LvglTheme*>(current_theme_)->emoji_collection();
        // Assets are mounted a few seconds after SetupUI.  Do not create the
        // old vector fallback while waiting; SetTheme() or the next timer tick
        // will initialize the illustrated avatar as soon as frames exist.
        if (!collection) return;
        bool ok = avatar_.Init(screen, canvas_w_, canvas_h_, collection);
        if (!ok || !avatar_.IsImageMode()) {
            avatar_.Destroy();
            return;
        }

        ApplyAvatarLayoutLocked();
        ESP_LOGI("StackChanAvatar", "Avatar initialized in %s mode",
                 avatar_.IsImageMode() ? "illustrated" : "vector fallback");

        if (avatar_init_timer_) {
            esp_timer_stop(avatar_init_timer_);
            esp_timer_delete(avatar_init_timer_);
            avatar_init_timer_ = nullptr;
        }
    }
};

class Pmic : public Axp2101 {
public:
    // Power Init
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        uint8_t data = ReadReg(0x90);
        data |= 0b10110100;
        WriteReg(0x90, data);
        WriteReg(0x99, (0b11110 - 5));
        WriteReg(0x97, (0b11110 - 2));
        WriteReg(0x69, 0b00110101);
        WriteReg(0x30, 0b111111);
        WriteReg(0x90, 0xBF);
        WriteReg(0x94, 33 - 5);
        WriteReg(0x95, 33 - 5);
    }

    void SetBrightness(uint8_t brightness) {
        brightness = ((brightness + 641) >> 5);
        WriteReg(0x99, brightness);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(Pmic *pmic) : pmic_(pmic) {}

    void SetBrightnessImpl(uint8_t brightness) override {
        pmic_->SetBrightness(target_brightness_);
        brightness_ = target_brightness_;
    }

private:
    Pmic *pmic_;
};

class Aw9523 : public I2cDevice {
public:
    // Exanpd IO Init
    Aw9523(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x02, 0b00000111);  // P0
        WriteReg(0x03, 0b10001111);  // P1
        WriteReg(0x04, 0b00011000);  // CONFIG_P0
        WriteReg(0x05, 0b00001100);  // CONFIG_P1
        WriteReg(0x11, 0b00010000);  // GCR P0 port is Push-Pull mode.
        WriteReg(0x12, 0b11111111);  // LEDMODE_P0
        WriteReg(0x13, 0b11111111);  // LEDMODE_P1
    }

    void ResetAw88298() {
        ESP_LOGI(TAG, "Reset AW88298");
        WriteReg(0x02, 0b00000011);
        vTaskDelay(pdMS_TO_TICKS(10));
        WriteReg(0x02, 0b00000111);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void ResetIli9342() {
        ESP_LOGI(TAG, "Reset IlI9342");
        WriteReg(0x03, 0b10000001);
        vTaskDelay(pdMS_TO_TICKS(20));
        WriteReg(0x03, 0b10000011);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

class Ft6336 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    Ft6336(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t chip_id = ReadReg(0xA3);
        ESP_LOGI(TAG, "Get chip ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Ft6336() {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    inline const TouchPoint_t& GetTouchPoint() {
        return tp_;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

class M5StackCoreS3Board : public WifiBoard {
private:
    enum class SpeechMode : int {
        Normal = 0,
        Quiet = 1,
        TextOnly = 2,
    };

    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_;
    Aw9523* aw9523_;
    Ft6336* ft6336_;
    LcdDisplay* display_;
    EspVideo* camera_;
    StackChanServo servo_;
    FaceTracker face_tracker_;
    esp_timer_handle_t touchpad_timer_;
    esp_timer_handle_t batt_timer_ = nullptr;
    PowerSaveTimer* power_save_timer_;
    bool py32_found_ = false;
    // ---- PY32 持久 I2C 设备句柄（控 LED + 其他扩展）----
    i2c_master_dev_handle_t py32_dev_ = nullptr;
    bool led_manual_ = false;
    std::mutex py32_mutex_;
    // ---- BMI270 (IMU) ----
    i2c_master_dev_handle_t bmi_i2c_dev_ = nullptr;
    struct bmi2_dev bmi_dev_storage_ = {};
    bmi270_handle_t bmi_handle_ = nullptr;
    TaskHandle_t motion_task_ = nullptr;
    int64_t last_motion_trigger_us_ = 0;
    // ---- 早安 cron ----
    TaskHandle_t morning_task_ = nullptr;
    int last_greeting_day_ = -1;  // 上次触发的日期（tm_yday），跨日 reset
    // ---- Persistent local alarms (independent from Operit after creation) ----
    struct LocalAlarm {
        int id = 0;
        time_t next_at = 0;
        std::string label;
        std::string repeat = "once";
        int weekday_mask = 0;  // bit 0=Sunday ... bit 6=Saturday
        bool server_music = false;
        bool enabled = true;
    };
    static constexpr size_t kMaxAlarms = 8;
    std::vector<LocalAlarm> alarms_;
    std::mutex alarms_mutex_;
    TaskHandle_t alarm_task_ = nullptr;
    int next_alarm_id_ = 1;
    std::atomic<bool> alarm_active_{false};
    // ---- SI12T 3 区触摸 ----
    i2c_master_bus_handle_t si12t_bus_ = nullptr;
    i2c_master_dev_handle_t si12t_dev_ = nullptr;
    TaskHandle_t si12t_task_ = nullptr;
    uint8_t si12t_last_state_ = 0;
    bool servo_ok_ = false;
    bool low_batt_warned_ = false;
    SpeechMode speech_mode_ = SpeechMode::Normal;
    std::atomic<bool> camera_app_busy_{false};
    std::atomic<bool> camera_viewfinder_running_{false};
    std::atomic<bool> mcp_camera_waiting_{false};
    std::atomic<TaskHandle_t> mcp_camera_task_{nullptr};
    std::atomic<bool> dance_ble_mode_{false};
    std::atomic<int64_t> dance_last_motion_us_{0};
    std::atomic<int64_t> dance_last_avatar_us_{0};
    std::atomic<int64_t> dance_last_rgb_us_{0};
    std::atomic<int> dance_last_yaw_{0};
    std::atomic<int> dance_last_pitch_{30};

    struct CameraAppRequest {
        M5StackCoreS3Board* board;
        int action;
    };

    static const char* SpeechModeName(SpeechMode mode) {
        switch (mode) {
            case SpeechMode::Quiet: return "quiet";
            case SpeechMode::TextOnly: return "text_only";
            default: return "normal";
        }
    }

    static int SpeechModeVolume(SpeechMode mode) {
        switch (mode) {
            case SpeechMode::Quiet: return 20;
            case SpeechMode::TextOnly: return 0;
            default: return 70;
        }
    }

    void LoadSpeechMode() {
        Settings settings("speech_mode", false);
        const std::string stored = settings.GetString("mode", "normal");
        if (stored == "quiet") speech_mode_ = SpeechMode::Quiet;
        else if (stored == "text_only") speech_mode_ = SpeechMode::TextOnly;
        else speech_mode_ = SpeechMode::Normal;

        // AudioCodec::Start reads this value later.  In particular, zero is a
        // valid persisted value for the text-only mode.
        Settings audio_settings("audio", true);
        audio_settings.SetInt("output_volume", SpeechModeVolume(speech_mode_));
        ESP_LOGI(TAG, "Speech mode restored: %s", SpeechModeName(speech_mode_));
    }

    void SetSpeechMode(SpeechMode mode) {
        speech_mode_ = mode;
        Settings settings("speech_mode", true);
        settings.SetString("mode", SpeechModeName(mode));
        GetAudioCodec()->SetOutputVolume(SpeechModeVolume(mode));
        if (display_) {
            static_cast<M5StackAvatarDisplay*>(display_)->SetSpeechModeUi(static_cast<int>(mode));
        }
        ESP_LOGI(TAG, "Speech mode set to %s (volume=%d)",
                 SpeechModeName(mode), SpeechModeVolume(mode));
    }

    bool SetSpeechModeByName(const std::string& name) {
        if (name == "normal") SetSpeechMode(SpeechMode::Normal);
        else if (name == "quiet") SetSpeechMode(SpeechMode::Quiet);
        else if (name == "text_only" || name == "silent") SetSpeechMode(SpeechMode::TextOnly);
        else return false;
        return true;
    }

    void RegisterSpeechModeMcpTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.speech.get_mode",
            "Get StackChan's current speaking mode: normal, quiet, or text_only.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return std::string(SpeechModeName(speech_mode_));
            });
        mcp.AddTool("self.speech.set_mode",
            "Set StackChan's persistent speaking mode. mode must be normal, quiet, or text_only. "
            "text_only keeps replies and subtitles but silences the speaker.",
            PropertyList({Property("mode", kPropertyTypeString)}),
            [this](const PropertyList& props) -> ReturnValue {
                const std::string mode = props["mode"].value<std::string>();
                if (!SetSpeechModeByName(mode)) {
                    return std::string("Invalid mode; use normal, quiet, or text_only");
                }
                return std::string(SpeechModeName(speech_mode_));
            });
    }

    void InitializeBmi270() {
        // BMI270 实际在 0x69（不是 SDK 默认的 0x68），自己用 IDF i2c API + 底层 bmi270_init 绕过硬编码
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x69,
            .scl_speed_hz = 400000,
        };
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &bmi_i2c_dev_) != ESP_OK) {
            ESP_LOGW(TAG, "BMI270: add i2c device failed");
            return;
        }

        // 自己构造 bmi2_dev：用我们的 i2c_master_dev_handle 作为 intf_ptr，read/write 走 0x69
        bmi_dev_storage_.intf = BMI2_I2C_INTF;
        bmi_dev_storage_.intf_ptr = bmi_i2c_dev_;
        bmi_dev_storage_.read = Bmi270I2cRead;
        bmi_dev_storage_.write = Bmi270I2cWrite;
        bmi_dev_storage_.delay_us = Bmi270DelayUs;
        bmi_dev_storage_.read_write_len = 256;
        bmi_dev_storage_.config_file_ptr = bmi270_config_file;
        bmi_dev_storage_.dummy_byte = 0;

        int8_t rslt = bmi270_init(&bmi_dev_storage_);
        if (rslt != BMI2_OK) {
            ESP_LOGW(TAG, "bmi270_init failed: %d", rslt);
            return;
        }
        ESP_LOGI(TAG, "BMI270 initialized (custom driver @ 0x69, chip_id=0x%02X)", bmi_dev_storage_.chip_id);

        bmi_handle_ = &bmi_dev_storage_;

        const uint8_t sens_list[] = {BMI2_ACCEL};
        rslt = bmi270_sensor_enable(sens_list, 1, bmi_handle_);
        if (rslt != BMI2_OK) {
            ESP_LOGW(TAG, "bmi270_sensor_enable failed: %d", rslt);
            bmi_handle_ = nullptr;
            return;
        }
        ESP_LOGI(TAG, "BMI270 accel enabled");

        xTaskCreatePinnedToCore(MotionTaskFunc, "motion", 4096, this, 1, &motion_task_, 1);
    }

    static void MotionTaskFunc(void* arg) {
        static_cast<M5StackCoreS3Board*>(arg)->MotionLoop();
        vTaskDelete(nullptr);
    }

    void MotionLoop() {
        // 只把明确、连续的用力摇晃发送给对话端。
        // 普通拿起、放下、转头和轻碰都不能代表用户发言。
        const float SHAKE_DELTA_THRESHOLD = 0.75f;
        const int SHAKE_PEAKS_TO_TRIGGER = 3;
        const int64_t SHAKE_WINDOW_US = 1200 * 1000;
        const int STILL_SAMPLES_TO_REARM = 12;     // 静止约 1.2 秒即可再次触发
        const int64_t GLOBAL_COOLDOWN_US = 3 * 1000 * 1000LL;

        int still_count = 0;
        bool armed = true;
        int64_t shake_peak_times[8] = {0};
        int shake_idx = 0;
        int log_counter = 0;
        float last_ax = 0, last_ay = 0, last_az = 0;
        bool last_valid = false;

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (!bmi_handle_) continue;

            struct bmi2_sens_data accel;
            int8_t rd = bmi2_get_sensor_data(&accel, bmi_handle_);
            if (rd != BMI2_OK) {
                if (++log_counter >= 10) {
                    log_counter = 0;
                    ESP_LOGW(TAG, "motion: bmi2_get_sensor_data err=%d", rd);
                }
                continue;
            }

            // BMI270 SDK 默认 ±8g 量程，int16 raw，1g ≈ 4096
            float ax = (float)accel.acc.x / 4096.0f;
            float ay = (float)accel.acc.y / 4096.0f;
            float az = (float)accel.acc.z / 4096.0f;
            // 用"轴变化率"检测动作（旋转/摇晃只改变各轴分量但不改变 mag）
            float delta = 0.0f;
            if (last_valid) {
                float dx = ax - last_ax;
                float dy = ay - last_ay;
                float dz = az - last_az;
                delta = sqrtf(dx * dx + dy * dy + dz * dz);
            }
            last_ax = ax; last_ay = ay; last_az = az; last_valid = true;

            // 只看连续采样间的轴向变化。重力方向变化或单次拿放不算摇晃。
            bool moving = delta > SHAKE_DELTA_THRESHOLD;
            int64_t now = esp_timer_get_time();
            (void)log_counter;  // 诊断 log 已禁用

            if (!moving) {
                still_count++;
                if (still_count >= STILL_SAMPLES_TO_REARM) armed = true;
                for (int i = 0; i < 8; i++) shake_peak_times[i] = 0;
                continue;
            }

            still_count = 0;
            if (!armed) continue;  // 已触发过，等静止 re-arm
            // 短冷却只用于去抖；不能让第二次摇晃在几分钟内完全失效。
            if (last_motion_trigger_us_ != 0 && (now - last_motion_trigger_us_) < GLOBAL_COOLDOWN_US) continue;

            // 摇晃检测：1 秒内累计 ≥3 个尖峰
            shake_peak_times[shake_idx % 8] = now;
            shake_idx++;
            int peak_count = 0;
            for (int i = 0; i < 8; i++) {
                if (shake_peak_times[i] > 0 &&
                    (now - shake_peak_times[i]) < SHAKE_WINDOW_US) {
                    peak_count++;
                }
            }
            if (peak_count >= SHAKE_PEAKS_TO_TRIGGER) {
                armed = false;
                last_motion_trigger_us_ = now;
                for (int i = 0; i < 8; i++) shake_peak_times[i] = 0;
                // Factory-style local reaction.  Physical interaction is not
                // a chat message and must not unexpectedly start a dialogue.
                // The official firmware uses a dizzy/shy reaction rather than
                // anger.  The illustrated theme has no dizzy decorator, so its
                // surprised frame is the closest direct image equivalent.
                static_cast<M5StackAvatarDisplay*>(display_)->OnShaken();
                if (servo_ok_) servo_.Shake();
                continue;
            }
        }
    }

    void InitializeMorningGreeting() {
        // 设置北京时间时区——SNTP 启动延后到 task 内（等 WiFi/lwip 起来）
        setenv("TZ", "CST-8", 1);
        tzset();
        xTaskCreatePinnedToCore(MorningTaskFunc, "morning", 3072, this, 1, &morning_task_, 1);
    }

    static void MorningTaskFunc(void* arg) {
        static_cast<M5StackCoreS3Board*>(arg)->MorningLoop();
        vTaskDelete(nullptr);
    }

    void MorningLoop() {
        // 工作日早 7:50（容差 7:50-7:55）触发一次"早安+天气"

        // 等 WiFi/lwip 起来再启动 SNTP（构造函数早期 init SNTP 会导致 tcpip mbox assert）
        vTaskDelay(pdMS_TO_TICKS(20000));
        if (!esp_sntp_enabled()) {
            esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "ntp.aliyun.com");
            esp_sntp_setservername(1, "ntp.tencent.com");
            esp_sntp_init();
            ESP_LOGI(TAG, "SNTP started (delayed)");
        }

        // Log the first usable China-local time so clock synchronization is
        // observable rather than silently continuing with the boot default.
        for (int attempt = 0; attempt < 30; ++attempt) {
            time_t synced_now;
            struct tm synced_local;
            time(&synced_now);
            localtime_r(&synced_now, &synced_local);
            if (synced_local.tm_year + 1900 >= 2025) {
                char local_text[32];
                strftime(local_text, sizeof(local_text), "%Y-%m-%d %H:%M:%S", &synced_local);
                ESP_LOGI(TAG, "China local time ready: %s", local_text);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(60000));  // 每分钟检查一次

            time_t now;
            struct tm tm_now;
            time(&now);
            localtime_r(&now, &tm_now);

            // NTP 还没同步成功（默认年是 1970）— 跳过
            if (tm_now.tm_year + 1900 < 2025) continue;

            int wday = tm_now.tm_wday;  // 0=周日, 1=周一, ..., 6=周六
            if (wday < 1 || wday > 5) continue;  // 只在工作日

            if (tm_now.tm_hour != 7) continue;
            if (tm_now.tm_min < 50 || tm_now.tm_min > 55) continue;

            if (last_greeting_day_ == tm_now.tm_yday) continue;  // 今天已触发过
            last_greeting_day_ = tm_now.tm_yday;

            ESP_LOGI(TAG, "Morning greeting trigger at %02d:%02d wday=%d",
                     tm_now.tm_hour, tm_now.tm_min, wday);
            SendUserMessage("（早安场景：工作日早上到了，请用温暖的语气问候用户，并用 get_weather 查询今天天气、给出穿衣建议。）");
        }
    }

    void InitializeAlarms() {
        setenv("TZ", "CST-8", 1);
        tzset();
        LoadAlarms();
        xTaskCreatePinnedToCore(AlarmTaskFunc, "alarms", 4096, this, 1, &alarm_task_, 1);
    }

    static void AlarmTaskFunc(void* arg) {
        static_cast<M5StackCoreS3Board*>(arg)->AlarmLoop();
        vTaskDelete(nullptr);
    }

    static time_t CalculateNextOccurrence(const LocalAlarm& alarm, time_t after) {
        struct tm candidate;
        localtime_r(&alarm.next_at, &candidate);
        const int target_hour = candidate.tm_hour;
        const int target_minute = candidate.tm_min;
        const int target_second = candidate.tm_sec;

        for (int days = 1; days <= 8; ++days) {
            time_t probe = after + days * 86400;
            struct tm next;
            localtime_r(&probe, &next);
            next.tm_hour = target_hour;
            next.tm_min = target_minute;
            next.tm_sec = target_second;
            next.tm_isdst = -1;

            bool matches = false;
            if (alarm.repeat == "daily") {
                matches = true;
            } else if (alarm.repeat == "weekdays") {
                matches = next.tm_wday >= 1 && next.tm_wday <= 5;
            } else if (alarm.repeat == "weekly") {
                matches = next.tm_wday == candidate.tm_wday;
            } else if (alarm.repeat == "custom") {
                matches = (alarm.weekday_mask & (1 << next.tm_wday)) != 0;
            }
            if (matches) return mktime(&next);
        }
        return 0;
    }

    void SaveAlarmsLocked() {
        cJSON* root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "next_id", next_alarm_id_);
        cJSON* items = cJSON_AddArrayToObject(root, "items");
        for (const auto& alarm : alarms_) {
            cJSON* item = cJSON_CreateObject();
            cJSON_AddNumberToObject(item, "id", alarm.id);
            cJSON_AddNumberToObject(item, "next_at", static_cast<double>(alarm.next_at));
            cJSON_AddStringToObject(item, "label", alarm.label.c_str());
            cJSON_AddStringToObject(item, "repeat", alarm.repeat.c_str());
            cJSON_AddNumberToObject(item, "weekday_mask", alarm.weekday_mask);
            cJSON_AddBoolToObject(item, "server_music", alarm.server_music);
            cJSON_AddBoolToObject(item, "enabled", alarm.enabled);
            cJSON_AddItemToArray(items, item);
        }
        char* encoded = cJSON_PrintUnformatted(root);
        if (encoded) {
            Settings settings("stack_alarm", true);
            settings.SetString("alarms", encoded);
            cJSON_free(encoded);
        }
        cJSON_Delete(root);
    }

    void LoadAlarms() {
        Settings settings("stack_alarm", false);
        const std::string encoded = settings.GetString("alarms");
        if (encoded.empty()) return;

        cJSON* root = cJSON_Parse(encoded.c_str());
        if (!root) return;
        std::lock_guard<std::mutex> lock(alarms_mutex_);
        cJSON* next_id = cJSON_GetObjectItem(root, "next_id");
        if (cJSON_IsNumber(next_id)) next_alarm_id_ = std::max(1, next_id->valueint);
        cJSON* items = cJSON_GetObjectItem(root, "items");
        cJSON* item = nullptr;
        cJSON_ArrayForEach(item, items) {
            if (alarms_.size() >= kMaxAlarms) break;
            cJSON* id = cJSON_GetObjectItem(item, "id");
            cJSON* next_at = cJSON_GetObjectItem(item, "next_at");
            cJSON* label = cJSON_GetObjectItem(item, "label");
            cJSON* repeat = cJSON_GetObjectItem(item, "repeat");
            if (!cJSON_IsNumber(id) || !cJSON_IsNumber(next_at)) continue;
            LocalAlarm alarm;
            alarm.id = id->valueint;
            alarm.next_at = static_cast<time_t>(next_at->valuedouble);
            if (cJSON_IsString(label)) alarm.label = label->valuestring;
            if (cJSON_IsString(repeat)) alarm.repeat = repeat->valuestring;
            cJSON* mask = cJSON_GetObjectItem(item, "weekday_mask");
            cJSON* server_music = cJSON_GetObjectItem(item, "server_music");
            cJSON* enabled = cJSON_GetObjectItem(item, "enabled");
            if (cJSON_IsNumber(mask)) alarm.weekday_mask = mask->valueint;
            if (cJSON_IsBool(server_music)) alarm.server_music = cJSON_IsTrue(server_music);
            if (cJSON_IsBool(enabled)) alarm.enabled = cJSON_IsTrue(enabled);
            alarms_.push_back(std::move(alarm));
        }
        cJSON_Delete(root);
    }

    void AlarmLoop() {
        // Wi-Fi/lwIP is not ready in the board constructor, so SNTP is delayed.
        vTaskDelay(pdMS_TO_TICKS(20000));
        if (!esp_sntp_enabled()) {
            esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "ntp.aliyun.com");
            esp_sntp_setservername(1, "ntp.tencent.com");
            esp_sntp_init();
            ESP_LOGI(TAG, "SNTP started for alarms");
        }

        int ring_tick = 0;
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (alarm_active_.load()) {
                // A single vibration prompt is easy to miss. Repeat a clear
                // alert sound until the physical stop gesture is used.
                if ((ring_tick++ % 2) == 0) {
                    Application::GetInstance().PlaySound(Lang::Sounds::OGG_EXCLAMATION);
                }
            } else {
                ring_tick = 0;
            }
            time_t now;
            struct tm tm_now;
            time(&now);
            localtime_r(&now, &tm_now);
            if (tm_now.tm_year + 1900 < 2025) continue;

            std::vector<LocalAlarm> firing;
            {
                std::lock_guard<std::mutex> lock(alarms_mutex_);
                bool changed = false;
                for (auto& alarm : alarms_) {
                    if (!alarm.enabled || alarm.next_at <= 0 || alarm.next_at > now) continue;
                    firing.push_back(alarm);
                    if (alarm.repeat == "once") {
                        alarm.enabled = false;
                    } else {
                        alarm.next_at = CalculateNextOccurrence(alarm, now);
                        if (alarm.next_at <= 0) alarm.enabled = false;
                    }
                    changed = true;
                }
                if (changed) SaveAlarmsLocked();
            }

            for (const auto& alarm : firing) {
                ESP_LOGI(TAG, "Alarm %d fired: %s", alarm.id, alarm.label.c_str());
                const std::string label = alarm.label.empty() ? "闹钟时间到" : alarm.label;
                const bool server_music = alarm.server_music;
                auto& app = Application::GetInstance();
                // Stop the current utterance without closing the audio channel.
                // A server-side music alarm uses this same connection to start
                // its song immediately after the local fallback begins.
                app.AbortSpeaking(kAbortReasonNone);
                app.Schedule([this, &app, label, server_music]() {
                    // Speech mode controls the assistant's voice, not alarm audibility.
                    // Temporarily use a clear alarm volume and restore the
                    // selected speech-mode volume when the alarm is stopped.
                    GetAudioCodec()->SetOutputVolume(70);
                    alarm_active_.store(true);
                    app.Alert("闹钟", label.c_str(), "surprised");
                    app.PlaySound(Lang::Sounds::OGG_EXCLAMATION);
                    if (server_music) {
                        // Music alarms are cached by the bridge, but the device
                        // normally has no WebSocket while idle. Open it now so
                        // the bridge can deliver the prepared song.
                        app.StartListening();
                    }
                    if (py32_dev_) {
                        uint16_t colors[12];
                        for (auto& color : colors) color = Rgb888To565(255, 80, 0);
                        Py32SetLedFrame(colors, 12);
                    }
                });
            }
        }
    }

    // ---- PY32 IO Expander 控 WS2812 RGB LED ×12 ----
    // 协议（来自 M5Stack/StackChan-BSP）:
    //   REG_LED_CFG  (0x24): 低6位=LED数量, bit6=1 触发刷新
    //   REG_LED_RAM  (0x30+): 颜色数据起点，每颗 LED 2 字节 RGB565 little-endian

    void InitializePy32LedDevice() {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x6F,
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &py32_dev_) != ESP_OK) {
            ESP_LOGW(TAG, "PY32 LED: add device failed");
            py32_dev_ = nullptr;
            return;
        }
        // 设 LED 数量 = 12
        uint8_t cmd[2] = {0x24, 12};
        i2c_master_transmit(py32_dev_, cmd, 2, 200);
        uint16_t off[12] = {};
        Py32SetLedFrame(off, 12);
        ESP_LOGI(TAG, "PY32 LED ready (12 LEDs, off)");
    }

    bool Py32WriteRegBlock(uint8_t reg, const uint8_t* data, size_t len) {
        if (!py32_dev_) return false;
        std::lock_guard<std::mutex> lock(py32_mutex_);
        uint8_t buf[80];
        if (len + 1 > sizeof(buf)) return false;
        buf[0] = reg;
        memcpy(buf + 1, data, len);
        return i2c_master_transmit(py32_dev_, buf, len + 1, 200) == ESP_OK;
    }

    bool Py32SetLedFrame(const uint16_t* rgb565_colors, size_t count) {
        if (!py32_dev_) return false;
        if (count > 12) count = 12;
        uint8_t data[24];
        for (size_t i = 0; i < count; i++) {
            data[i * 2]     = rgb565_colors[i] & 0xFF;
            data[i * 2 + 1] = (rgb565_colors[i] >> 8) & 0xFF;
        }
        bool ok1 = Py32WriteRegBlock(0x30, data, count * 2);
        uint8_t refresh = (uint8_t)(count | 0x40);
        bool ok2 = Py32WriteRegBlock(0x24, &refresh, 1);
        return ok1 && ok2;
    }

    static uint16_t Rgb888To565(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
    }

    void UpdateLedsFromEmotion(const char* emotion) {
        if (!py32_dev_) return;
        if (led_manual_) return;
        uint8_t r, g, b;
        if (!emotion) { r=60; g=35; b=10; }
        else if (!strcmp(emotion, "happy") || !strcmp(emotion, "laughing") || !strcmp(emotion, "funny")) { r=255; g=180; b=0; }
        else if (!strcmp(emotion, "loving") || !strcmp(emotion, "kissy")) { r=255; g=0; b=100; }
        else if (!strcmp(emotion, "sad") || !strcmp(emotion, "crying")) { r=0; g=50; b=255; }
        else if (!strcmp(emotion, "angry")) { r=255; g=0; b=0; }
        else if (!strcmp(emotion, "surprised") || !strcmp(emotion, "shocked")) { r=200; g=0; b=255; }
        else if (!strcmp(emotion, "thinking") || !strcmp(emotion, "confused")) { r=0; g=100; b=255; }
        else if (!strcmp(emotion, "winking")) { r=255; g=120; b=0; }
        else if (!strcmp(emotion, "cool")) { r=0; g=180; b=255; }
        else if (!strcmp(emotion, "relaxed")) { r=180; g=255; b=100; }
        else if (!strcmp(emotion, "delicious")) { r=255; g=80; b=0; }
        else if (!strcmp(emotion, "confident")) { r=255; g=200; b=0; }
        else if (!strcmp(emotion, "sleepy")) { r=10; g=5; b=30; }
        else if (!strcmp(emotion, "embarrassed")) { r=255; g=80; b=120; }
        else if (!strcmp(emotion, "silly")) { r=100; g=255; b=0; }
        else { r=60; g=35; b=10; }  // neutral 暖橙待机

        uint16_t color = Rgb888To565(r, g, b);
        uint16_t colors[12];
        for (int i = 0; i < 12; i++) colors[i] = color;
        Py32SetLedFrame(colors, 12);
    }

    // ---- SI12T 3 区触摸（在主 I2C 总线 0x68 上）----
    void InitializeSi12T() {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x68,
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &si12t_dev_) != ESP_OK) {
            ESP_LOGW(TAG, "SI12T: add device failed");
            si12t_dev_ = nullptr;
            return;
        }
        // 初始化序列（来自 M5Stack StackChan-BSP src/drivers/Si12T/Si12T.cpp begin()）
        Si12tWriteReg(0x0A, 0x00);  // REF_RST1
        Si12tWriteReg(0x0C, 0x00);  // CH_HOLD1
        Si12tWriteReg(0x0E, 0x00);  // CAL_HOLD1
        Si12tWriteReg(0x0B, 0x00);  // REF_RST2
        Si12tWriteReg(0x0D, 0x00);  // CH_HOLD2
        Si12tWriteReg(0x0F, 0x00);  // CAL_HOLD2
        Si12tWriteReg(0x09, 0x0F);  // CTRL2 reset
        vTaskDelay(pdMS_TO_TICKS(10));
        Si12tWriteReg(0x09, 0x07);  // CTRL2 normal
        Si12tWriteReg(0x08, 0x22);  // CTRL1
        // Match the official firmware exactly: LOW sensitivity, level 3.
        // The previous 0xCC value is HIGH sensitivity level 4 and can leave a
        // channel latched after several strokes, so the recognizer never sees
        // Release and subsequent pet gestures appear to stop working.
        for (uint8_t reg = 0x02; reg <= 0x06; reg++) {
            Si12tWriteReg(reg, 0x33);
        }
        ESP_LOGI(TAG, "SI12T 3-zone touch initialized (official LOW level 3)");

        xTaskCreatePinnedToCore(Si12tTaskFunc, "si12t", 3072, this, 1, &si12t_task_, 1);
    }

    bool Si12tWriteReg(uint8_t reg, uint8_t val) {
        if (!si12t_dev_) return false;
        uint8_t buf[2] = {reg, val};
        return i2c_master_transmit(si12t_dev_, buf, 2, 200) == ESP_OK;
    }

    uint8_t Si12tReadReg(uint8_t reg) {
        if (!si12t_dev_) return 0;
        uint8_t val = 0;
        i2c_master_transmit_receive(si12t_dev_, &reg, 1, &val, 1, 200);
        return val;
    }

    static void Si12tTaskFunc(void* arg) {
        static_cast<M5StackCoreS3Board*>(arg)->Si12tLoop();
        vTaskDelete(nullptr);
    }

    void Si12tLoop() {
        // Official HAL starts polling after 200 ms.  Waiting 12 seconds made
        // every reboot look as if head touch were intermittently unavailable.
        vTaskDelay(pdMS_TO_TICKS(200));
        si12t_last_state_ = si12t_dev_ ? Si12tReadReg(0x10) : 0;

        // Port of the official three-zone GestureRecognizer.  A mere press is
        // ignored; a real stroke must travel by more than 40 position points.
        enum class PetState { Idle, Touched, Swiping };
        PetState state = PetState::Idle;
        int initial_position = 0;
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (!si12t_dev_) continue;
            uint8_t out = Si12tReadReg(0x10);
            const int left = out & 0x03;
            const int middle = (out >> 2) & 0x03;
            const int right = (out >> 4) & 0x03;
            const int total = left + middle + right;
            const bool touched = total > 0;
            const int position = touched ? ((right - left) * 100 / total) : 0;

            if (state == PetState::Idle && touched) {
                state = PetState::Touched;
                initial_position = position;
            } else if (state == PetState::Touched && !touched) {
                state = PetState::Idle;
            } else if (state == PetState::Touched && abs(position - initial_position) > 40) {
                state = PetState::Swiping;
                ESP_LOGI(TAG, "SI12T head-pet swipe");
                static_cast<M5StackAvatarDisplay*>(display_)->OnPetted();
            } else if (state == PetState::Swiping && !touched) {
                state = PetState::Idle;
            }
            si12t_last_state_ = out;
        }
    }

    void RegisterLedMcpTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.led.set_color",
            "Set the LED ring color. Use this when the user asks to change the light color. Args: r,g,b (0-255 each).",
            PropertyList({
                Property("r", kPropertyTypeInteger, 0, 255),
                Property("g", kPropertyTypeInteger, 0, 255),
                Property("b", kPropertyTypeInteger, 0, 255),
            }),
            [this](const PropertyList& props) -> ReturnValue {
                uint8_t r = props["r"].value<int>();
                uint8_t g = props["g"].value<int>();
                uint8_t b = props["b"].value<int>();
                uint16_t color = Rgb888To565(r, g, b);
                uint16_t colors[12];
                for (int i = 0; i < 12; i++) colors[i] = color;
                Py32SetLedFrame(colors, 12);
                led_manual_ = true;
                ESP_LOGI(TAG, "MCP set LED color: r=%d g=%d b=%d", r, g, b);
                return true;
            });
        mcp.AddTool("self.led.turn_off",
            "Turn off the LED ring light.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                uint16_t off[12] = {};
                Py32SetLedFrame(off, 12);
                led_manual_ = true;
                ESP_LOGI(TAG, "MCP LED off");
                return true;
            });
        mcp.AddTool("self.led.auto",
            "Set LED back to automatic emotion-based color mode.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                led_manual_ = false;
                ESP_LOGI(TAG, "MCP LED auto mode");
                return true;
            });
    }

    void RegisterServoMcpTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.motion.nod",
            "Make StackChan nod its head to express agreement. Use once when the user asks to nod or when a short affirmative gesture is appropriate.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                servo_.Nod();
                return true;
            });
        mcp.AddTool("self.motion.shake_head",
            "Make StackChan shake its head to express disagreement. Use once when the user asks to shake its head.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                servo_.Shake();
                return true;
            });
        mcp.AddTool("self.motion.tilt_head",
            "Make StackChan tilt its head briefly, for example to express curiosity. Use once per request.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                servo_.Tilt();
                return true;
            });
        mcp.AddTool("self.motion.center",
            "Return StackChan's head to its neutral center position.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                servo_.StopDance();
                face_tracker_.Pause(false);
                servo_.Center();
                return true;
            });
        mcp.AddTool("self.motion.look_at",
            "Turn StackChan's head to a safe angle. yaw is left/right from -45 to 45 degrees; pitch is up/down from 5 to 60 degrees.",
            PropertyList({
                Property("yaw", kPropertyTypeInteger, -45, 45),
                Property("pitch", kPropertyTypeInteger, 5, 60),
            }),
            [this](const PropertyList& props) -> ReturnValue {
                servo_.StopDance();
                face_tracker_.Pause(false);
                servo_.MoveTo(props["yaw"].value<int>(), props["pitch"].value<int>(), 600);
                return true;
            });
        mcp.AddTool("self.motion.dance",
            "Make StackChan perform a continuous dance. sequence may be a preset (happy, cute, swing) or a comma-separated custom sequence using left, right, up, down, upper_left, upper_right, lower_left, lower_right, and center. tempo_ms controls each step and repeat controls the number of rounds.",
            PropertyList({
                Property("sequence", kPropertyTypeString, std::string("happy")),
                Property("tempo_ms", kPropertyTypeInteger, 360, 180, 1000),
                Property("repeat", kPropertyTypeInteger, 2, 1, 5),
            }),
            [this](const PropertyList& props) -> ReturnValue {
                const bool started = servo_.StartDance(
                    props["sequence"].value<std::string>(),
                    props["tempo_ms"].value<int>(),
                    props["repeat"].value<int>());
                if (!started) {
                    return std::string("舞步无效或小机正在执行其他动作");
                }
                static_cast<M5StackAvatarDisplay*>(display_)->SetDanceStatus("自定义舞蹈进行中");
                return std::string("小机开始跳舞了");
            });
        mcp.AddTool("self.motion.stop_dance",
            "Stop StackChan's current dance and return its head to the neutral center position.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                servo_.StopDance();
                face_tracker_.Pause(false);
                servo_.Center();
                static_cast<M5StackAvatarDisplay*>(display_)->SetDanceStatus("已停止并回正");
                return std::string("舞蹈已停止");
            });
    }

    void RegisterFaceFollowMcpTool() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.face.follow",
            "Control on-device face following. action must be on, off, or status.",
            PropertyList({Property("action", kPropertyTypeString)}),
            [this](const PropertyList& props) -> ReturnValue {
                const std::string action = props["action"].value<std::string>();
                if (action == "on") {
                    face_tracker_.SetEnabled(true);
                    Settings settings("stackchan", true);
                    settings.SetBool("face_follow", true);
                    static_cast<M5StackAvatarDisplay*>(display_)->SetFaceFollowUi(true);
                    return std::string("人脸跟随已开启");
                }
                if (action == "off") {
                    face_tracker_.SetEnabled(false);
                    Settings settings("stackchan", true);
                    settings.SetBool("face_follow", false);
                    static_cast<M5StackAvatarDisplay*>(display_)->SetFaceFollowUi(false);
                    return std::string("人脸跟随已关闭");
                }
                if (action == "status") {
                    return std::string(face_tracker_.IsEnabled() ? "人脸跟随已开启" : "人脸跟随已关闭");
                }
                return std::string("action 只能是 on、off 或 status");
            });
    }

    void RegisterAlarmMcpTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.alarm.set",
            "The single tool for every alarm or timer request. Convert both clock times and relative requests "
            "such as 'in 10 minutes' to an exact China local date and time before calling. local_datetime must "
            "be YYYY-MM-DD HH:MM:SS. label is display text only and never controls whether the alarm rings. "
            "repeat: once, daily, weekdays, weekly, or custom. custom weekday_mask uses bit0=Sunday through bit6=Saturday.",
            PropertyList({
                Property("local_datetime", kPropertyTypeString),
                Property("label", kPropertyTypeString, std::string("Alarm")),
                Property("repeat", kPropertyTypeString, std::string("once")),
                Property("weekday_mask", kPropertyTypeInteger, 0, 0, 127),
                Property("server_music", kPropertyTypeBoolean, false),
            }),
            [this](const PropertyList& props) -> ReturnValue {
                const std::string repeat = props["repeat"].value<std::string>();
                if (repeat != "once" && repeat != "daily" && repeat != "weekdays" &&
                    repeat != "weekly" && repeat != "custom") {
                    return std::string("Invalid repeat");
                }
                const int weekday_mask = props["weekday_mask"].value<int>();
                if (repeat == "custom" && weekday_mask == 0) {
                    return std::string("weekday_mask is required for custom recurrence");
                }
                int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
                const std::string text = props["local_datetime"].value<std::string>();
                if (sscanf(text.c_str(), "%d-%d-%d %d:%d:%d", &year, &month, &day,
                           &hour, &minute, &second) != 6 || year < 2024 || month < 1 || month > 12 ||
                    day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
                    second < 0 || second > 59) {
                    return std::string("Invalid local_datetime; use YYYY-MM-DD HH:MM:SS");
                }
                struct tm local_tm = {};
                local_tm.tm_year = year - 1900;
                local_tm.tm_mon = month - 1;
                local_tm.tm_mday = day;
                local_tm.tm_hour = hour;
                local_tm.tm_min = minute;
                local_tm.tm_sec = second;
                local_tm.tm_isdst = -1;
                const time_t requested = mktime(&local_tm);
                time_t now;
                time(&now);
                if (requested <= now) return std::string("Alarm time must be in the future");

                std::lock_guard<std::mutex> lock(alarms_mutex_);
                size_t active_count = 0;
                for (const auto& alarm : alarms_) if (alarm.enabled) ++active_count;
                if (active_count >= kMaxAlarms) return std::string("Alarm limit reached (8)");
                alarms_.erase(std::remove_if(alarms_.begin(), alarms_.end(),
                    [](const LocalAlarm& alarm) { return !alarm.enabled; }), alarms_.end());
                LocalAlarm alarm;
                alarm.id = next_alarm_id_++;
                alarm.next_at = requested;
                alarm.label = props["label"].value<std::string>();
                alarm.repeat = repeat;
                alarm.weekday_mask = weekday_mask;
                alarm.server_music = props["server_music"].value<bool>();
                alarms_.push_back(alarm);
                SaveAlarmsLocked();
                return std::string("Alarm created: id=") + std::to_string(alarm.id) +
                    ", China local time=" + text + ", label=" + alarm.label;
            });

        mcp.AddTool("self.alarm.list", "List all active alarms and timers.", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                std::lock_guard<std::mutex> lock(alarms_mutex_);
                cJSON* root = cJSON_CreateArray();
                for (const auto& alarm : alarms_) {
                    if (!alarm.enabled) continue;
                    cJSON* item = cJSON_CreateObject();
                    cJSON_AddNumberToObject(item, "id", alarm.id);
                    cJSON_AddNumberToObject(item, "next_at", static_cast<double>(alarm.next_at));
                    struct tm local_tm;
                    localtime_r(&alarm.next_at, &local_tm);
                    char local_datetime[20];
                    strftime(local_datetime, sizeof(local_datetime), "%Y-%m-%d %H:%M:%S", &local_tm);
                    cJSON_AddStringToObject(item, "china_local_datetime", local_datetime);
                    cJSON_AddStringToObject(item, "label", alarm.label.c_str());
                    cJSON_AddStringToObject(item, "repeat", alarm.repeat.c_str());
                    cJSON_AddNumberToObject(item, "weekday_mask", alarm.weekday_mask);
                    cJSON_AddItemToArray(root, item);
                }
                char* encoded = cJSON_PrintUnformatted(root);
                std::string result = encoded ? encoded : "[]";
                if (encoded) cJSON_free(encoded);
                cJSON_Delete(root);
                return result;
            });

        mcp.AddTool("self.alarm.cancel", "Cancel an alarm by id.",
            PropertyList({Property("id", kPropertyTypeInteger, 1, 2147483647)}),
            [this](const PropertyList& props) -> ReturnValue {
                const int id = props["id"].value<int>();
                std::lock_guard<std::mutex> lock(alarms_mutex_);
                auto it = std::find_if(alarms_.begin(), alarms_.end(),
                    [id](const LocalAlarm& alarm) { return alarm.id == id && alarm.enabled; });
                if (it == alarms_.end()) return std::string("Alarm not found");
                it->enabled = false;
                SaveAlarmsLocked();
                return true;
            });

        mcp.AddTool("self.alarm.stop", "Stop the currently ringing alarm.", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                alarm_active_.store(false);
                auto& app = Application::GetInstance();
                // This hidden tool acknowledges the device-side fallback when
                // the server is about to play a music alarm.  Do not close or
                // reset that stream here; physical long-press remains the full
                // stop action for both alarms and music.
                GetAudioCodec()->SetOutputVolume(SpeechModeVolume(speech_mode_));
                app.DismissAlert();
                return true;
            });

    }

    void RegisterDisplayMcpTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.screen.set_emotion",
            "Show a facial expression on StackChan. emotion: neutral, happy, angry, sad, sleepy, loving, "
            "crying, kissy, cool, confident, shocked, thinking, surprised, confused, embarrassed, silly, "
            "winking, laughing, funny, relaxed, or delicious.",
            PropertyList({Property("emotion", kPropertyTypeString)}),
            [this](const PropertyList& props) -> ReturnValue {
                const std::string emotion = props["emotion"].value<std::string>();
                static const char* allowed[] = {
                    "neutral", "happy", "angry", "sad", "sleepy", "loving", "crying", "kissy",
                    "cool", "confident", "shocked", "thinking", "surprised", "confused",
                    "embarrassed", "silly", "winking", "laughing", "funny", "relaxed", "delicious"
                };
                bool valid = false;
                for (const char* item : allowed) {
                    if (emotion == item) { valid = true; break; }
                }
                if (!valid) return std::string("Unsupported emotion");
                display_->SetEmotion(emotion.c_str());
                return true;
            });
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, -1);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(10);
            servo_.PauseScan();
            if (py32_dev_) {
                uint16_t off[12] = {};
                Py32SetLedFrame(off, 12);
            }
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
            servo_.ResumeScan();
            if (py32_dev_) {
                uint16_t neutral = Rgb888To565(60, 35, 10);
                uint16_t colors[12];
                for (int i = 0; i < 12; i++) colors[i] = neutral;
                Py32SetLedFrame(colors, 12);
            }
        });
        power_save_timer_->OnShutdownRequest([this]() {
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeAw9523() {
        ESP_LOGI(TAG, "Init AW9523");
        aw9523_ = new Aw9523(i2c_bus_, 0x58);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // 物理交互只有两个经过明确批准的对话动作：顶部摸头、用力连续摇晃。
    static void SendUserMessage(const char* msg) {
        if (!msg) return;
        // SendUserText 内部处理状态分流：Idle 走 WakeWord 建 channel，对话中走 SendWakeWordDetected 不打断
        Application::GetInstance().SendUserText(msg);
    }

    static std::string CameraResponseText(const std::string& result) {
        std::string key = "\"response\":\"";
        size_t start = result.find(key);
        if (start == std::string::npos) {
            key = "\"message\":\"";
            start = result.find(key);
        }
        if (start == std::string::npos) return {};
        size_t pos = start + key.size();
        std::string text;
        while (pos < result.size() && text.size() < 72) {
            char ch = result[pos++];
            if (ch == '"') break;
            if (ch == '\\' && pos < result.size()) {
                char escaped = result[pos++];
                if (escaped == 'n' || escaped == 'r') ch = ' ';
                else ch = escaped;
            }
            text.push_back(ch);
        }
        return text;
    }

    static void CameraViewfinderTaskFunc(void* arg) {
        auto* self = static_cast<M5StackCoreS3Board*>(arg);
        auto* ui = static_cast<M5StackAvatarDisplay*>(self->display_);
        Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        ESP_LOGI(TAG, "Camera viewfinder started");
        while (ui->IsCameraViewfinderActive()) {
            if (!self->camera_ || !self->camera_->IsOk()) {
                ui->SetCameraViewfinderActive(false);
                ui->SetCameraAppStatus("相机不可用");
                break;
            }
            if (!self->camera_->Capture()) {
                ui->SetCameraViewfinderActive(false);
                ui->SetCameraAppStatus("取景失败，请重试");
                break;
            }
            // About 4 fps is responsive enough for framing while leaving CPU
            // and PSRAM bandwidth for touch, audio and display refresh.
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        self->camera_viewfinder_running_.store(false);
        if (!self->camera_app_busy_.load()) {
            self->face_tracker_.Resume();
            Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        }
        ESP_LOGI(TAG, "Camera viewfinder stopped");
        vTaskDeleteWithCaps(nullptr);
    }

    static void CameraAppTaskFunc(void* arg) {
        auto* request = static_cast<CameraAppRequest*>(arg);
        auto* self = request->board;
        const int action = request->action;
        delete request;

        auto* ui = static_cast<M5StackAvatarDisplay*>(self->display_);
        Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        // Give StopAudioSession() and the face-tracker loop time to release
        // their network/camera buffers before taking exclusive ownership.
        vTaskDelay(pdMS_TO_TICKS(700));
        ESP_LOGI(TAG,
                 "Camera app task started: action=%d, free_internal=%u, largest_internal=%u",
                 action,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        try {
            auto& wifi = WifiManager::GetInstance();
            if (!wifi.IsConnected()) {
                ui->SetCameraAppStatus("正在连接网络…");
                ESP_LOGW(TAG, "Camera upload waiting for WiFi reconnection");
                for (int waited_ms = 0; waited_ms < 18000 && !wifi.IsConnected();
                     waited_ms += 250) {
                    vTaskDelay(pdMS_TO_TICKS(250));
                }
            }
            if (!wifi.IsConnected()) {
                ui->SetCameraAppStatus("网络未连接，请稍后重试");
            } else if (!self->camera_ || !self->camera_->IsOk()) {
                ui->SetCameraAppStatus("相机不可用");
            } else if (!self->camera_->Capture()) {
                ui->SetCameraAppStatus("拍照失败，请重试");
            } else {
                const std::string result = self->camera_->Explain(
                    action == 11 ? "__stackchan_manual_photo__"
                                 : "__stackchan_camera_look__");
                const std::string response = CameraResponseText(result);
                if (!response.empty()) {
                    ui->SetCameraAppStatus(response.c_str());
                } else {
                    ui->SetCameraAppStatus("助手看完了");
                }
            }
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "Camera app action %d failed: %s", action, e.what());
            ui->SetCameraAppStatus("操作失败，请检查网络后重试");
        }
        self->face_tracker_.Resume();
        Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        self->camera_app_busy_.store(false);
        // This worker's stack is allocated from PSRAM by
        // xTaskCreatePinnedToCoreWithCaps(), so it must use the matching delete.
        vTaskDeleteWithCaps(nullptr);
    }

    void HandleCameraAppAction(int action) {
        auto* ui = static_cast<M5StackAvatarDisplay*>(display_);
        if (action == 13) {
            const bool enabled = !face_tracker_.IsEnabled();
            face_tracker_.SetEnabled(enabled);
            Settings settings("stackchan", true);
            settings.SetBool("face_follow", enabled);
            ui->SetFaceFollowUi(enabled);
            ui->SetCameraAppStatus(enabled ? "人脸追踪已开启" : "人脸追踪已关闭");
            return;
        }
        if (action == 12) {
            if (camera_viewfinder_running_.exchange(true)) return;
            ui->SetCameraAppStatus("");
            ui->SetCameraViewfinderActive(true);
            face_tracker_.Pause(false);
            const BaseType_t preview_result = xTaskCreatePinnedToCoreWithCaps(
                CameraViewfinderTaskFunc, "camera_viewfinder", 8 * 1024,
                this, 1, nullptr, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (preview_result != pdPASS) {
                camera_viewfinder_running_.store(false);
                ui->SetCameraViewfinderActive(false);
                face_tracker_.Resume();
                ui->SetCameraAppStatus("无法启动取景器");
            }
            return;
        }
        if (action != 10 && action != 11) return;
        if (action == 11 && !ui->IsCameraViewfinderActive()) return;
        // When the AI initiated the camera MCP call, the on-screen shutter
        // completes that pending tool call instead of starting a second local
        // upload. The MCP response will then carry the real JPEG to Operit.
        if (action == 11 && mcp_camera_waiting_.load()) {
            ui->SetCameraAppStatus("正在拍照…");
            TaskHandle_t waiting_task = mcp_camera_task_.load();
            if (waiting_task) xTaskNotifyGive(waiting_task);
            return;
        }
        if (camera_app_busy_.exchange(true)) {
            ui->SetCameraAppStatus("相机正在处理中…");
            return;
        }
        if (action == 11) {
            ui->SetCameraViewfinderActive(false);
            for (int i = 0; i < 50 && camera_viewfinder_running_.load(); ++i) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            // An in-flight preview frame may have arrived just after the first
            // clear. Clear once more before taking the user-triggered frame.
            ui->SetCameraViewfinderActive(false);
            ui->SetCameraAppStatus("正在拍照…");
        } else {
            ui->SetCameraAppStatus("助手正在看…");
        }
        // Local camera actions do not need an open conversational audio
        // channel. Release its scarce internal buffers and prevent the
        // background tracker from reading the same camera concurrently.
        face_tracker_.Pause(false);
        Application::GetInstance().StopAudioSession();
        auto* request = new CameraAppRequest{this, action};
        // Camera capture + HTTPS upload needs a comparatively large contiguous
        // stack. At idle the CoreS3 may have enough total internal RAM but no
        // contiguous 8 KB block, making ordinary xTaskCreate fail. Put this
        // short-lived worker stack in PSRAM and keep only its TCB internally.
        const BaseType_t result = xTaskCreatePinnedToCoreWithCaps(
            CameraAppTaskFunc, "camera_app", 12 * 1024, request, 1, nullptr, 1,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (result != pdPASS) {
            delete request;
            face_tracker_.Resume();
            camera_app_busy_.store(false);
            ui->SetCameraAppStatus("无法启动相机任务");
            ESP_LOGE(TAG,
                     "Failed to create camera app task: %ld (internal largest=%u, psram largest=%u)",
                     (long)result,
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        }
    }

    static int DanceJsonItemInt(cJSON* parent, const char* object_name,
                                const char* item_name, int fallback) {
        cJSON* object = cJSON_GetObjectItem(parent, object_name);
        cJSON* item = object ? cJSON_GetObjectItem(object, item_name) : nullptr;
        return cJSON_IsNumber(item) ? item->valueint : fallback;
    }

    void HandleOfficialDanceMotion(const std::string& json) {
        const int64_t now = esp_timer_get_time();
        const int64_t previous = dance_last_motion_us_.load();
        // The phone can send motion frames much faster than the two physical
        // servos can settle. Flooding the bus also creates large current
        // spikes, which may brown out the CoreS3. Keep a smooth 12.5 Hz cap.
        if (previous != 0 && now - previous < 80000) return;
        dance_last_motion_us_.store(now);
        cJSON* root = cJSON_ParseWithLength(json.data(), json.size());
        if (!root) return;
        const int raw_yaw = DanceJsonItemInt(root, "yawServo", "angle", 0);
        const int raw_pitch = DanceJsonItemInt(root, "pitchServo", "angle", 0);
        cJSON_Delete(root);

        // The official app uses tenths-of-a-degree-like logical units with a
        // much wider factory range. Map them into this unit's proven safe
        // limits instead of forwarding raw servo positions.
        const int yaw = std::clamp(raw_yaw * 45 / 1280, -45, 45);
        const int pitch = std::clamp(30 + raw_pitch * 30 / 900, 5, 60);
        const int old_yaw = dance_last_yaw_.exchange(yaw);
        const int old_pitch = dance_last_pitch_.exchange(pitch);
        if (abs(yaw - old_yaw) < 2 && abs(pitch - old_pitch) < 2) return;
        servo_.MoveTo(yaw, pitch, 180);
    }

    void HandleOfficialDanceAvatar(const std::string& json) {
        const int64_t now = esp_timer_get_time();
        const int64_t previous = dance_last_avatar_us_.load();
        if (previous != 0 && now - previous < 100000) return;
        dance_last_avatar_us_.store(now);
        cJSON* root = cJSON_ParseWithLength(json.data(), json.size());
        if (!root) return;
        const int left_eye = DanceJsonItemInt(root, "leftEye", "weight", 100);
        const int right_eye = DanceJsonItemInt(root, "rightEye", "weight", 100);
        const int mouth = DanceJsonItemInt(root, "mouth", "weight", 0);
        cJSON_Delete(root);
        static_cast<M5StackAvatarDisplay*>(display_)->ApplyDanceAvatar(
            (left_eye + right_eye) / 2, mouth);
    }

    static uint16_t DanceHexToRgb565(const char* text) {
        if (!text || text[0] != '#' || strlen(text) < 7) return 0;
        char* end = nullptr;
        const unsigned long rgb = strtoul(text + 1, &end, 16);
        if (!end || end < text + 7) return 0;
        uint8_t red = (rgb >> 16) & 0xff;
        uint8_t green = (rgb >> 8) & 0xff;
        uint8_t blue = rgb & 0xff;
        // Twelve full-white LEDs plus two moving servos can exceed the small
        // unit's instantaneous power budget. Preserve hue while capping the
        // dance light peak to a safe level.
        const uint8_t peak = std::max(red, std::max(green, blue));
        constexpr uint8_t kSafePeak = 96;
        if (peak > kSafePeak) {
            red = static_cast<uint8_t>((static_cast<uint16_t>(red) * kSafePeak) / peak);
            green = static_cast<uint8_t>((static_cast<uint16_t>(green) * kSafePeak) / peak);
            blue = static_cast<uint8_t>((static_cast<uint16_t>(blue) * kSafePeak) / peak);
        }
        return ((red & 0xf8) << 8) | ((green & 0xfc) << 3) | (blue >> 3);
    }

    void HandleOfficialDanceRgb(const std::string& json) {
        const int64_t now = esp_timer_get_time();
        const int64_t previous = dance_last_rgb_us_.load();
        if (previous != 0 && now - previous < 120000) return;
        dance_last_rgb_us_.store(now);
        cJSON* root = cJSON_ParseWithLength(json.data(), json.size());
        if (!root) return;
        cJSON* left = cJSON_GetObjectItem(root, "leftRgbColor");
        cJSON* right = cJSON_GetObjectItem(root, "rightRgbColor");
        const uint16_t left_color = DanceHexToRgb565(
            cJSON_IsString(left) ? left->valuestring : "#000000");
        const uint16_t right_color = DanceHexToRgb565(
            cJSON_IsString(right) ? right->valuestring : "#000000");
        cJSON_Delete(root);
        uint16_t colors[12];
        for (int i = 0; i < 6; ++i) colors[i] = left_color;
        for (int i = 6; i < 12; ++i) colors[i] = right_color;
        led_manual_ = true;
        Py32SetLedFrame(colors, 12);
    }

    void EnterOfficialDanceMode() {
        if (dance_ble_mode_.exchange(true)) return;
        auto* ui = static_cast<M5StackAvatarDisplay*>(display_);
        ui->SetDanceStatus("正在开启蓝牙，请稍候…");
        // Page navigation must not alter the network/audio session or rebuild
        // the wake-word engine.  Music and speech continue independently while
        // the official dance BLE service is active.
        face_tracker_.Pause(false);
        servo_.StopDance();
        dance_last_motion_us_.store(0);
        dance_last_avatar_us_.store(0);
        dance_last_rgb_us_.store(0);
        dance_last_yaw_.store(0);
        dance_last_pitch_.store(30);
        vTaskDelay(pdMS_TO_TICKS(350));
        ESP_LOGI(TAG, "Starting dance BLE: free_internal=%u, largest_internal=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        const bool started = StackChanBleCompat::Instance().Start(
            [this](const std::string& data) { HandleOfficialDanceMotion(data); },
            [this](const std::string& data) { HandleOfficialDanceAvatar(data); },
            [this](const std::string& data) { HandleOfficialDanceRgb(data); },
            [this](StackChanBleCompat::Status status) {
                const char* text = "等待原厂 App 连接 StackChan";
                if (status == StackChanBleCompat::Status::Connected) {
                    text = "原厂 App 已连接，可以开始录制";
                } else if (status == StackChanBleCompat::Status::Error) {
                    text = "蓝牙广播失败，请退出后重试";
                }
                static_cast<M5StackAvatarDisplay*>(display_)->SetDanceStatus(text);
            });
        if (!started) {
            dance_ble_mode_.store(false);
            face_tracker_.Resume();
            ui->SetDanceStatus("蓝牙启动失败，请重新进入此页");
        }
    }

    void ExitOfficialDanceMode() {
        if (!dance_ble_mode_.exchange(false)) return;
        StackChanBleCompat::Instance().Stop();
        led_manual_ = false;
        servo_.Center();
        face_tracker_.Resume();
    }

    void SyncOfficialDanceMode() {
        const bool should_run =
            static_cast<M5StackAvatarDisplay*>(display_)->IsDancePage();
        if (should_run) EnterOfficialDanceMode();
        else if (!StackChanBleCompat::Instance().IsConnected()) ExitOfficialDanceMode();
    }

    void PollTouchpad() {
        // Keep BLE while the phone is connected, even on the avatar page. If
        // it disconnects there, release BLE and restore wake-word resources.
        if (dance_ble_mode_.load() &&
            !static_cast<M5StackAvatarDisplay*>(display_)->IsDancePage() &&
            !StackChanBleCompat::Instance().IsConnected()) {
            ExitOfficialDanceMode();
        }
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        static int touch_start_x = 0, touch_start_y = 0;
        static int touch_last_x = 0, touch_last_y = 0;
        static int touch_total_move = 0;
        static bool pet_triggered = false;
        static bool alarm_dismissed = false;
        static bool pending_single_release = false;
        static int64_t pending_single_release_time = 0;

        const int64_t SHORT_TOUCH_MS = 500;
        const int64_t PET_TOUCH_MS = 1500;
        const int64_t DOUBLE_CLICK_MS = 500;       // 双击窗口放宽
        const int SWIPE_THRESHOLD_PX = 20;         // 滑动门槛降低
        const int PET_MOVE_THRESHOLD_PX = 3;
        const int CLICK_MAX_MOVE_PX = 5;           // 短按/双击允许的最大位移：超过就不算短按了

        ft6336_->UpdateTouchPoint();
        auto& touch_point = ft6336_->GetTouchPoint();
        int64_t now = esp_timer_get_time() / 1000;

        // 待定单击超过双击窗口 → 执行单击（ToggleChat）
        if (pending_single_release && (now - pending_single_release_time) > DOUBLE_CLICK_MS) {
            pending_single_release = false;
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            // Do not bind an ordinary screen tap to the network conversation
            // state.  FT6336 can occasionally report a short residual/ghost
            // touch; the old ToggleChatState() call then opened a second audio
            // channel immediately after a voice session closed, leaving the UI
            // apparently stuck at "Connecting".  Voice wake-up remains the
            // only normal way to start a conversation.
            ESP_LOGI(TAG, "Ignoring single tap for chat-state control");
            return;
        }

        if (touch_point.num > 0 && !was_touched) {
            // 按下
            was_touched = true;
            pet_triggered = false;
            alarm_dismissed = false;
            touch_start_time = now;
            touch_start_x = touch_point.x;
            touch_start_y = touch_point.y;
            touch_last_x = touch_point.x;
            touch_last_y = touch_point.y;
            touch_total_move = 0;
        }
        else if (touch_point.num > 0 && was_touched) {
            // 按住中 — 累积移动距离
            touch_total_move += abs(touch_point.x - touch_last_x) + abs(touch_point.y - touch_last_y);
            touch_last_x = touch_point.x;
            touch_last_y = touch_point.y;

            // A server-side music alarm arrives as ordinary streamed speech,
            // so it cannot set alarm_active_. Treat a deliberate two-second
            // hold during either a local alarm or active playback as STOP.
            // This also provides a useful physical stop for ordinary music,
            // while idle petting keeps its existing behaviour.
            auto& app = Application::GetInstance();
            bool stoppable_audio = app.GetDeviceState() == kDeviceStateSpeaking;
            if ((alarm_active_.load() || stoppable_audio) && !alarm_dismissed &&
                (now - touch_start_time) >= 2000) {
                alarm_dismissed = true;
                pet_triggered = true;
                alarm_active_.store(false);
                app.StopAudioSession();
                GetAudioCodec()->SetOutputVolume(SpeechModeVolume(speech_mode_));
                app.DismissAlert();
                ESP_LOGI(TAG, "Alarm or streamed audio stopped by two-second screen hold");
                return;
            }

            // 长按摸头（要手指有移动，不算被物体压）
            if (!pet_triggered) {
                int64_t held = now - touch_start_time;
                if (held >= PET_TOUCH_MS && touch_total_move >= PET_MOVE_THRESHOLD_PX) {
                    pet_triggered = true;
                    static_cast<M5StackAvatarDisplay*>(display_)->OnPetted();
                    // 屏幕长按只改变本机表情，不代表用户向助手发消息。
                }
            }
        }
        else if (touch_point.num == 0 && was_touched) {
            // 抬起
            was_touched = false;
            int64_t touch_duration = now - touch_start_time;
            int dx_total = touch_last_x - touch_start_x;
            int dy_total = touch_last_y - touch_start_y;
            int abs_dx = abs(dx_total);
            int abs_dy = abs(dy_total);

            if (pet_triggered) return;  // 摸头已触发就不再判别

            auto* avatar_display = static_cast<M5StackAvatarDisplay*>(display_);

            // Horizontal swipes navigate the full-screen local pages directly:
            // avatar -> sound -> camera.  Handle this before tap detection.
            if (touch_duration < 800 && abs_dx >= SWIPE_THRESHOLD_PX && abs_dx > abs_dy &&
                avatar_display->HandleLocalAppSwipe(dx_total, dy_total)) {
                pending_single_release = false;
                SyncOfficialDanceMode();
                return;
            }

            // App buttons tolerate normal finger jitter; avatar tap/double-tap
            // handling below keeps its conservative threshold.
            constexpr int APP_TAP_MAX_MOVE_PX = 22;
            if (touch_duration < 800 && (abs_dx + abs_dy) <= APP_TAP_MAX_MOVE_PX) {
                int app_action = avatar_display->HandleLocalAppTap(touch_start_x, touch_start_y);
                if (app_action != -1) {
                    pending_single_release = false;
                    SyncOfficialDanceMode();
                    if (app_action >= 0 && app_action <= 2) {
                        SetSpeechMode(static_cast<SpeechMode>(app_action));
                    } else if (app_action >= 10) {
                        HandleCameraAppAction(app_action);
                    }
                    return;
                }
            }

            // 滑动手势：短促 + 位移够大
            if (touch_duration < SHORT_TOUCH_MS && (abs_dx >= SWIPE_THRESHOLD_PX || abs_dy >= SWIPE_THRESHOLD_PX)) {
                // 屏幕滑动只作为本机手势，不向 Operit 发送虚构动作。
                return;
            }

            // 短按（位移要几乎为零，否则视为"模糊手势"不触发任何切换）
            int total_move = abs_dx + abs_dy;
            if (touch_duration < SHORT_TOUCH_MS && total_move <= CLICK_MAX_MOVE_PX) {
                if (pending_single_release && (now - pending_single_release_time) <= DOUBLE_CLICK_MS) {
                    // 第二次短按落在窗口内 → 双击
                    pending_single_release = false;
                    // 双击不向 Operit 发送虚构动作。
                } else {
                    // 候选单击，等下一帧或下次按下判定
                    pending_single_release = true;
                    pending_single_release_time = now;
                }
            }
            // 中间地带（5px < 位移 < 20px，或时长过长）—— 什么都不做，避免误切对话
        }
    }

    void InitializeFt6336TouchPad() {
        ESP_LOGI(TAG, "Init FT6336");
        ft6336_ = new Ft6336(i2c_bus_, 0x38);

        // 创建定时器，20ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                M5StackCoreS3Board* board = (M5StackCoreS3Board*)arg;
                board->PollTouchpad();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };

        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_37;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_36;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeIli9342Display() {
        ESP_LOGI(TAG, "Init IlI9342");

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_3;
        io_config.dc_gpio_num = GPIO_NUM_35;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        aw9523_->ResetIli9342();

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new M5StackAvatarDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

     void InitializeCamera() {
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAMERA_PIN_D0,
                [1] = CAMERA_PIN_D1,
                [2] = CAMERA_PIN_D2,
                [3] = CAMERA_PIN_D3,
                [4] = CAMERA_PIN_D4,
                [5] = CAMERA_PIN_D5,
                [6] = CAMERA_PIN_D6,
                [7] = CAMERA_PIN_D7,
            },
            .vsync_io = CAMERA_PIN_VSYNC,
            .de_io = CAMERA_PIN_HREF,
            .pclk_io = CAMERA_PIN_PCLK,
            .xclk_io = CAMERA_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c_bus_,
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAMERA_PIN_RESET,
            .pwdn_pin = CAMERA_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = XCLK_FREQ_HZ,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new EspVideo(video_config);
        camera_->SetHMirror(true);
        // Local camera pages are independent apps and must not depend on an
        // active voice/MCP session to receive the temporary vision endpoint.
        // The VPS accepts tokenless reserved camera commands only from this
        // device/client pair; ordinary vision requests still require a token.
        camera_->SetExplainUrl(
            "", "");
    }

public:
    M5StackCoreS3Board() {
        ESP_LOGW(TAG, "Boot reset reason: %d (power-on=1, software=3, panic=4, task-WDT=6, brownout=9)",
                 static_cast<int>(esp_reset_reason()));
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeAxp2101();
        InitializeAw9523();
        I2cDetect();

        py32_found_ = EnableServoPowerViaPy32(i2c_bus_);
        if (py32_found_) {
            vTaskDelay(pdMS_TO_TICKS(200));
            servo_ok_ = servo_.Begin();
            InitializePy32LedDevice();
            RegisterLedMcpTools();
            if (servo_ok_) {
                RegisterServoMcpTools();
            }
        }
        RegisterAlarmMcpTools();

        InitializeSpi();
        InitializeIli9342Display();
        LoadSpeechMode();
        static_cast<M5StackAvatarDisplay*>(display_)->SetSpeechModeUi(static_cast<int>(speech_mode_));
        RegisterSpeechModeMcpTools();
        RegisterDisplayMcpTools();
        InitializeCamera();
        auto* avatar_display = static_cast<M5StackAvatarDisplay*>(display_);
        if (servo_ok_) {
            avatar_display->SetServo(&servo_);
        }
        if (camera_ && camera_->IsOk() && servo_ok_) {
            face_tracker_.Start(camera_, &servo_);
            servo_.SetFaceTracker(&face_tracker_);
            avatar_display->SetFaceTracker(&face_tracker_);
            Settings settings("stackchan", false);
            face_tracker_.SetEnabled(settings.GetBool("face_follow", true));
            avatar_display->SetFaceFollowUi(face_tracker_.IsEnabled());
            RegisterFaceFollowMcpTool();
        }
        avatar_display->SetLedUpdater([this](const char* emotion) {
            UpdateLedsFromEmotion(emotion);
        });
        InitializeFt6336TouchPad();
        InitializeBmi270();
        InitializeSi12T();
        InitializeMorningGreeting();
        InitializeAlarms();

        esp_timer_create_args_t status_args = {};
        status_args.callback = [](void* arg) {
            auto* self = static_cast<M5StackCoreS3Board*>(arg);
            ESP_LOGW(TAG, "=== INIT STATUS ===");
            ESP_LOGW(TAG, "PY32 (0x6F): %s | Servo bus: %s",
                     self->py32_found_ ? "OK" : "NOT FOUND",
                     self->servo_ok_ ? "OK" : "FAILED");
            ESP_LOGW(TAG, "Camera: %s",
                     self->camera_ && self->camera_->IsOk() ? "OK" : "FAILED");
            ESP_LOGW(TAG, "===================");
        };
        status_args.arg = this;
        status_args.name = "servo_status";
        esp_timer_handle_t status_timer;
        esp_timer_create(&status_args, &status_timer);
        esp_timer_start_once(status_timer, 5000000);

        esp_timer_create_args_t batt_args = {};
        batt_args.callback = [](void* arg) {
            auto* self = static_cast<M5StackCoreS3Board*>(arg);
            int level = 0; bool charging = false, discharging = false;
            self->GetBatteryLevel(level, charging, discharging);
            if (level > 0 && level <= 15 && !charging && !self->low_batt_warned_) {
                self->low_batt_warned_ = true;
                auto* disp = static_cast<M5StackAvatarDisplay*>(self->display_);
                disp->SetEmotion("sad");
                auto& app = Application::GetInstance();
                app.Schedule([&app]() {
                    app.Alert("Warning", "我快没电了，快给我充电嘛……");
                });
                ESP_LOGW(TAG, "Low battery alert: %d%%", level);
            } else if (level > 20 || charging) {
                self->low_batt_warned_ = false;
            }
        };
        batt_args.arg = this;
        batt_args.name = "batt_check";
        batt_args.dispatch_method = ESP_TIMER_TASK;
        batt_args.skip_unhandled_events = true;
        esp_timer_create(&batt_args, &batt_timer_);
        esp_timer_start_periodic(batt_timer_, 60000000);

        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CoreS3AudioCodec audio_codec(i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_AW88298_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool WaitForCameraShutter() override {
        if (camera_app_busy_.exchange(true)) {
            ESP_LOGW(TAG, "AI camera request rejected: camera is busy");
            return false;
        }
        if (mcp_camera_waiting_.exchange(true)) {
            camera_app_busy_.store(false);
            return false;
        }

        auto* ui = static_cast<M5StackAvatarDisplay*>(display_);
        TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
        mcp_camera_task_.store(current_task);
        ulTaskNotifyTake(pdTRUE, 0);
        Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        face_tracker_.Pause(false);

        // Reuse the same live preview and physical shutter as the local camera
        // app, but keep this MCP request pending until the user confirms.
        HandleCameraAppAction(12);
        ui->SetCameraAppStatus("准备好后按下方拍照");
        ESP_LOGI(TAG, "AI camera request waiting for physical shutter");

        bool preview_started = camera_viewfinder_running_.load() &&
                               ui->IsCameraViewfinderActive();
        bool shutter_pressed = preview_started &&
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(18000)) > 0;

        mcp_camera_waiting_.store(false);
        mcp_camera_task_.store(nullptr);
        ui->SetCameraViewfinderActive(false);
        for (int i = 0; i < 50 && camera_viewfinder_running_.load(); ++i) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        ui->SetCameraViewfinderActive(false);

        if (!preview_started) {
            ESP_LOGW(TAG, "AI camera request cancelled: viewfinder failed");
            ui->SetCameraAppStatus("无法启动取景器");
            face_tracker_.Resume();
            Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
            camera_app_busy_.store(false);
            return false;
        }

        if (!shutter_pressed) {
            ESP_LOGI(TAG, "AI camera shutter timed out; capturing automatically");
            ui->SetCameraAppStatus("未按快门，正在自动拍照…");
        } else {
            ui->SetCameraAppStatus("正在发送给助手…");
        }
        return true;
    }

    virtual void FinishCameraCapture(bool success) override {
        auto* ui = static_cast<M5StackAvatarDisplay*>(display_);
        ui->SetCameraAppStatus(success ? "照片已发送给助手" : "照片发送失败");
        face_tracker_.Resume();
        Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        camera_app_busy_.store(false);
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }

    virtual Backlight *GetBacklight() override {
        static CustomBacklight backlight(pmic_);
        return &backlight;
    }
};

DECLARE_BOARD(M5StackCoreS3Board);
