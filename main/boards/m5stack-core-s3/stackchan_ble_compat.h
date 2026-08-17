#pragma once

#include <atomic>
#include <functional>
#include <string>

// BLE peripheral compatible with the official M5Stack StackChan app's
// Dance mode.  The phone owns recording and playback; the device receives
// live motion, avatar and RGB JSON frames.
class StackChanBleCompat {
public:
    using DataCallback = std::function<void(const std::string&)>;
    enum class Status { Advertising, Connected, Error };
    using StatusCallback = std::function<void(Status)>;

    bool Start(DataCallback motion, DataCallback avatar, DataCallback rgb,
               StatusCallback status);
    void Stop();
    bool IsActive() const { return active_.load(); }
    bool IsConnected() const { return connected_.load(); }

    static StackChanBleCompat& Instance();

    void DispatchMotion(const char* data, size_t len);
    void DispatchAvatar(const char* data, size_t len);
    void DispatchRgb(const char* data, size_t len);
    void SetConnected(bool connected);
    void ReportAdvertisingResult(bool ready);

private:
    StackChanBleCompat() = default;

    DataCallback motion_callback_;
    DataCallback avatar_callback_;
    DataCallback rgb_callback_;
    StatusCallback status_callback_;
    std::atomic<bool> active_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> host_running_{false};
};
