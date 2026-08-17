#pragma once

#include <lvgl.h>


class LvglFont {
public:
    virtual const lv_font_t* font() const = 0;
    virtual ~LvglFont() = default;
};

// Built-in font
class LvglBuiltInFont : public LvglFont {
public:
    LvglBuiltInFont(const lv_font_t* font) : font_(font) {}
    virtual const lv_font_t* font() const override { return font_; }

private:
    const lv_font_t* font_;
};


class LvglCBinFont : public LvglFont {
public:
    LvglCBinFont(void* data);
    virtual ~LvglCBinFont();
    virtual const lv_font_t* font() const override { return font_; }
    void set_fallback(const lv_font_t* fallback) {
        if (font_ != nullptr) {
            font_->fallback = fallback;
        }
    }

private:
    lv_font_t* font_;
};
