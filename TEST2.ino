#include <LovyanGFX.hpp>
#include <lvgl.h>

/* =====================================================
   LovyanGFX configuration for WT32-SC01 Plus
   ===================================================== */
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7796  panel;
  lgfx::Bus_Parallel8 bus;
  lgfx::Light_PWM     light;
  lgfx::Touch_FT5x06  touch;

public:
  LGFX(void)
  {
    // -------- BUS --------
    {
      auto cfg = bus.config();
      cfg.freq_write = 40000000;
      cfg.pin_wr = 47;
      cfg.pin_rd = -1;
      cfg.pin_rs = 0;

      cfg.pin_d0 = 9;
      cfg.pin_d1 = 46;
      cfg.pin_d2 = 3;
      cfg.pin_d3 = 8;
      cfg.pin_d4 = 18;
      cfg.pin_d5 = 17;
      cfg.pin_d6 = 16;
      cfg.pin_d7 = 15;

      bus.config(cfg);
      panel.setBus(&bus);
    }

    // -------- PANEL --------
    {
      auto cfg = panel.config();
      cfg.pin_cs = -1;
      cfg.pin_rst = 4;
      cfg.panel_width = 320;
      cfg.panel_height = 480;

      cfg.invert = true;
      cfg.rgb_order = true;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      cfg.offset_rotation = 0;   // ❗ NO rotation here

      panel.config(cfg);
    }

    // -------- BACKLIGHT --------
    {
      auto cfg = light.config();
      cfg.pin_bl = 45;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      light.config(cfg);
      panel.setLight(&light);
    }

    // -------- TOUCH --------
    {
      auto cfg = touch.config();

      // RAW FT5x06 RANGE (MEASURED)
      cfg.x_min = 0;
      cfg.x_max = 719;
      cfg.y_min = 0;
      cfg.y_max = 319;

      cfg.offset_rotation = 0;   // RAW data
      cfg.bus_shared = false;
      cfg.pin_int = -1;

      cfg.i2c_port = 1;
      cfg.i2c_addr = 0x38;
      cfg.pin_sda = 6;
      cfg.pin_scl = 5;
      cfg.freq = 400000;

      touch.config(cfg);
      panel.setTouch(&touch);
    }

    setPanel(&panel);
  }
};

LGFX display;

/* =====================================================
   LVGL
   ===================================================== */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[480 * 20];

lv_obj_t *menu1;
lv_obj_t *menu2;

/* =====================================================
   Display flush
   ===================================================== */
void my_disp_flush(lv_disp_drv_t *disp,
                   const lv_area_t *area,
                   lv_color_t *color_p)
{
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  display.startWrite();
  display.setAddrWindow(area->x1, area->y1, w, h);
  display.pushPixels((uint16_t *)color_p, w * h);
  display.endWrite();

  lv_disp_flush_ready(disp);
}

/* =====================================================
   TOUCH → LVGL (FINAL, CORRECT FOR YOUR BOARD)
   ===================================================== */
void my_touch_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
  uint16_t x, y;

  if (!display.getTouch(&x, &y))
  {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  data->state = LV_INDEV_STATE_PR;

  // RAW:  X = 0..719  → LVGL X = 0..479
  // RAW:  Y = 0..319  → LVGL Y = 0..319
  data->point.x = map(x, 0, 719, 0, 479);
  data->point.y = map(y, 0, 319, 0, 319);
}


void goto_menu2(lv_event_t *e)
{
  lv_scr_load_anim(menu2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
}

void goto_menu1(lv_event_t *e)
{
  lv_scr_load_anim(menu1, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
}


void create_menu1()
{
  menu1 = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(menu1, lv_color_hex(0x202020), 0);

  lv_obj_t *title = lv_label_create(menu1);
  lv_label_set_text(title, "Hitrost");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t *btn = lv_btn_create(menu1);
  lv_obj_set_size(btn, 220, 70);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 40);
  lv_obj_add_event_cb(btn, goto_menu2, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "BMS");
  lv_obj_center(lbl);
}


void create_menu2()
{
  menu2 = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(menu2, lv_color_hex(0x663300), 0);

  lv_obj_t *title = lv_label_create(menu2);
  lv_label_set_text(title, "BMS Podatki");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t *btn = lv_btn_create(menu2);
  lv_obj_set_size(btn, 120, 60);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, 20, -20);
  lv_obj_add_event_cb(btn, goto_menu1, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "Nazaj");
  lv_obj_center(lbl);
}


void setup()
{
  Serial.begin(115200);

  display.init();
  display.setRotation(1);     // LANDSCAPE
  display.setBrightness(255);

  lv_init();

  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 480 * 20);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 480;
  disp_drv.ver_res = 320;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);

  create_menu1();
  create_menu2();
  lv_scr_load(menu1);
}

void loop()
{
  static uint32_t last = 0;
  uint32_t now = millis();

  lv_tick_inc(now - last);
  last = now;

  lv_timer_handler();
  delay(1);
}
