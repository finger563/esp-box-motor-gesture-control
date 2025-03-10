#include "gui.hpp"

extern "C" {
#include "ui.h"
#include "ui_helpers.h"
// #include "ui_comp.h" // we don't have any components in this project yet
}

void Gui::deinit_ui() {
  logger_.info("Deinitializing UI");
  // delete the ui
  lv_obj_del(ui_Screen1);
  lv_obj_del(ui_Screen2);
}

void Gui::init_ui() {
  logger_.info("Initializing UI");
  ui_init();
  // register event handlers for:
  // - BondButton pressed
  // - EnabledCheckbox value changed
  // - ControlDropdown value changed
  lv_obj_add_event_cb(ui_BondButton, &Gui::event_callback, LV_EVENT_PRESSED,
                      static_cast<void *>(this));
  lv_obj_add_event_cb(ui_EnabledCheckbox, &Gui::event_callback, LV_EVENT_VALUE_CHANGED,
                      static_cast<void *>(this));
  lv_obj_add_event_cb(ui_ControlDropdown, &Gui::event_callback, LV_EVENT_VALUE_CHANGED,
                      static_cast<void *>(this));
}

void Gui::show_bond_screen() {
  logger_.info("Showing bond screen");
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  lv_scr_load(ui_Screen1);
  _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, &ui_Screen1_screen_init);
}

void Gui::show_control_screen() {
  logger_.info("Showing control screen");
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  lv_scr_load(ui_Screen2);
  _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, &ui_Screen2_screen_init);
}

void Gui::set_target_label_text(const std::string &text) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  lv_label_set_text(ui_TargetLabel, text.c_str());
}

void Gui::set_image_rotation(float angle_rads) {
  angle_rads = std::fmod(angle_rads, 2 * M_PI);
  int angle = angle_rads * 180 / M_PI * 10;
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  // NOTE: angle has .1 degree precision and takes an integer, so we multiply by
  //       10 to get the final value
  lv_img_set_angle(ui_MotorImage, angle);
  // we also have to set the style for the Panel3 and TargetLabel so that they
  // rotate with the image
  // lv_obj_set_style_transform_angle(ui_Panel3, angle, LV_PART_MAIN);
  lv_obj_set_style_transform_angle(ui_TargetLabel, angle, LV_PART_MAIN);
}

void Gui::on_value_changed(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.debug("Value changed: {}", fmt::ptr(target));
  if (target == ui_EnabledCheckbox) {
    logger_.info("EnabledCheckbox value changed");
    // call the callback if it's set
    if (on_enabled_check_box_checked_) {
      // get the value of the checkbox
      bool checked = lv_obj_get_state(ui_EnabledCheckbox) & LV_STATE_CHECKED;
      on_enabled_check_box_checked_(checked);
    }
  } else if (target == ui_ControlDropdown) {
    logger_.info("ControlDropdown value changed");
    // call the callback if it's set
    if (on_control_drop_down_value_changed_) {
      // get the value of the dropdown
      int selected = lv_dropdown_get_selected(ui_ControlDropdown);
      on_control_drop_down_value_changed_(selected);
    }
  }
}

void Gui::on_pressed(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.debug("PRESSED: {}", fmt::ptr(target));
  if (target == ui_BondButton) {
    logger_.info("BondButton pressed");
    // call the callback if it's set
    if (on_bond_button_pressed_) {
      on_bond_button_pressed_();
    }
  }
}

void Gui::on_scroll(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("SCROLL: {}", fmt::ptr(target));
}

void Gui::on_key(lv_event_t *e) {
  // print the key
  auto key = lv_indev_get_key(lv_indev_get_act());
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("KEY: {} on {}", key, fmt::ptr(target));
}
