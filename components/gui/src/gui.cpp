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

  // TODO: register event handlers for:
  // - BondButton pressed
  // - EnabledCheckBox value changed
  // - ControlDropDown value changed
}

void Gui::on_value_changed(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("Value changed: {}", fmt::ptr(target));
}

void Gui::on_pressed(lv_event_t *e) {
  lv_obj_t *target = (lv_obj_t *)lv_event_get_target(e);
  logger_.info("PRESSED: {}", fmt::ptr(target));
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
