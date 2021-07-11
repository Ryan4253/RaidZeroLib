#pragma once
#include "gifdec.h"
#include "display/lvgl.h"
#include "lib4253/Utility/TaskWrapper.hpp"

/**
 * MIT License
 * Copyright (c) 2019 Theo Lemay
 * https://github.com/theol0403/gif-pros
 */

namespace lib4253{

class Gif: public TaskWrapper{

public:

  /**
   * Construct the Gif class
   * @param fname  the gif filename on the SD card (prefixed with /usd/)
   * @param parent the LVGL parent object
   */
  Gif(const char* fname, lv_obj_t* parent);

  /**
   * Destructs and cleans the Gif class
   */
  ~Gif();

  /**
   * Deletes GIF and frees all allocated memory
   */
  void clean();

private:

  gd_GIF* _gif = nullptr; // gif decoder object
  void* _gifmem = nullptr; // gif file loaded from SD into memory 
  uint8_t* _buffer = nullptr; // decoder frame buffer

  lv_color_t* _cbuf = nullptr; // canvas buffer
  lv_obj_t* _canvas = nullptr; // canvas object

  /**
   * Cleans and frees all allocated memory
   */
  void _cleanup();

  /**
   * Render cycle, blocks until loop count exceeds gif loop count flag (if any)
   */
  void loop() override;
};
}