#include "main.h"
namespace lib4253{

// Styles
lv_style_t autoSelectBtnStyleREL;
lv_style_t closeBtnStyleREL;
lv_style_t normalBtnStyleREL;
lv_style_t skillsBtnStyleREL;
lv_style_t noneBtnStyleREL;
lv_style_t redBtnStyleREL;
lv_style_t blueBtnStyleREL;
lv_style_t driveBtnStyleREL;
lv_style_t nearBtnStyleREL;
lv_style_t farBtnStyleREL;
lv_style_t homeRowBtnStyleREL;
lv_style_t myButtonStylePR; //pressed style

// Autonomous selector button
lv_obj_t * autoSelectBtn;
lv_obj_t * autoSelectBtnLabel;

// First row buttons: Close/exit, normal, skills
lv_obj_t * closeBtn;
lv_obj_t * closeBtnLabel;

lv_obj_t * normalBtn;
lv_obj_t * normalBtnLabel;

lv_obj_t * skillsBtn;
lv_obj_t * skillsBtnLabel;

// Second row buttons: none, blue, red, home row
lv_obj_t * noneBtn;
lv_obj_t * noneBtnLabel;

lv_obj_t * redBtn;
lv_obj_t * redBtnLabel;

lv_obj_t * blueBtn;
lv_obj_t * blueBtnLabel;

lv_obj_t * driveBtn;
lv_obj_t * driveBtnLabel;

// Third row buttons: near tile, far tile
lv_obj_t * nearTileBtn;
lv_obj_t * nearTileBtnLabel;

lv_obj_t * farTileBtn;
lv_obj_t * farTileBtnLabel;

lv_obj_t * homeRowBtn;
lv_obj_t * homeRowBtnLabel;

// Description at the bottom of the screen
lv_obj_t * txt1;
lv_obj_t * txt2;
lv_obj_t * txt3;
lv_obj_t * driveTxt;

bool homeRowToggle = false;

lv_obj_t * createBtn(lv_obj_t * parent, lv_coord_t x, lv_coord_t y, lv_coord_t width, lv_coord_t height,
    int id, const char * title)
{
    lv_obj_t * btn = lv_btn_create(parent, NULL);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, width, height);
    lv_obj_set_free_num(btn, id);

    lv_obj_t * label = lv_label_create(btn, NULL);
    lv_label_set_text(label, title);
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

    return btn;
}

lv_style_t * createBtnStyle(lv_style_t * copy, lv_color_t rel, lv_color_t pr,
    lv_color_t tglRel, lv_color_t tglPr, lv_color_t tglBorder, lv_color_t textColor, lv_obj_t * btn)
{
    lv_style_t * btnStyle = (lv_style_t *)malloc(sizeof(lv_style_t) * 4);

    for(int i = 0; i < 4; i++) lv_style_copy(&btnStyle[i], copy);

    btnStyle[0].body.main_color = rel;
    btnStyle[0].body.grad_color = rel;
    btnStyle[0].text.color = textColor;

    btnStyle[1].body.main_color = pr;
    btnStyle[1].body.grad_color = pr;
    btnStyle[1].text.color = textColor;

    btnStyle[2].body.main_color = tglRel;
    btnStyle[2].body.grad_color = tglRel;
    btnStyle[2].body.border.width = 2;
    btnStyle[2].body.border.color = tglBorder;
    btnStyle[2].text.color = textColor;

    btnStyle[3].body.main_color = tglPr;
    btnStyle[3].body.grad_color = tglPr;
    btnStyle[3].body.border.width = 2;
    btnStyle[3].body.border.color = tglBorder;
    btnStyle[3].text.color = textColor;

    lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
    lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);

    return btnStyle;
}

void setBtnStyle(lv_style_t * btnStyle, lv_obj_t * btn) {
  lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
  lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
  lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
  lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);
}

static lv_res_t btn_click_action(lv_obj_t * btn) {
  uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

  switch(id) {
    case 0: // autonomous selector
    lv_obj_set_hidden(autoSelectBtn, true);
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, true);
    lv_obj_set_hidden(redBtn, true);
    lv_obj_set_hidden(blueBtn, true);
    lv_obj_set_hidden(driveBtn, true);
    lv_obj_set_hidden(nearTileBtn, true);
    lv_obj_set_hidden(farTileBtn, true);
    lv_obj_set_hidden(homeRowBtn, true);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, true);
    lv_obj_set_hidden(txt3, true);
    lv_obj_set_hidden(driveTxt, true);

    lv_label_set_text(txt1, "The robot will do nothing.");
    break;

    case 1: // close
    lv_obj_set_hidden(autoSelectBtn, false);
    lv_obj_set_hidden(closeBtn, true);
    lv_obj_set_hidden(normalBtn, true);
    lv_obj_set_hidden(skillsBtn, true);
    lv_obj_set_hidden(noneBtn, true);
    lv_obj_set_hidden(redBtn, true);
    lv_obj_set_hidden(blueBtn, true);
    lv_obj_set_hidden(driveBtn, true);
    lv_obj_set_hidden(nearTileBtn, true);
    lv_obj_set_hidden(farTileBtn, true);
    lv_obj_set_hidden(homeRowBtn, true);
    lv_obj_set_hidden(txt1, true);
    lv_obj_set_hidden(txt2, true);
    lv_obj_set_hidden(txt3, true);
    lv_obj_set_hidden(driveTxt, true);
    break;

    case 2: // normal
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, true);
    lv_obj_set_hidden(farTileBtn, true);
    lv_obj_set_hidden(homeRowBtn, true);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, false);
    lv_obj_set_hidden(txt3, false);
    lv_obj_set_hidden(driveTxt, true);

    lv_label_set_text(txt1, "The robot will perform match autonomous routine.");
    break;

    case 3: // skills
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, true);
    lv_obj_set_hidden(farTileBtn, true);
    lv_obj_set_hidden(homeRowBtn, true);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, false);
    lv_obj_set_hidden(txt3, false);
    lv_obj_set_hidden(driveTxt, true);

    lv_label_set_text(txt1, "The robot will do skills.");
    break;

    case 4: // none
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, true);
    lv_obj_set_hidden(farTileBtn, true);
    lv_obj_set_hidden(homeRowBtn, true);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, true);
    lv_obj_set_hidden(txt3, true);
    lv_obj_set_hidden(driveTxt, true);

    lv_label_set_text(txt1, "The robot will do nothing.");
    break;

    case 5: // red
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, false);
    lv_obj_set_hidden(farTileBtn, false);
    lv_obj_set_hidden(homeRowBtn, false);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, false);
    lv_obj_set_hidden(txt3, false);
    lv_obj_set_hidden(driveTxt, true);

    lv_label_set_text(txt2, "The robot will be on the red side.");
    break;

    case 6: // blue
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, false);
    lv_obj_set_hidden(farTileBtn, false);
    lv_obj_set_hidden(homeRowBtn, false);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, false);
    lv_obj_set_hidden(txt3, false);
    lv_obj_set_hidden(driveTxt, true);

    lv_label_set_text(txt2, "The robot will be on the blue side.");
    break;

    case 7: // drive
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, true);
    lv_obj_set_hidden(farTileBtn, true);
    lv_obj_set_hidden(homeRowBtn, true);
    lv_obj_set_hidden(txt1, true);
    lv_obj_set_hidden(txt2, true);
    lv_obj_set_hidden(txt3, true);
    lv_obj_set_hidden(driveTxt, false);

    lv_label_set_text(driveTxt, "The robot will drive at a set speed.");
    break;

    case 8: // near tile
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, false);
    lv_obj_set_hidden(farTileBtn, false);
    lv_obj_set_hidden(homeRowBtn, false);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, false);
    lv_obj_set_hidden(txt3, false);

    lv_label_set_text(txt3, "The robot will be on the near tile.");
    break;

    case 9: // far tile
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, false);
    lv_obj_set_hidden(farTileBtn, false);
    lv_obj_set_hidden(homeRowBtn, false);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, false);
    lv_obj_set_hidden(txt3, false);

    lv_label_set_text(txt3, "The robot will be on the far tile.");
    break;

    case 10: // home row
    lv_obj_set_hidden(closeBtn, false);
    lv_obj_set_hidden(normalBtn, false);
    lv_obj_set_hidden(skillsBtn, false);
    lv_obj_set_hidden(noneBtn, false);
    lv_obj_set_hidden(redBtn, false);
    lv_obj_set_hidden(blueBtn, false);
    lv_obj_set_hidden(driveBtn, false);
    lv_obj_set_hidden(nearTileBtn, false);
    lv_obj_set_hidden(farTileBtn, false);
    lv_obj_set_hidden(homeRowBtn, false);
    lv_obj_set_hidden(txt1, false);
    lv_obj_set_hidden(txt2, false);
    lv_obj_set_hidden(txt3, false);

    homeRowToggle = !homeRowToggle;

    homeRowToggle ? lv_label_set_text(homeRowBtnLabel, "Home Row: Yes") : lv_label_set_text(homeRowBtnLabel, "Home Row: No");
    break;
  }

  return LV_RES_OK;
}

void gui() {
// Button styles
  lv_style_copy(&autoSelectBtnStyleREL, &lv_style_plain);
  autoSelectBtnStyleREL.body.main_color = LV_COLOR_MAKE(173, 216, 230);
  autoSelectBtnStyleREL.body.grad_color = LV_COLOR_MAKE(173, 216, 230);
  autoSelectBtnStyleREL.body.radius = 0;
  autoSelectBtnStyleREL.text.color = LV_COLOR_MAKE(0, 0, 139);

  lv_style_copy(&closeBtnStyleREL, &lv_style_plain);
  closeBtnStyleREL.body.main_color = LV_COLOR_MAKE(155, 0, 0);
  closeBtnStyleREL.body.grad_color = LV_COLOR_MAKE(150, 0, 0);
  closeBtnStyleREL.body.radius = 0;
  closeBtnStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

  lv_style_copy(&normalBtnStyleREL, &lv_style_plain);
  normalBtnStyleREL.body.main_color = LV_COLOR_MAKE(0, 0, 128);
  normalBtnStyleREL.body.grad_color = LV_COLOR_MAKE(0, 0, 128);
  normalBtnStyleREL.body.radius = 0;
  normalBtnStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

  lv_style_copy(&skillsBtnStyleREL, &lv_style_plain);
  skillsBtnStyleREL.body.main_color = LV_COLOR_MAKE(173, 216, 230);
  skillsBtnStyleREL.body.grad_color = LV_COLOR_MAKE(173, 216, 230);
  skillsBtnStyleREL.body.radius = 0;
  skillsBtnStyleREL.text.color = LV_COLOR_MAKE(0, 0, 139);

  lv_style_copy(&noneBtnStyleREL, &lv_style_plain);
  noneBtnStyleREL.body.main_color = LV_COLOR_MAKE(173, 207, 230);
  noneBtnStyleREL.body.grad_color = LV_COLOR_MAKE(173, 207, 230);
  noneBtnStyleREL.body.radius = 0;
  noneBtnStyleREL.text.color = LV_COLOR_MAKE(0, 0, 139);

  lv_style_copy(&redBtnStyleREL, &lv_style_plain);
  redBtnStyleREL.body.main_color = LV_COLOR_MAKE(155, 0, 0);
  redBtnStyleREL.body.grad_color = LV_COLOR_MAKE(150, 0, 0);
  redBtnStyleREL.body.radius = 0;
  redBtnStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

  lv_style_copy(&blueBtnStyleREL, &lv_style_plain);
  blueBtnStyleREL.body.main_color = LV_COLOR_MAKE(40, 96, 134);
  blueBtnStyleREL.body.grad_color = LV_COLOR_MAKE(40, 96, 134);
  blueBtnStyleREL.body.radius = 0;
  blueBtnStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

  lv_style_copy(&driveBtnStyleREL, &lv_style_plain);
  driveBtnStyleREL.body.main_color = LV_COLOR_MAKE(173, 216, 230);
  driveBtnStyleREL.body.grad_color = LV_COLOR_MAKE(173, 216, 230);
  driveBtnStyleREL.body.radius = 0;
  driveBtnStyleREL.text.color = LV_COLOR_MAKE(0, 0, 139);

  lv_style_copy(&nearBtnStyleREL, &lv_style_plain);
  nearBtnStyleREL.body.main_color = LV_COLOR_MAKE(173, 216, 230);
  nearBtnStyleREL.body.grad_color = LV_COLOR_MAKE(173, 216, 230);
  nearBtnStyleREL.body.radius = 0;
  nearBtnStyleREL.text.color = LV_COLOR_MAKE(0, 0, 139);

  lv_style_copy(&farBtnStyleREL, &lv_style_plain);
  farBtnStyleREL.body.main_color = LV_COLOR_MAKE(173, 216, 230);
  farBtnStyleREL.body.grad_color = LV_COLOR_MAKE(173, 216, 230);
  farBtnStyleREL.body.radius = 0;
  farBtnStyleREL.text.color = LV_COLOR_MAKE(0, 0, 139);

  lv_style_copy(&homeRowBtnStyleREL, &lv_style_plain);
  homeRowBtnStyleREL.body.main_color = LV_COLOR_MAKE(173, 216, 230);
  homeRowBtnStyleREL.body.grad_color = LV_COLOR_MAKE(173, 216, 230);
  homeRowBtnStyleREL.body.radius = 0;
  homeRowBtnStyleREL.text.color = LV_COLOR_MAKE(0, 0, 139);

  lv_style_copy(&myButtonStylePR, &lv_style_plain);
  myButtonStylePR.body.main_color = LV_COLOR_MAKE(211, 211, 211);
  myButtonStylePR.body.grad_color = LV_COLOR_MAKE(211, 211, 211);
  myButtonStylePR.body.radius = 0;
  myButtonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

// ID = 0
  autoSelectBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(autoSelectBtn, 0);
  lv_btn_set_action(autoSelectBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(autoSelectBtn, LV_BTN_STYLE_REL, &autoSelectBtnStyleREL);
  lv_btn_set_style(autoSelectBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(autoSelectBtn, 300, 40);
  lv_obj_align(autoSelectBtn, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);

  autoSelectBtnLabel = lv_label_create(autoSelectBtn, NULL);
  lv_label_set_text(autoSelectBtnLabel, "Open Autonomous Selector");
  lv_obj_set_hidden(autoSelectBtn, false);

// ID = 1
  closeBtn = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is default screen object
  lv_obj_set_free_num(closeBtn, 1); //set button id to 1
  lv_btn_set_action(closeBtn, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
  lv_btn_set_style(closeBtn, LV_BTN_STYLE_REL, &closeBtnStyleREL); //set the released style
  lv_btn_set_style(closeBtn, LV_BTN_STYLE_PR, &myButtonStylePR); //set the pressed style
  lv_obj_set_size(closeBtn, 100, 40); //set the button size
  lv_obj_align(closeBtn, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0); //set the position to top left

  closeBtnLabel = lv_label_create(closeBtn, NULL); //create label and puts it inside of the button
  lv_label_set_text(closeBtnLabel, "close"); //sets label text
  lv_obj_set_hidden(closeBtn, true);

// ID = 2
  normalBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(normalBtn, 2);
  lv_btn_set_action(normalBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(normalBtn, LV_BTN_STYLE_REL, &normalBtnStyleREL);
  lv_btn_set_style(normalBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(normalBtn, 187, 40);
  lv_obj_align(normalBtn, NULL, LV_ALIGN_IN_TOP_MID, -44, 0);

  normalBtnLabel = lv_label_create(normalBtn, NULL);
  lv_label_set_text(normalBtnLabel, "Normal (15s)");
  lv_obj_set_hidden(normalBtn, true);

// ID = 3
  skillsBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(skillsBtn, 3);
  lv_btn_set_action(skillsBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(skillsBtn, LV_BTN_STYLE_REL, &skillsBtnStyleREL);
  lv_btn_set_style(skillsBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(skillsBtn, 187, 40);
  lv_obj_align(skillsBtn, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);

  skillsBtnLabel = lv_label_create(skillsBtn, NULL);
  lv_label_set_text(skillsBtnLabel, "Skills (60s)");
  lv_obj_set_hidden(skillsBtn, true);

// ID = 4
  noneBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(noneBtn, 4);
  lv_btn_set_action(noneBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(noneBtn, LV_BTN_STYLE_REL, &noneBtnStyleREL);
  lv_btn_set_style(noneBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(noneBtn, 110, 45);
  lv_obj_align(noneBtn, NULL, LV_ALIGN_IN_TOP_LEFT, 12, 45);

  noneBtnLabel = lv_label_create(noneBtn, NULL);
  lv_label_set_text(noneBtnLabel, "None");
  lv_obj_set_hidden(noneBtn, true);

// ID = 5
  redBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(redBtn, 5);
  lv_btn_set_action(redBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(redBtn, LV_BTN_STYLE_REL, &redBtnStyleREL);
  lv_btn_set_style(redBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(redBtn, 110, 45);
  lv_obj_align(redBtn, NULL, LV_ALIGN_IN_TOP_LEFT, 127, 45);

  redBtnLabel = lv_label_create(redBtn, NULL);
  lv_label_set_text(redBtnLabel, "Red");
  lv_obj_set_hidden(redBtn, true);

// ID = 6
  blueBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(blueBtn, 6);
  lv_btn_set_action(blueBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(blueBtn, LV_BTN_STYLE_REL, &blueBtnStyleREL);
  lv_btn_set_style(blueBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(blueBtn, 110, 45);
  lv_obj_align(blueBtn, NULL, LV_ALIGN_IN_TOP_MID, 57, 45);

  blueBtnLabel = lv_label_create(blueBtn, NULL);
  lv_label_set_text(blueBtnLabel, "Blue");
  lv_obj_set_hidden(blueBtn, true);

// ID = 7
  driveBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(driveBtn, 7);
  lv_btn_set_action(driveBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(driveBtn, LV_BTN_STYLE_REL, &driveBtnStyleREL);
  lv_btn_set_style(driveBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(driveBtn, 110, 45);
  lv_obj_align(driveBtn, NULL, LV_ALIGN_IN_TOP_RIGHT, -13, 45);

  driveBtnLabel = lv_label_create(driveBtn, NULL);
  lv_label_set_text(driveBtnLabel, "Drive");
  lv_obj_set_hidden(driveBtn, true);

// ID = 8
  nearTileBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(nearTileBtn, 8);
  lv_btn_set_action(nearTileBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(nearTileBtn, LV_BTN_STYLE_REL, &nearBtnStyleREL);
  lv_btn_set_style(nearTileBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(nearTileBtn, 130, 45);
  lv_obj_align(nearTileBtn, NULL, LV_ALIGN_IN_LEFT_MID, 5, -2);

  nearTileBtnLabel = lv_label_create(nearTileBtn, NULL);
  lv_label_set_text(nearTileBtnLabel, "Near Tile");
  lv_obj_set_hidden(nearTileBtn, true);

// ID = 9
  farTileBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(farTileBtn, 9);
  lv_btn_set_action(farTileBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(farTileBtn, LV_BTN_STYLE_REL, &farBtnStyleREL);
  lv_btn_set_style(farTileBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(farTileBtn, 130, 45);
  lv_obj_align(farTileBtn, NULL, LV_ALIGN_IN_LEFT_MID, 140, -2);

  farTileBtnLabel = lv_label_create(farTileBtn, NULL);
  lv_label_set_text(farTileBtnLabel, "Far Tile");
  lv_obj_set_hidden(farTileBtn, true);

// ID = 10
  homeRowBtn = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_free_num(homeRowBtn, 10);
  lv_btn_set_action(homeRowBtn, LV_BTN_ACTION_CLICK, btn_click_action);
  lv_btn_set_style(homeRowBtn, LV_BTN_STYLE_REL, &homeRowBtnStyleREL);
  lv_btn_set_style(homeRowBtn, LV_BTN_STYLE_PR, &myButtonStylePR);
  lv_obj_set_size(homeRowBtn, 200, 45);
  lv_obj_align(homeRowBtn, NULL, LV_ALIGN_IN_RIGHT_MID, -3, -2);

  homeRowBtnLabel = lv_label_create(homeRowBtn, NULL);
  lv_label_set_text(homeRowBtnLabel, "Home Row: No");
  lv_obj_set_hidden(homeRowBtn, true);

  txt1 = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(txt1, "");
  lv_obj_align(txt1, NULL, LV_ALIGN_IN_LEFT_MID, 10, 50);
  lv_obj_set_hidden(txt1, true);

  txt2 = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(txt2, "");
  lv_obj_align(txt2, NULL, LV_ALIGN_IN_LEFT_MID, 10, 70);
  lv_obj_set_hidden(txt2, true);

  txt3 = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(txt3, "");
  lv_obj_align(txt3, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -20);
  lv_obj_set_hidden(txt3, true);

  driveTxt = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(driveTxt, "");
  lv_obj_align(driveTxt, NULL, LV_ALIGN_IN_LEFT_MID, 10, 50);
  lv_obj_set_hidden(driveTxt, true);
}
}