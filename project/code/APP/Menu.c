/**
 * @file    Menu.c
 * @brief   串口屏菜单系统实现 (按键快照优化版)
 */

#include "Menu.h"
#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include "zf_device_gnss.h"
#include <stdint.h>
#include "IMU_Deal.h"
#include "Motor.h"
#include "controller.h"
#include "chassic.h"
#include "minesweep.h"
#include "GPS.h"
#include "IPS200.h"
#include "ins_navigation.h"
#include <stdio.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
//  颜色宏
// ─────────────────────────────────────────────────────────────────────────────
#define COLOR_WHITE     (0xFFFF)
#define COLOR_BLACK     (0x0000)
#define COLOR_BLUE      (0x001F)
#define COLOR_CYAN      (0x07FF)
#define COLOR_GREEN     (0x07E0)
#define COLOR_YELLOW    (0xFFE0)
#define COLOR_RED       (0xF800)
#define COLOR_SELECTED  (0x07FF)   // 高亮选中行：青色

// ─────────────────────────────────────────────────────────────────────────────
//  字体尺寸 (8x16)
// ─────────────────────────────────────────────────────────────────────────────
#define FONT_W  8
#define FONT_H  16
#define ROW(n)  ((n) * FONT_H)

// ─────────────────────────────────────────────────────────────────────────────
//  菜单状态机
// ─────────────────────────────────────────────────────────────────────────────
typedef enum
{
    STATE_STARTUP_PID = 0, // 启动时PID预调阶段
    STATE_MAIN_MENU,       // 主菜单
    STATE_IMU,             // IMU 数据页
    STATE_PID_LIST,        // PID 参数列表
    STATE_PID_EDIT,        // 正在编辑某个 PID 参数
    STATE_SPEED,           // 小车速度页
    STATE_GPS,             // GPS 信息页
    STATE_SUBJECT1,        // 科目1 惯导子菜单
    STATE_SUBJECT2,        // 科目2 定点排雷子菜单
} menu_state_t;

// 主菜单条目
#define MAIN_ITEM_COUNT  6
static const char *s_main_items[MAIN_ITEM_COUNT] = {
    "1. IMU  Data",
    "2. PID  Params",
    "3. Speed",
    "4. GPS  Info",
    "5. Subject1",
    "6. Subject2",
};

// PID 参数条目
#define PID_ITEM_COUNT   4
static const char *s_pid_items[PID_ITEM_COUNT] = {
    "GYRO_KP",
    "ANG_KP ",
    "ANG_KI ",
    "ANG_KD ",
};
static const float s_pid_steps[PID_ITEM_COUNT] = {10.0f, 0.2f, 0.001f, 0.01f};

#ifdef MENU_STARTUP_PID_TUNE
static menu_state_t s_state        = STATE_STARTUP_PID;
static uint8      s_startup_done = 0;  // 启动阶段完成标志
#else
static menu_state_t s_state        = STATE_MAIN_MENU;
static uint8      s_startup_done = 1;  // 跳过预调，直接视为已完成
#endif
static int8_t       s_main_sel    = 0;   // 主菜单光标
static int8_t       s_pid_sel     = 0;   // PID 列表光标
static int8_t       s_subj1_sel   = 0;   // 科目1子菜单光标
static int8_t       s_subj2_sel   = 0;   // 科目2子菜单光标
static uint8      s_need_redraw = 1;   // 强制刷屏标志

// ─────────────────────────────────────────────────────────────────────────────
//  获取当前 PID 参数值指针 (全局变量)
// ─────────────────────────────────────────────────────────────────────────────
static float *get_global_pid_var(int8_t idx)
{
    switch (idx)
    {
        case 0: return &GYRO_KP;
        case 1: return &ANG_KP;
        case 2: return &ANG_KI;
        case 3: return &ANG_KD;
        default: return NULL;
    }
}

// 同步全局变量到 PID 实例
static void sync_pid_to_instance(int8_t idx)
{
    PIDInstance *ap = Motor_Get_Angle_PID();
    PIDInstance *gp = Motor_Get_Gyro_PID();
    switch (idx)
    {
        case 0: gp->Kp = GYRO_KP; break;
        case 1: ap->Kp = ANG_KP;  break;
        case 2:
            ap->Ki    = ANG_KI;
            ap->ITerm = 0.0f;
            ap->Iout  = 0.0f;
            break;
        case 3: ap->Kd = ANG_KD; break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  屏幕绘制函数
// ─────────────────────────────────────────────────────────────────────────────
static void draw_title(const char *title)
{
    ips200_set_color(COLOR_WHITE, COLOR_BLUE);
    // 填充标题背景行
    for (uint16 x = 0; x < 240; x++)
        for (uint16 y = 0; y < FONT_H; y++)
            ips200_draw_point(x, y, COLOR_BLUE);
    ips200_show_string(4, 0, title);
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);
}

static void draw_main_menu(void)
{
    ips200_clear();
    draw_title("  === MENU ===  ");
    for (int8_t i = 0; i < MAIN_ITEM_COUNT; i++)
    {
        uint16 y = ROW(i + 1) + 4;
        if (i == s_main_sel)
        {
            ips200_set_color(COLOR_BLACK, COLOR_SELECTED);
            ips200_show_string(0, y, ">");
            ips200_show_string(FONT_W, y, s_main_items[i]);
            ips200_set_color(COLOR_WHITE, COLOR_BLACK);
        }
        else
        {
            ips200_show_string(0, y, " ");
            ips200_show_string(FONT_W, y, s_main_items[i]);
        }
    }
    ips200_show_string(0, ROW(8), "[0]Dn [1]Up [2]OK");
}

static void draw_imu_page(void)
{
    char buf[32];
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);

    snprintf(buf, sizeof(buf), "Pitch:%+7.2f deg", imu_sys.pitch);
    ips200_show_string(0, ROW(1), buf);

    snprintf(buf, sizeof(buf), "Roll :%+7.2f deg", imu_sys.roll);
    ips200_show_string(0, ROW(2), buf);

    snprintf(buf, sizeof(buf), "Yaw  :%+7.2f deg", imu_sys.yaw);
    ips200_show_string(0, ROW(3), buf);

    snprintf(buf, sizeof(buf), "Gx   :%+8.4f r/s", imu_sys.gx);
    ips200_show_string(0, ROW(4), buf);

    snprintf(buf, sizeof(buf), "Gy   :%+8.4f r/s", imu_sys.gy);
    ips200_show_string(0, ROW(5), buf);

    snprintf(buf, sizeof(buf), "Gz   :%+8.4f r/s", imu_sys.gz);
    ips200_show_string(0, ROW(6), buf);

    ips200_show_string(0, ROW(8), "[3]Back");
}

static void draw_pid_list(void)
{
    char buf[32];
    float vals[PID_ITEM_COUNT] = {GYRO_KP, ANG_KP, ANG_KI, ANG_KD};
    for (int8_t i = 0; i < PID_ITEM_COUNT; i++)
    {
        uint16 y = ROW(i + 1) + 4;
        snprintf(buf, sizeof(buf), "%s: %7.4f", s_pid_items[i], vals[i]);
        if (i == s_pid_sel)
        {
            ips200_set_color(COLOR_BLACK, COLOR_SELECTED);
            ips200_show_string(0, y, ">");
            ips200_show_string(FONT_W, y, buf);
            ips200_set_color(COLOR_WHITE, COLOR_BLACK);
        }
        else
        {
            ips200_set_color(COLOR_WHITE, COLOR_BLACK);
            ips200_show_string(0, y, " ");
            ips200_show_string(FONT_W, y, buf);
        }
    }
    ips200_show_string(0, ROW(6), "[0]Dn [1]Up [2]Edit");
    ips200_show_string(0, ROW(7), "[3]Back        ");
}

static void draw_pid_edit(void)
{
    char buf[32];
    float *pval = get_global_pid_var(s_pid_sel);
    float  val  = pval ? *pval : 0.0f;

    snprintf(buf, sizeof(buf), "Param: %s  ", s_pid_items[s_pid_sel]);
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);
    ips200_show_string(0, ROW(2), buf);

    snprintf(buf, sizeof(buf), "Value: %+9.4f", val);
    ips200_set_color(COLOR_YELLOW, COLOR_BLACK);
    ips200_show_string(0, ROW(3), buf);
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);

    snprintf(buf, sizeof(buf), "Step : %-10g", s_pid_steps[s_pid_sel]);
    ips200_show_string(0, ROW(4), buf);

    ips200_show_string(0, ROW(6), "[0]+  [1]-          ");
    ips200_show_string(0, ROW(7), "[2]Confirm&Apply    ");
    ips200_show_string(0, ROW(8), "[3]Cancel           ");
}

static void draw_speed_page(void)
{
    char buf[32];
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);

    int16_t spd_l = Motor_Get_Left_Speed();
    int16_t spd_r = Motor_Get_Right_Speed();

    snprintf(buf, sizeof(buf), "Left  Speed: %5d", spd_l);
    ips200_show_string(0, ROW(2), buf);

    snprintf(buf, sizeof(buf), "Right Speed: %5d", spd_r);
    ips200_show_string(0, ROW(3), buf);

    snprintf(buf, sizeof(buf), "Avg   Speed: %5d", (spd_l + spd_r) / 2);
    ips200_show_string(0, ROW(4), buf);

    ips200_show_string(0, ROW(8), "[3]Back");
}

static void draw_gps_page(void)
{
    char buf[32];
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);

    snprintf(buf, sizeof(buf), "Date:%04d-%02d-%02d", gnss.time.year, gnss.time.month, gnss.time.day);
    ips200_show_string(0, ROW(1), buf);

    snprintf(buf, sizeof(buf), "Time:%02d:%02d:%02d", gnss.time.hour, gnss.time.minute, gnss.time.second);
    ips200_show_string(0, ROW(2), buf);

    snprintf(buf, sizeof(buf), "Fix:%s Sat:%2d", gnss.state ? "OK" : "--", gnss.satellite_used);
    ips200_show_string(0, ROW(3), buf);

    snprintf(buf, sizeof(buf), "Lat:%+11.6f", gnss.latitude);
    ips200_show_string(0, ROW(4), buf);

    snprintf(buf, sizeof(buf), "Lon:%+11.6f", gnss.longitude);
    ips200_show_string(0, ROW(5), buf);

    snprintf(buf, sizeof(buf), "Spd:%5.2f Dir:%6.2f", gnss.speed, gnss.direction);
    ips200_show_string(0, ROW(6), buf);

    snprintf(buf, sizeof(buf), "Alt:%7.2f m", gnss.height);
    ips200_show_string(0, ROW(7), buf);

    ips200_show_string(0, ROW(9), "[3]Back");
}

// ─── 科目1 惯导子菜单绘制 ─────────────────────────────────────────────────
#define SUBJ1_ITEM_COUNT  2
static const char *s_subj1_items[SUBJ1_ITEM_COUNT] = {
    "Record Path",
    "Start Replay",
};

static const char *chassic_state_str(chassic_state_t st)
{
    switch (st)
    {
        case CHASSIC_IDLE:        return "IDLE   ";
        case CHASSIC_RECORDING:   return "REC... ";
        case CHASSIC_READY:       return "READY  ";
        case CHASSIC_STABILIZING: return "STAB.. ";
        case CHASSIC_REPLAYING:   return "PLAY.. ";
        case CHASSIC_COASTING:    return "COAST. ";
        case CHASSIC_DONE:        return "DONE   ";
        default:                  return "???    ";
    }
}

static void draw_subject1_page(void)
{
    char buf[32];
    chassic_state_t cst = Chassic_GetState();
    
    snprintf(buf, sizeof(buf), "State: %s", chassic_state_str(cst));
    ips200_set_color(COLOR_GREEN, COLOR_BLACK);
    ips200_show_string(0, ROW(1), buf);
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);

    for (int8_t i = 0; i < SUBJ1_ITEM_COUNT; i++)
    {
        uint16 y = ROW(i + 2) + 4;
        if (i == s_subj1_sel)
        {
            ips200_set_color(COLOR_BLACK, COLOR_SELECTED);
            ips200_show_string(0, y, ">");
            ips200_show_string(FONT_W, y, s_subj1_items[i]);
            ips200_set_color(COLOR_WHITE, COLOR_BLACK);
        }
        else
        {
            ips200_show_string(0, y, " ");
            ips200_show_string(FONT_W, y, s_subj1_items[i]);
        }
    }

    snprintf(buf, sizeof(buf), "Pts:%4d Dist:%6.1fcm", Chassic_GetRecordCount(), Chassic_GetRecordDistance());
    ips200_show_string(0, ROW(5), buf);

    if (cst == CHASSIC_RECORDING) {
        snprintf(buf, sizeof(buf), "Yaw:%+7.2f deg    ", imu_sys.yaw);
        ips200_show_string(0, ROW(6), buf);
    }
    else if (cst == CHASSIC_REPLAYING || cst == CHASSIC_COASTING || cst == CHASSIC_DONE) {
        snprintf(buf, sizeof(buf), "Progress: %3d%%    ", Chassic_GetReplayProgress());
        ips200_show_string(0, ROW(6), buf);
    }
    else {
        ips200_show_string(0, ROW(6), "                    ");
    }

    ips200_show_string(0, ROW(7), "[0]Dn [1]Up [2]OK  ");
    ips200_show_string(0, ROW(8), "[3]Back/Stop       ");
}

// ─── 科目2 定点排雷子菜单绘制 ─────────────────────────────────────────────────
#define SUBJ2_ITEM_COUNT  3
static const char *s_subj2_items[SUBJ2_ITEM_COUNT] = {
    "Record/Stop",
    "Mark Rotation",
    "Start Replay",
};

static const char *ms_state_str(ms_state_t st)
{
    switch (st)
    {
        case MS_IDLE:         return "IDLE   ";
        case MS_RECORDING:    return "REC... ";
        case MS_READY:        return "READY  ";
        case MS_STABILIZING:  return "STAB.. ";
        case MS_DRIVING:      return "DRIVE. ";
        case MS_SPIN_ALIGN:   return "SPIN.. ";
        case MS_COASTING:     return "COAST. ";
        case MS_DONE:         return "DONE   ";
        default:              return "???    ";
    }
}

static void draw_subject2_page(void)
{
    char buf[32];
    ms_state_t mst = Minesweep_GetState();
    
    snprintf(buf, sizeof(buf), "State: %s", ms_state_str(mst));
    ips200_set_color(COLOR_GREEN, COLOR_BLACK);
    ips200_show_string(0, ROW(1), buf);
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);

    for (int8_t i = 0; i < SUBJ2_ITEM_COUNT; i++)
    {
        uint16 y = ROW(i + 2) + 4;
        if (i == s_subj2_sel)
        {
            ips200_set_color(COLOR_BLACK, COLOR_SELECTED);
            ips200_show_string(0, y, ">");
            ips200_show_string(FONT_W, y, s_subj2_items[i]);
            ips200_set_color(COLOR_WHITE, COLOR_BLACK);
        }
        else
        {
            ips200_show_string(0, y, " ");
            ips200_show_string(FONT_W, y, s_subj2_items[i]);
        }
    }

    snprintf(buf, sizeof(buf), "Mk:%2d Dist:%6.1fcm ", Minesweep_GetMarkerCount(), Minesweep_GetRecordDistance());
    ips200_show_string(0, ROW(6), buf);

    if (mst == MS_RECORDING) {
        snprintf(buf, sizeof(buf), "Yaw:%+7.2f deg    ", imu_sys.yaw);
        ips200_show_string(0, ROW(7), buf);
    }
    else if (mst >= MS_DRIVING && mst <= MS_COASTING) {
        snprintf(buf, sizeof(buf), "Progress: %3d%%    ", Minesweep_GetReplayProgress());
        ips200_show_string(0, ROW(7), buf);
    }
    else if (mst == MS_DONE) {
        snprintf(buf, sizeof(buf), "Done! Markers: %2d  ", Minesweep_GetMarkerCount());
        ips200_show_string(0, ROW(7), buf);
    }
    else {
        ips200_show_string(0, ROW(7), "                    ");
    }

    ips200_show_string(0, ROW(8), "[0]Dn [1]Up [2]OK  ");
    ips200_show_string(0, ROW(9), "[3]Back/Stop       ");
}

// ─────────────────────────────────────────────────────────────────────────────
//  按键读取并清除 (底层驱动)
// ─────────────────────────────────────────────────────────────────────────────
static uint8 key_pressed(key_index_enum k)
{
    if (key_get_state(k) == KEY_SHORT_PRESS)
    {
        key_clear_state(k);
        return 1;
    }
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  公共接口
// ─────────────────────────────────────────────────────────────────────────────
void Menu_Init(void)
{
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);
    ips200_clear();
    s_main_sel     = 0;
    s_pid_sel      = 0;
    s_need_redraw  = 1;
#ifdef MENU_STARTUP_PID_TUNE
    s_state        = STATE_STARTUP_PID;
    s_startup_done = 0;
    draw_title("  PRE-TUNE PID  ");
    ips200_show_string(0, ROW(1), "[0]+  [1]-  [2]Next ");
#else
    s_state        = STATE_MAIN_MENU;
    s_startup_done = 1;
    draw_title("  MAIN MENU     ");
#endif
}

uint8 Menu_IsStartupDone(void)
{
    return s_startup_done;
}

void Menu_Process(void)
{
    // =========================================================================
    // 【核心优化】：按键快照 (Snapshot)
    // 在函数入口处，一次性抓取当前所有按键的状态并存储。
    // 这样无论后面的画屏函数 (draw_xxx) 耗时多久，都不会错过这个周期的按键脉冲。
    // =========================================================================
    uint8 btn1 = key_pressed(KEY_1); // 下 / +
    uint8 btn2 = key_pressed(KEY_2); // 上 / -
    uint8 btn3 = key_pressed(KEY_3); // 确认(OK) / Next
    uint8 btn4 = key_pressed(KEY_4); // 返回 / Stop

    // 动态页面（IMU/Speed）每约100ms刷新一次
    static uint32 refresh_tick = 0;
    uint8 do_refresh = 0;
    if (++refresh_tick >= 100) { refresh_tick = 0; do_refresh = 1; }

    switch (s_state)
    {
        // ──────────────────────────────────────────────
        case STATE_STARTUP_PID:
        {
#ifndef MENU_STARTUP_PID_TUNE
            s_state = STATE_MAIN_MENU;
            s_startup_done = 1;
            s_need_redraw = 1;
            break;
#endif
            if (s_need_redraw)
            {
                draw_pid_edit();
                char step_buf[24];
                snprintf(step_buf, sizeof(step_buf), "Step %d/4           ", s_pid_sel + 1);
                ips200_set_color(COLOR_YELLOW, COLOR_BLACK);
                ips200_show_string(0, ROW(0) + 2, step_buf);
                ips200_set_color(COLOR_WHITE, COLOR_BLACK);
                s_need_redraw = 0;
            }

            float *pval = get_global_pid_var(s_pid_sel);
            float  step = s_pid_steps[s_pid_sel];

            if (btn1 && pval)       // 增大
            {
                *pval += step;
                s_need_redraw = 1;
            }
            else if (btn2 && pval)  // 减小
            {
                *pval -= step;
                s_need_redraw = 1;
            }
            else if (btn3)          // 确认当前参数
            {
                if (pval) sync_pid_to_instance(s_pid_sel);
                printf("[Startup] %s=%.4f confirmed.\r\n", s_pid_items[s_pid_sel], pval ? *pval : 0.0f);

                if (s_pid_sel < PID_ITEM_COUNT - 1)
                {
                    s_pid_sel++;
                    ips200_clear();
                    draw_title("  PRE-TUNE PID  ");
                    s_need_redraw = 1;
                }
                else
                {
                    printf("[Startup] All params confirmed.\r\n");
                    s_startup_done = 1;
                    s_state = STATE_MAIN_MENU;
                    s_need_redraw = 1;
                }
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_MAIN_MENU:
        {
            if (s_need_redraw) { draw_main_menu(); s_need_redraw = 0; }

            if (btn1)      // 下
            {
                s_main_sel = (s_main_sel + 1) % MAIN_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (btn2) // 上
            {
                s_main_sel = (s_main_sel + MAIN_ITEM_COUNT - 1) % MAIN_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (btn3) // 确认
            {
                if (s_main_sel == 0)      { s_state = STATE_IMU;      ips200_clear(); draw_title("  IMU Data      "); }
                else if (s_main_sel == 1) { s_state = STATE_PID_LIST; s_pid_sel = 0; ips200_clear(); draw_title("  PID Params    "); }
                else if (s_main_sel == 2) { s_state = STATE_SPEED;    ips200_clear(); draw_title("  Speed         "); }
                else if (s_main_sel == 3) { s_state = STATE_GPS;      ips200_clear(); draw_title("  GPS Info      "); }
                else if (s_main_sel == 4) { s_state = STATE_SUBJECT1; s_subj1_sel = 0; ips200_clear(); draw_title(" Subject1 INS   "); }
                else if (s_main_sel == 5) { s_state = STATE_SUBJECT2; s_subj2_sel = 0; ips200_clear(); draw_title(" Subject2 Mine  "); }
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_IMU:
        {
            if (s_need_redraw || do_refresh) { draw_imu_page(); s_need_redraw = 0; }
            if (btn4) { s_state = STATE_MAIN_MENU; s_need_redraw = 1; }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_PID_LIST:
        {
            if (s_need_redraw) { draw_pid_list(); s_need_redraw = 0; }

            if (btn1)      { s_pid_sel = (s_pid_sel + 1) % PID_ITEM_COUNT; s_need_redraw = 1; }
            else if (btn2) { s_pid_sel = (s_pid_sel + PID_ITEM_COUNT - 1) % PID_ITEM_COUNT; s_need_redraw = 1; }
            else if (btn3) 
            {
                s_state = STATE_PID_EDIT;
                ips200_clear();
                draw_title("  Edit PID      ");
                s_need_redraw = 1;
            }
            else if (btn4) { s_state = STATE_MAIN_MENU; s_need_redraw = 1; }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_PID_EDIT:
        {
            if (s_need_redraw) { draw_pid_edit(); s_need_redraw = 0; }

            float *pval = get_global_pid_var(s_pid_sel);
            float  step = s_pid_steps[s_pid_sel];

            if (btn1 && pval)      { *pval += step; s_need_redraw = 1; }
            else if (btn2 && pval) { *pval -= step; s_need_redraw = 1; }
            else if (btn3)         
            {
                if (pval) sync_pid_to_instance(s_pid_sel);
                s_state = STATE_PID_LIST;
                ips200_clear();
                draw_title("  PID Params    ");
                s_need_redraw = 1;
            }
            else if (btn4)         
            {
                s_state = STATE_PID_LIST;
                ips200_clear();
                draw_title("  PID Params    ");
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_SPEED:
        {
            if (s_need_redraw || do_refresh) { draw_speed_page(); s_need_redraw = 0; }
            if (btn4) { s_state = STATE_MAIN_MENU; s_need_redraw = 1; }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_GPS:
        {
            GPS_Update();
            if (s_need_redraw || do_refresh) { draw_gps_page(); s_need_redraw = 0; }
            if (btn4) { s_state = STATE_MAIN_MENU; s_need_redraw = 1; }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_SUBJECT1:
        {
            if (s_need_redraw || do_refresh) { draw_subject1_page(); s_need_redraw = 0; }

            chassic_state_t cst = Chassic_GetState();

            if (btn1)      { s_subj1_sel = (s_subj1_sel + 1) % SUBJ1_ITEM_COUNT; s_need_redraw = 1; }
            else if (btn2) { s_subj1_sel = (s_subj1_sel + SUBJ1_ITEM_COUNT - 1) % SUBJ1_ITEM_COUNT; s_need_redraw = 1; }
            else if (btn3) 
            {
                if (s_subj1_sel == 0) 
                {
                    if (cst == CHASSIC_IDLE || cst == CHASSIC_DONE) Chassic_StartRecord();
                    else if (cst == CHASSIC_RECORDING) Chassic_StopRecord();
                }
                else if (s_subj1_sel == 1) 
                {
                    if (cst == CHASSIC_READY) Chassic_StartReplay();
                }
                s_need_redraw = 1;
            }
            else if (btn4) 
            {
                if (cst != CHASSIC_IDLE && cst != CHASSIC_DONE && cst != CHASSIC_READY) {
                    Chassic_Stop();
                }
                s_state = STATE_MAIN_MENU;
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_SUBJECT2:
        {
            if (s_need_redraw || do_refresh) { draw_subject2_page(); s_need_redraw = 0; }

            ms_state_t mst = Minesweep_GetState();

            if (btn1)      { s_subj2_sel = (s_subj2_sel + 1) % SUBJ2_ITEM_COUNT; s_need_redraw = 1; }
            else if (btn2) { s_subj2_sel = (s_subj2_sel + SUBJ2_ITEM_COUNT - 1) % SUBJ2_ITEM_COUNT; s_need_redraw = 1; }
            else if (btn3) 
            {
                if (s_subj2_sel == 0)  // Record/Stop
                {
                    if (mst == MS_IDLE || mst == MS_DONE) {
                        Minesweep_StartRecord();
                    }
                    else if (mst == MS_RECORDING) {
                        Minesweep_StopRecord();
                    }
                }
                else if (s_subj2_sel == 1)  // Mark Rotation
                {
                    if (mst == MS_RECORDING) {
                        Minesweep_MarkRotation();
                    }
                }
                else if (s_subj2_sel == 2)  // Start Replay
                {
                    if (mst == MS_READY) {
                        Minesweep_StartReplay();
                    }
                }
                s_need_redraw = 1;
            }
            else if (btn4) 
            {
                if (mst != MS_IDLE && mst != MS_DONE && mst != MS_READY) {
                    Minesweep_Stop(); // 紧急停车
                }
                s_state = STATE_MAIN_MENU;
                s_need_redraw = 1;
            }
            break;
        }
    }
}