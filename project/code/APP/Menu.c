/**
 * @file   Menu.c
 * @brief  串口屏菜单系统实现
 */

#include "Menu.h"
#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include "zf_device_gnss.h"
#include "IMU_Deal.h"
#include "Motor.h"
#include "controller.h"
#include "chassic.h"
#include "GPS.h"
#include "IPS200.h"
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
} menu_state_t;

// 主菜单条目
#define MAIN_ITEM_COUNT  5
static const char *s_main_items[MAIN_ITEM_COUNT] = {
    "1. IMU  Data",
    "2. PID  Params",
    "3. Speed",
    "4. GPS  Info",
    "5. Subject1",
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
static uint8_t      s_startup_done = 0;  // 启动阶段完成标志
#else
static menu_state_t s_state        = STATE_MAIN_MENU;
static uint8_t      s_startup_done = 1;  // 跳过预调，直接视为已完成
#endif
static int8_t       s_main_sel    = 0;   // 主菜单光标
static int8_t       s_pid_sel     = 0;   // PID 列表光标
static int8_t       s_subj1_sel   = 0;   // 科目1子菜单光标 (0=取点, 1=启动)
static uint8_t      s_need_redraw = 1;   // 强制刷屏标志

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
    for (uint16_t x = 0; x < 240; x++)
        for (uint16_t y = 0; y < FONT_H; y++)
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
        uint16_t y = ROW(i + 1) + 4;
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
    ips200_show_string(0, ROW(6), "[0]Dn [1]Up [2]OK");
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
    // 不调用 ips200_clear()，直接覆盖写各行，避免闪屏
    float vals[PID_ITEM_COUNT] = {GYRO_KP, ANG_KP, ANG_KI, ANG_KD};
    for (int8_t i = 0; i < PID_ITEM_COUNT; i++)
    {
        uint16_t y = ROW(i + 1) + 4;
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

    // 不调用 ips200_clear()，直接覆盖写各行，避免闪屏
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

    snprintf(buf, sizeof(buf), "Date:%04d-%02d-%02d",
             gnss.time.year, gnss.time.month, gnss.time.day);
    ips200_show_string(0, ROW(1), buf);

    snprintf(buf, sizeof(buf), "Time:%02d:%02d:%02d",
             gnss.time.hour, gnss.time.minute, gnss.time.second);
    ips200_show_string(0, ROW(2), buf);

    snprintf(buf, sizeof(buf), "Fix:%s Sat:%2d",
             gnss.state ? "OK" : "--", gnss.satellite_used);
    ips200_show_string(0, ROW(3), buf);

    snprintf(buf, sizeof(buf), "Lat:%+11.6f", gnss.latitude);
    ips200_show_string(0, ROW(4), buf);

    snprintf(buf, sizeof(buf), "Lon:%+11.6f", gnss.longitude);
    ips200_show_string(0, ROW(5), buf);

    snprintf(buf, sizeof(buf), "Spd:%5.2f Dir:%6.2f",
             gnss.speed, gnss.direction);
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
        case CHASSIC_DONE:        return "DONE   ";
        default:                  return "???    ";
    }
}

static void draw_subject1_page(void)
{
    char buf[32];

    // 状态信息
    chassic_state_t cst = Chassic_GetState();
    snprintf(buf, sizeof(buf), "State: %s", chassic_state_str(cst));
    ips200_set_color(COLOR_GREEN, COLOR_BLACK);
    ips200_show_string(0, ROW(1), buf);
    ips200_set_color(COLOR_WHITE, COLOR_BLACK);

    // 菜单条目
    for (int8_t i = 0; i < SUBJ1_ITEM_COUNT; i++)
    {
        uint16_t y = ROW(i + 2) + 4;
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

    // 数据行
    snprintf(buf, sizeof(buf), "Pts:%4d Dist:%6.1fcm",
             Chassic_GetRecordCount(), Chassic_GetRecordDistance());
    ips200_show_string(0, ROW(5), buf);

    if (cst == CHASSIC_RECORDING)
    {
        snprintf(buf, sizeof(buf), "Yaw:%+7.2f deg    ", imu_sys.yaw);
        ips200_show_string(0, ROW(6), buf);
    }
    else if (cst == CHASSIC_REPLAYING || cst == CHASSIC_DONE)
    {
        snprintf(buf, sizeof(buf), "Progress: %3d%%    ", Chassic_GetReplayProgress());
        ips200_show_string(0, ROW(6), buf);
    }
    else
    {
        ips200_show_string(0, ROW(6), "                    ");
    }

    ips200_show_string(0, ROW(7), "[0]Dn [1]Up [2]OK  ");
    ips200_show_string(0, ROW(8), "[3]Back/Stop       ");
}

// ─────────────────────────────────────────────────────────────────────────────
//  按键读取并清除 (防止重复触发)
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t key_pressed(key_index_enum k)
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
    // 启动页：直接画标题，正文由 draw_pid_edit 覆盖写
    draw_title("  PRE-TUNE PID  ");
    ips200_show_string(0, ROW(1), "[0]+  [1]-  [2]Next ");
#else
    s_state        = STATE_MAIN_MENU;
    s_startup_done = 1;
    draw_title("  MAIN MENU     ");
#endif
}

uint8_t Menu_IsStartupDone(void)
{
    return s_startup_done;
}

void Menu_Process(void)
{
    // 动态页面（IMU/Speed）每约100ms刷新一次
    static uint32_t refresh_tick = 0;
    uint8_t do_refresh = 0;
    if (++refresh_tick >= 100) { refresh_tick = 0; do_refresh = 1; }

    switch (s_state)
    {
        // ──────────────────────────────────────────────
        // 启动预调 PID 阶段：顺序逐一确认全部4个参数，全部确认后才开始运行
        case STATE_STARTUP_PID:
        {
#ifndef MENU_STARTUP_PID_TUNE
            // 宏未定义时不应进入此分支，直接跳主菜单
            s_state = STATE_MAIN_MENU;
            s_startup_done = 1;
            s_need_redraw = 1;
            break;
#endif
            if (s_need_redraw)
            {
                draw_pid_edit();
                // 顶部提示当前是第几步
                char step_buf[24];
                snprintf(step_buf, sizeof(step_buf), "Step %d/4           ", s_pid_sel + 1);
                ips200_set_color(COLOR_YELLOW, COLOR_BLACK);
                ips200_show_string(0, ROW(0) + 2, step_buf);
                ips200_set_color(COLOR_WHITE, COLOR_BLACK);
                s_need_redraw = 0;
            }

            float *pval = get_global_pid_var(s_pid_sel);
            float  step = s_pid_steps[s_pid_sel];

            if (key_pressed(KEY_1) && pval)       // 增大 (原下)
            {
                *pval += step;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_2) && pval)  // 减小 (原上)
            {
                *pval -= step;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_3))          // 确认当前参数，前进到下一个
            {
                if (pval) sync_pid_to_instance(s_pid_sel);
                printf("[Startup] %s=%.4f confirmed.\r\n",
                       s_pid_items[s_pid_sel], pval ? *pval : 0.0f);

                if (s_pid_sel < PID_ITEM_COUNT - 1)
                {
                    // 还有下一个参数，继续
                    s_pid_sel++;
                    ips200_clear();
                    draw_title("  PRE-TUNE PID  ");
                    s_need_redraw = 1;
                }
                else
                {
                    // 全部4个参数都确认完毕，开始运行
                    printf("[Startup] All params confirmed. Starting control.\r\n");
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

            if (key_pressed(KEY_1))      // 下 (原上)
            {
                s_main_sel = (s_main_sel + 1) % MAIN_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_2)) // 上 (原下)
            {
                s_main_sel = (s_main_sel + MAIN_ITEM_COUNT - 1) % MAIN_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_3)) // 确认
            {
                if (s_main_sel == 0)
                {
                    s_state = STATE_IMU;
                    ips200_clear();
                    draw_title("  IMU Data      ");
                }
                else if (s_main_sel == 1)
                {
                    s_state = STATE_PID_LIST;
                    s_pid_sel = 0;
                    // 进入PID列表时清屏一次并画标题，之后靠覆盖写
                    ips200_clear();
                    draw_title("  PID Params    ");
                }
                else if (s_main_sel == 2)
                {
                    s_state = STATE_SPEED;
                    ips200_clear();
                    draw_title("  Speed         ");
                }
                else if (s_main_sel == 3)
                {
                    s_state = STATE_GPS;
                    ips200_clear();
                    draw_title("  GPS Info      ");
                }
                else if (s_main_sel == 4)
                {
                    s_state = STATE_SUBJECT1;
                    s_subj1_sel = 0;
                    ips200_clear();
                    draw_title(" Subject1 INS   ");
                }
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_IMU:
        {
            if (s_need_redraw || do_refresh) { draw_imu_page(); s_need_redraw = 0; }

            if (key_pressed(KEY_4))
            {
                s_state = STATE_MAIN_MENU;
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_PID_LIST:
        {
            // 只在 s_need_redraw 时重绘，不走 do_refresh（静态页面，无需定时刷新）
            if (s_need_redraw) { draw_pid_list(); s_need_redraw = 0; }

            if (key_pressed(KEY_1))
            {
                s_pid_sel = (s_pid_sel + 1) % PID_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_2))
            {
                s_pid_sel = (s_pid_sel + PID_ITEM_COUNT - 1) % PID_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_3))  // 进入编辑
            {
                s_state = STATE_PID_EDIT;
                // 进入编辑时清屏一次并画标题，之后靠覆盖写
                ips200_clear();
                draw_title("  Edit PID      ");
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_4))  // 返回主菜单
            {
                s_state = STATE_MAIN_MENU;
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_PID_EDIT:
        {
            // 只在 s_need_redraw 时重绘（覆盖写，不闪）
            if (s_need_redraw) { draw_pid_edit(); s_need_redraw = 0; }

            float *pval = get_global_pid_var(s_pid_sel);
            float  step = s_pid_steps[s_pid_sel];

            if (key_pressed(KEY_1) && pval)       // 增大 (原下)
            {
                *pval += step;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_2) && pval)  // 减小 (原上)
            {
                *pval -= step;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_3))           // 确认写入单片机
            {
                if (pval) sync_pid_to_instance(s_pid_sel);
                printf("[Menu] %s set to %.4f\r\n", s_pid_items[s_pid_sel],
                       pval ? *pval : 0.0f);
                s_state = STATE_PID_LIST;
                // 返回列表时清屏一次并画标题
                ips200_clear();
                draw_title("  PID Params    ");
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_4))           // 取消，不写入
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

            if (key_pressed(KEY_4))
            {
                s_state = STATE_MAIN_MENU;
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_GPS:
        {
            GPS_Update();
            if (s_need_redraw || do_refresh) { draw_gps_page(); s_need_redraw = 0; }

            if (key_pressed(KEY_4))
            {
                s_state = STATE_MAIN_MENU;
                s_need_redraw = 1;
            }
            break;
        }

        // ──────────────────────────────────────────────
        case STATE_SUBJECT1:
        {
            if (s_need_redraw || do_refresh) { draw_subject1_page(); s_need_redraw = 0; }

            chassic_state_t cst = Chassic_GetState();

            if (key_pressed(KEY_1))      // 下 (原上)
            {
                s_subj1_sel = (s_subj1_sel + 1) % SUBJ1_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_2)) // 上 (原下)
            {
                s_subj1_sel = (s_subj1_sel + SUBJ1_ITEM_COUNT - 1) % SUBJ1_ITEM_COUNT;
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_3)) // 确认
            {
                if (s_subj1_sel == 0)  // Record Path
                {
                    if (cst == CHASSIC_IDLE || cst == CHASSIC_DONE)
                    {
                        Chassic_StartRecord();
                    }
                    else if (cst == CHASSIC_RECORDING)
                    {
                        Chassic_StopRecord();
                    }
                }
                else if (s_subj1_sel == 1)  // Start Replay
                {
                    if (cst == CHASSIC_READY)
                    {
                        Chassic_StartReplay();
                    }
                }
                s_need_redraw = 1;
            }
            else if (key_pressed(KEY_4)) // 返回 / 停止
            {
                if (cst == CHASSIC_RECORDING || cst == CHASSIC_STABILIZING ||
                    cst == CHASSIC_REPLAYING)
                {
                    Chassic_Stop();
                }
                s_state = STATE_MAIN_MENU;
                s_need_redraw = 1;
            }
            break;
        }
    }
}
