#include "manual_control.h"
#include "project_config.h"
#include "keypad.h"
#include "stepper.h"
#include "task_scheduler.h"

/*====================================================================*/
/* 手动控制模式                                                        */
/* 按键功能 (实测ADC对齐):                                             */
/*   S1=X轴正转  S2=X轴反转   S3=电磁阀1  S4=电磁阀2                   */
/*   S5=加湿阀  S6=电磁阀3  S7=OLED下翻页  S8=OLED上翻页              */
/*   S9=手动模式开关(toggle)  S10=Z轴正转  S11=Z轴反转               */
/*   S12=弃用                                                         */
/*====================================================================*/

volatile uint8_t g_manual_active = 0;

/*====================================================================*/
/* 执行器切换函数 (电磁阀/加湿阀)                                       */
/*====================================================================*/
static void toggle_valve(uint8_t idx)
{
    switch (idx) {
    case 0: ACT_TOGGLE(VALVE1); break;
    case 1: ACT_TOGGLE(VALVE2); break;
    case 2: ACT_TOGGLE(VALVE3); break;
    }
}

static void toggle_humidifier(void)
{
    ACT_TOGGLE(HUMIDIFIER);
}

/*====================================================================*/
/* API                                                                 */
/*====================================================================*/

void MANUAL_Init(void)
{
    g_manual_active = 0;
}

void MANUAL_Scan(void)
{
    KeyCode_t key = KEYPAD_Scan();   // 已消抖, 稳定输出
    static KeyCode_t prev = KEY_NONE;

    /* 边沿检测: 出现一个与上次不同的新有效键 */
    uint8_t is_new_press = (key != KEY_NONE && key != prev);

    /* ---- S9: 手动模式开关 (按一下 toggle) ---- */
    if (is_new_press && key == KEY_S9) {
        g_manual_active = !g_manual_active;
        STEPPER_Stop(&g_x_motor);
        STEPPER_Stop(&g_z_motor);
    }

    /* ---- 手动模式: 电磁阀/加湿阀 (新按下 toggle 一次) ---- */
    if (is_new_press && g_manual_active) {
        switch (key) {
        case KEY_S3: toggle_valve(0); break;     // 电磁阀1
        case KEY_S4: toggle_valve(1); break;     // 电磁阀2
        case KEY_S5: toggle_humidifier(); break; // 加湿阀
        case KEY_S6: toggle_valve(2); break;     // 电磁阀3
        default: break;
        }
    }

    /* ---- S7/S8 翻页 (任何模式, 新按下一次) ---- */
    if (is_new_press) {
        if (key == KEY_S7) { extern void OLED_PrevPage(void); OLED_PrevPage(); }
        if (key == KEY_S8) { extern void OLED_NextPage(void); OLED_NextPage(); }
    }

    prev = key;

    /* ---- 手动模式: 电机点动 (按住转, 松开停) ---- */
    if (g_manual_active) {
        switch (key) {
        case KEY_S1:    // X轴电机正转
            STEPPER_SetDirection(&g_x_motor, 1);
            STEPPER_Start(&g_x_motor, 1, 0);
            break;
        case KEY_S2:    // X轴电机反转
            STEPPER_SetDirection(&g_x_motor, 0);
            STEPPER_Start(&g_x_motor, 0, 0);
            break;
        case KEY_S10:   // Z轴电机正转
            STEPPER_SetDirection(&g_z_motor, 1);
            STEPPER_Start(&g_z_motor, 1, 0);
            break;
        case KEY_S11:   // Z轴电机反转
            STEPPER_SetDirection(&g_z_motor, 0);
            STEPPER_Start(&g_z_motor, 0, 0);
            break;
        default:        // 非电机键 => 停
            STEPPER_Stop(&g_x_motor);
            STEPPER_Stop(&g_z_motor);
            break;
        }
    }
}
