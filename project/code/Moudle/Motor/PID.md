1.15

* 中高度下的pid参数1
![alt text](image.png)

// 1. 最内环：角速度环 (输入: rad/s, 输出: PWM)
// 只有这一环直接控制电机，Kp决定了电机对旋转速度的响应快慢
#define GYRO_KP   480.0f   // 典型值：根据 PWM 量程调整        
#define GYRO_KI   0.0f     // 也就是图片中的 "角速度环 PD"
#define GYRO_KD   0.0f     // 抑制高频噪声

// 2. 中间环：角度环 (输入: 角度/弧度, 输出: 目标角速度 rad/s)
// 这一环决定了车回正的"欲望"强弱，输出的是给内环的指令
#define ANG_KP    1.5f     // 典型值：输出是 rad/s，所以不会很大    
#define ANG_KI    0.05f     //
#define ANG_KD    0.01f     // 

参数2
// 1. 最内环：角速度环 (输入: rad/s, 输出: PWM)
// 只有这一环直接控制电机，Kp决定了电机对旋转速度的响应快慢
#define GYRO_KP   380.0f   // 典型值：根据 PWM 量程调整         480 580
#define GYRO_KI   0.0f     // 也就是图片中的 "角速度环 PD"
#define GYRO_KD   0.0f     // 抑制高频噪声

// 2. 中间环：角度环 (输入: 角度/弧度, 输出: 目标角速度 rad/s)
// 这一环决定了车回正的"欲望"强弱，输出的是给内环的指令
#define ANG_KP    1.2f     // 典型值：输出是 rad/s，所以不会很大    1.0 1.4 1.5/1.2
#define ANG_KI    0.04f     //0.55  0.04
#define ANG_KD    0.09f     // 通常设为0，因为微分作用由内环(角速度)承担了 // 0.01//1.88 0.09

// 3. 最外环：速度环 (输入: 速度, 输出: 目标角度)
#define SPD_KP    0.10f   //非常小
#define SPD_KI    0.001f  // 积分也要很小
#define SPD_KD    0.0f     // 速度环通常不需要 D
#define SPD_MAX_PITCH  12.0f // 速度环最大允许输出多少度倾角安全限制

// 4. 转向环参数 (输入: 摇杆差值, 输出: PWM差分)
#define TURN_KP 1.0f
#define TURN_KI 0.0f
#define TURN_KD 0.05f

// 5. 手柄映射配置
#define JOYSTICK_DEADZONE 2000   // 摇杆死区 (防止漂移)
#define MAX_TARGET_SPEED  5000.0f // 满油门时的最大编码器速度
#define MAX_TURN_SPEED    2000.0f // 满舵时的最大转向PWM分量


// ===== 经过地狱越野路况淬炼的最优参数 =====
#define GYRO_KP   200.00f
#define GYRO_KI   0.0f
#define GYRO_KD   0.0f

#define ANG_KP    0.80f
#define ANG_KI    0.0200f
#define ANG_KD    0.0102f

#define SPD_KP    0.0855f
#define SPD_KI    0.0016f
#define SPD_KD    0.0f


// ===== 中等越野工况推荐参数 =====
#define GYRO_KP   450.00f

#define ANG_KP    1.40f
#define ANG_KI    0.0450f
#define ANG_KD    0.0500f

#define SPD_KP    0.1000f
#define SPD_KI    0.0010f
// ===============================================