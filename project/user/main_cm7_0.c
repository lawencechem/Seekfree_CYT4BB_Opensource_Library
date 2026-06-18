
/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "filter.h"
#include "quaternion.h"
#include "mpu660ra.h"      // current_euler(欧拉角)在此声明，打印 YAW/PIT/ROL 需要
#include "pid.h"
#include "dl1b_altitude.h"
#include "battery_comp.h"
#include "UP_FLOW_302.h"
#include "cam_share.h"
#include "math.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设


// **************************** 代码区域 ****************************

float global_throttle = 0; // 全局油门，由你后续的控制逻辑修改
float comp_throttle = 0;   // 调试显示用：当前最终送入姿态混控的油门
volatile uint32_t sys_time_ms = 0; // 【定高调试】全局毫秒级时间戳

 int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    SCB_DisableDCache();                // ★关D-cache：核间IPC要求两核都关
    Cam_IPC_Init();                     // ★IPC尽早初始化
    // 此处编写用户代码 例如外设初始化代码等
    imu660ra_init();
    wireless_uart_init();
    small_driver_uart_init();
    PID_Params_Init();                  // 姿态 PID 模式：启用 PID 参数
    
    // --- 电调唤醒与自检延时---
    global_throttle = 0;                // 确保初始油门为 0
    small_driver_set_duty(0, 0, 0, 0); 
    
    system_delay_ms(500);              // 关键：延时 1 秒，等电调“滴滴”自检声结束 
    
    dl1b_init();              //初始化 DL1B TOF
    Altitude_System_Init();   // 定高模式：初始化高度位置环和速度环 PID
    Battery_Comp_Init();  // 定高模式：启用电压采样/补偿
    Up_Flow_302_Init();       // 光流模块初始化：UART 接收 + 应用层速度/PI 状态清零
    flight_state = ALT_IDLE;          // 校准期间保持 IDLE，禁止混控输出电机
    
    pit_ms_init(PIT_CH0,2);
    
    // 校准期间飞机务必绝对静止；若仍有持续偏置，可再延长到 3000ms
    Attitude_Zero_Calibrate(2000);   // 飞机静止放平，采样 2 秒作为姿态零点

    // === 基础姿态 + 定高 + 光流弱速度阻尼 ===
    flight_state = ALT_TAKEOFF;
   
    system_delay_ms(100);               // 等待 PID 环路稳定
    global_throttle = 0;                 // 定高模式下固定油门不参与控制，油门由 throttle_output 给出
    // global_throttle = 0;               // 静态陀螺仪测试：油门保持 0
    
    comp_throttle = throttle_output;
    uint32_t last_time = sys_time_ms;
    uint8_t print_div = 0;  //打印分频计数器，给 printf 降频

    // 跟随就绪状态：先爬到目标高度并稳住，再开始跟车
    float   follow_settle_s = 0.0f;   // "已到目标高度且垂直稳定"持续了多久(秒)
    uint8_t follow_latched  = 0;      // 1=已就绪(锁存)，开始发跟车速度指令
    static float corr_engage = 0.0f;  // 速度环切入平滑因子：FA恢复时渐入防偏航耦合

    
    while(true)
    {
        // 此处编写需要循环执行的代码 
      uint32_t current_time = sys_time_ms;
     
      if((current_time - last_time) >= 10) 
        {
            float dt = (current_time - last_time) / 1000.0f;
            last_time = current_time;
            if(dt < 0.005f) dt = 0.010f;
            if(dt > 0.030f) dt = 0.030f;  // 给 dt 加保护限幅，避免偶发大周期把速度估计拉爆
            dl1b_get_distance();          // 定高模式：发起/读取 DL1B 测距
            uint8 tof_has_new = dl1b_finsh_flag;
            dl1b_finsh_flag = 0;
            // 定高任务固定周期执行，TOF 新帧标志仅用于决定是否更新观测值
            Battery_Comp_Task_10ms();    // 10ms 更新一次电压估计，供定高油门补偿使用
            Altitude_Control_Task(dt, tof_has_new);
            comp_throttle = throttle_output; // 调试变量：当前最终送入姿态混控的油门

            Up_Flow_302_Update(current_height_cm);
            Cam_IPC_Process(current_height_cm);   // 收CM7_1视觉→换算cm→写cam_rel
            /* 爬升时光流测速本身受旋转残留污染，不可靠。
             * 50cm/s 门限：悬停(Vz<5)正常，爬升(Vz=15-48)关速度环，
             * 到顶Vz降回后自动恢复。积分冻结(D<12)防恢复时冲击。 */
            uint8_t flow_active = (current_height_cm > FLOW_START_HEIGHT_CM &&
                                   upf_data_fresh &&
                                   fabsf(current_speed_z) < 50.0f) ? 1U : 0U;
            float flow_weight = 0.0f;

            if (FLOW_HOLD_ENABLE && FLOW_VEL_DAMP_ENABLE && flow_active)
            {
#if FOLLOW_STEP == 1
                // 阶段1：起飞/悬停【位置保持】。目标=起飞点(原点)。
                Up_Flow_302_PosHold_Update(dt);
                (void)follow_settle_s;
                (void)follow_latched;
#elif FOLLOW_STEP == 4
                // 第4步：真实摄像头定点/跟车。
                // 首次到达1m时一次性锁存摄像头位置环，之后永不关闭。
                // 1m以下：光流+陀螺仪速度阻尼；1m以上摄像头介入。
                // 锁存后高度再掉回1m以下位置环也不关，防反复切入切出。
                {
                    static uint8_t cam_latched = 0;
                    if (!cam_latched && cam_valid && current_height_cm >= CAM_ENGAGE_HEIGHT_CM)
                    {
                        cam_latched = 1;
                        Up_Flow_302_PosHold_Prime();
                    }
                    if (cam_latched || (cam_valid && current_height_cm >= CAM_ENGAGE_HEIGHT_CM))
                    {
                        Cam_Follow_Outer_Update(dt);
                    }
                    else
                    {
                        upf_target_vx = 0.0f;
                        upf_target_vy = 0.0f;
                    }
                }
                (void)follow_settle_s;
                (void)follow_latched;
#else
                // ---- 跟随就绪判定：先爬到目标高度并稳住，再开始跟车 ----
                // 到达目标高度(±band) 且 垂直速度足够小 → 算"已稳定"
                uint8_t at_target = (fabsf(current_height_cm - ALT_HOLD_TARGET_CM) < FOLLOW_HEIGHT_BAND_CM)
                                 && (fabsf(current_speed_z) < FOLLOW_VZ_SETTLE_CMS);
                if (!follow_latched)
                {
                    if (at_target) follow_settle_s += dt;   // 稳定累计计时
                    else           follow_settle_s = 0.0f;  // 一旦不满足就清零重数
                    if (follow_settle_s >= FOLLOW_SETTLE_S) follow_latched = 1;  // 就绪，锁存
                }

                if (follow_latched)
                {
                    // 已就绪(到目标高度并稳住3s) → 按当前调试步骤决定做什么
#if   FOLLOW_STEP == 2
                    Cam_Vel_Mock_Update(dt);   // 第2步：发速度指令(前进/后退循环)，验证内环跟踪
#elif FOLLOW_STEP == 3
                    // 第3步：位置跟随。数据源=虚拟车mock(真相机时只换这一行)，外环不变。
                    Cam_Pos_Mock_Update(dt);      // (A)算相对坐标 cam_rel_x/y
                    Cam_Follow_Outer_Update(dt);  // (B)外环:相对坐标→期望速度
#endif
                }
                else
                {
                    // 还没就绪(爬升中/没稳) → 用位置保持把起飞点定住(复用C2)，爬升不漂；
                    // 到顶稳住锁存后再切到跟车。cam_drone 此时仍为0，切跟车时从干净原点开始。
                    Up_Flow_302_PosHold_Update(dt);
                }
#endif /* FOLLOW_STEP == 1 */

                Up_Flow_302_Speed_Damp(dt, &upf_pitch_corr, &upf_roll_corr);

                flow_weight = (current_height_cm - FLOW_START_HEIGHT_CM) /
                              (FLOW_FULL_HEIGHT_CM - FLOW_START_HEIGHT_CM);
                flow_weight = f_limit(flow_weight, 0.0f, 1.0f);

                /* 切入平滑：FA恢复时修正角从0渐变→全量，防止瞬态偏航耦合 */
                corr_engage += 0.02f * (1.0f - corr_engage);  // ~500ms渐入
                upf_pitch_corr *= flow_weight * corr_engage;
                upf_roll_corr  *= flow_weight * corr_engage;
            }
            else
            {
                // FA退出：修正角渐变衰减（每10ms×0.6，~40ms归零），不放电源线荡起
                corr_engage = 0.0f;  // 下次切入从0渐入
                upf_pitch_corr *= 0.6f;
                upf_roll_corr  *= 0.6f;
                // 离开定高/低高度 → 复位位置环/跟车状态，下次以当前点为新原点
#if FOLLOW_STEP == 1
                Up_Flow_302_PosHold_Reset();   // 阶段1：位移估计与目标速度清零，重新定原点
#else
                Cam_Vel_Mock_Reset();          // 复位虚拟车 mock 位移/计时
                Up_Flow_302_PosHold_Reset();   // 爬升段用到位置保持，一并复位
#endif
                follow_settle_s = 0.0f;
                follow_latched  = 0;
            }

            print_div++;
            if (print_div >= 5U)   // 20Hz：符号已验证，恢复常规打印频率
            {
                print_div = 0;

//                printf("STATE:%d TK:%d ABT:%d TW:%.2f | "
//                       "H:%.1f/%.1f Vz:%.1f | "
//                       "THR base:%.0f alt:%.0f out:%.0f cmp:%.0f | "
//                       "VB:%.2f VS:%.3f | "
//                       "ATT P:%.2f R:%.2f Y:%.2f | "
//                       "TAR P:%.2f R:%.2f Y:%.2f | "
//                       "CMD P:%.2f R:%.2f Y:%.2f | "
//                       "OUT P:%.0f R:%.0f Y:%.0f YI:%.2f YIE:%d YB:%.1f YBM:%.1f YW:%.2f | "
//                       "M:%d,%d,%d,%d\r\n",
//                       flight_state,
//                       dbg_takeoff_state,
//                       dbg_takeoff_abort,
//                       dbg_takeoff_trim_weight,
//                       current_height_cm,
//                       target_height_cm,
//                       current_speed_z,
//                       dbg_thr_base,
//                       dbg_thr_alt,
//                       throttle_output,
//                       comp_throttle,
//                       Battery_Comp_Get_Voltage(),
//                       Battery_Comp_Get_Scale(),
//                       current_euler.pitch,
//                       current_euler.roll,
//                       current_euler.yaw,
//                       pitch_target,
//                       roll_target,
//                       target_yaw,
//                       p_target_rate,
//                       r_target_rate_dbg,
//                       y_target_rate_dbg,
//                       p_out,
//                       r_out,
//                       y_limited,
//                       pid_yaw_rate.integral,
//                       yaw_i_enable,
//                       yaw_bias_adapt,
//                       dbg_yaw_bias_for_mix,
//                       dbg_yaw_takeoff_weight,
//                       dbg_m1_target,
//                       dbg_m2_target,
//                       dbg_m3_target,
//                       dbg_m4_target);

                // tvx/tvy = 期望速度(指令)，vx/vy = 实测速度。阶段1重点对比这两组：
                // 指令前进时 vx 应跟着上去、指令归零时 vx 应回到 0，且不振荡。
                // rx/ry=车相对飞机坐标(第3步用,第1/2步为0)，tvx/tvy=期望速度，vx/vy=实测速度
                printf("UPF FA:%d valid:%d fresh:%d H:%.1f Vz:%.1f "
                       "rx:%.1f ry:%.1f tvx:%.1f tvy:%.1f vx:%.2f vy:%.2f fx:%.1f fy:%.1f imx:%.1f imy:%.1f "
                       "fw:%.2f pc:%.2f rc:%.2f evx:%.1f evy:%.1f op:%.2f or:%.2f ivx:%.1f ivy:%.1f "
                       "ROL:%.1f PIT:%.1f "
                       "TGTY:%.1f YR:%.1f YRC:%.1f YL:%.0f YI:%.1f cv:%d cx:%.1f cy:%.1f u:%d v:%d ar:%d mg:%d rx:%u "
                       "PO:%.0f RO:%.0f YO:%.1f PR:%.1f RR:%.1f\r\n",
                       flow_active,
                       upf_data_valid,
                       upf_data_fresh,
                       current_height_cm,
                       current_speed_z,
                       cam_rel_x,
                       cam_rel_y,
                       upf_target_vx,
                       upf_target_vy,
                       upf_vel_x,
                       upf_vel_y,
                       upf_fused_vx,
                       upf_fused_vy,
                       upf_imu_vx,
                       upf_imu_vy,
                       flow_weight,
                       upf_pitch_corr,
                       upf_roll_corr,
                       upf_dbg_err_vx,
                       upf_dbg_err_vy,
                       upf_dbg_raw_out_pitch,
                       upf_dbg_raw_out_roll,
                       pid_upf_vx.integral,
                       pid_upf_vy.integral,
                       current_euler.roll,     // ROL：实际横滚角
                       current_euler.pitch,    // PIT：实际俯仰角
                       target_yaw,             // TGTY：锁定的目标航向
                       dbg_yaw_rate_fb,        // YR：实测偏航角速度
                       y_target_rate_dbg,      // YRC：目标偏航角速度
                       y_limited,              // YL：混控偏航量
                       pid_yaw_rate.integral,  // YI：偏航积分
                       cam_valid,              // cv：摄像头锁定
                       cam_dbg_x,              // cx：换算后X(cm)
                       cam_dbg_y,              // cy：换算后Y(cm)
                       cam_dbg_u,              // u：原始像素u
                       cam_dbg_v,              // v：原始像素v
                       cam_dbg_area,           // ar：亮点数
                       (unsigned int)cam_dbg_maxg, // mg：最大灰度
                       (unsigned int)cam_dbg_rx_cnt,  // rx：IPC计数
                       // PO/RO=内环最终混控值，YO=yaw限幅后输出，PR/RR=角度环目标角速度
                       p_out, r_out, y_limited,
                       p_target_rate, r_target_rate_dbg);
            }

        }

        system_delay_ms(5);
    }
}

void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH0);
    sys_time_ms += 2;
  
    //imu660ra_get_acc();//原始数据
    imu660ra_get_gyro();//原始数据
    
    //1. 获取最新原始数据 (带 offset 补偿的)
   
     IMU660RA_Get_Gyro(&last_gyro);//零点校准后数据
    IMU660RA_Get_Acc(&last_accel);//零点校准后数据
    // 2. 调用滤波函数，更新 filtered_gyro
    Update_Gyro_Filter(&last_gyro);
    Update_Accel_KalmanFilter(&last_accel);
    // 【修复点】：必须把滤波后的值赋给 filtered_data，否则解算函数拿到的全是 0
    
    // 这里的 filtered_gyro 和 filtered_accel 是你在 filter.c 里定义的变量
    filtered_data.gyro = filtered_gyro;
    filtered_data.accel = filtered_accel;
    
    // 3. 现在 filtered_data 有值了，解算才能成功
    Common_IMU_GetEulerAngle(&filtered_data, &current_euler, 0.002f);

    // 3.5 累积陀螺仪角位移（V2角度域旋转补偿用）
    Up_Flow_302_Gyro_Accum_2ms();
    
    // 4. 执行 PID 混控与电机输出
    Motor_Control_Mixing(throttle_output);       // 飞行模式：使用高度环输出的最终油门
}

// **************************** 代码区域 ****************************
