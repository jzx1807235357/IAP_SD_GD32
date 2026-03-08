#include "optical_tracking.h"

#include "light_sensor.h"
#include "sun_position.h"
#include "gimbal_control.h"
#include "gps_nmea.h"
#include "modbus_rtu.h"
#include "app_rtc.h"
#include "flash_config.h"
#include "systick.h"
#include "data_hub.h"    /* 引入data_hub以使用统一天文解算结果 */
#include "host_comm.h"   /* 引入host_comm以使用统一校准接口 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <math.h>
#include <string.h>

/* 控制参数 */
/* 步距与切换阈值 */
#define OPTICAL_STEP_SMALL_DEG      0.1f   /* 小步距 */
#define OPTICAL_STEP_LARGE_DEG      1.0f   /* 大步距 */
#define OPTICAL_STEP_SWITCH_ERR     0.4f   /* 误差绝对值 >0.4 用大步距 */

/* 兼容原来名字（如果还有地方用到） */
#define OPTICAL_STEP_SIZE_DEG       OPTICAL_STEP_SMALL_DEG

#define OPTICAL_SETTLE_MS            200U    /* 云台稳定时间：每次发命令后的冷却时间 */
#define OPTICAL_AXIS_LOCK_THRESHOLD  0.05f   /* 单轴锁定阈值：误差小于此值认为该轴已精调到位 */

/* FSM状态定义 */
typedef enum {
    OPT_FSM_IDLE = 0,        /* 未启动 / 模式非光追 */
    OPT_FSM_MEASURE,         /* 读取传感器 & 判死区 & 选轴 & 发送命令 */
    OPT_FSM_CALIBRATE        /* 刚刚进入死区，做一次校准 */
} opt_fsm_state_t;

/*!
    \brief    光学追踪控制上下文结构体
    \detail   维护光学追踪的完整状态，包括FSM状态、位置信息、误差数据和优化特性
              本结构体采用单例模式，全局只有一个实例 g
*/
typedef struct {
    /* ========== 基础状态 ========== */
    optical_control_mode_t mode;     /**< 控制模式：DISABLED/OPTICAL_TRACKING/ASTRONOMICAL_TRACKING */
    opt_fsm_state_t fsm;             /**< FSM当前状态：IDLE/MEASURE/CALIBRATE */
    bool initialized;                /**< 模块是否已初始化（调用optical_tracking_init后为true） */
    bool in_deadband;                /**< 是否在死区内（true=对准，false=需调整） */

    /* ========== 位置信息 ========== */
    float base_azimuth;              /**< 基准方位角（启动时的初始位置，单位：度，范围：0-360） */
    float base_elevation;            /**< 基准仰角（启动时的初始位置，单位：度，范围：0-90） */

    /* ========== 传感器误差 ========== */
    float ex;                        /**< X轴归一化误差（左右偏差，范围：-1.0~1.0，正值=向右偏） */
    float ey;                        /**< Y轴归一化误差（上下偏差，范围：-1.0~1.0，正值=向上偏） */

    /* ========== 控制时序 ========== */
    uint32_t last_position_set_time; /**< 上次发送云台命令的时间戳（systick，用于冷却时间判定） */
    
    /* ========== 优化特性1：收敛监控（防卡死） ========== */
    uint32_t convergence_failure_count;  /**< 连续未进入死区的调整次数（>120次视为卡死） */
    uint32_t last_deadband_enter_tick;   /**< 上次进入死区的时间戳（systick，超过5分钟未进入视为异常） */
    
    /* ========== 优化特性2：单轴优先控制（提高收敛速度） ========== */
    bool axis_az_locked;             /**< 方位轴锁定标志（true=误差<0.05°且稳定，无需调整） */
    bool axis_el_locked;             /**< 仰角轴锁定标志（true=误差<0.05°且稳定，无需调整） */
    uint8_t active_axis;             /**< 当前优先调整的轴：0=自动选择误差大的轴，1=仅调AZ，2=仅调EL */
} opt_ctx_t;

/*!
    \brief    光学追踪全局上下文实例（单例模式）
    \detail   初始化为安全默认值：
              - 模式：DISABLED（未启动）
              - 状态：IDLE（空闲）
              - 位置：全部清零
              - 优化特性：全部复位
              
              访问保护：所有外部访问必须通过optical_lock/optical_unlock互斥锁
*/
static opt_ctx_t g = {
    .mode = OPT_MODE_DISABLED,           /* 默认禁用，需调用optical_tracking_set_mode启用 */
    .fsm = OPT_FSM_IDLE,                 /* 空闲状态，调用optical_tracking_start后进入MEASURE */
    .initialized = false,                /* 未初始化，调用optical_tracking_init后为true */
    .in_deadband = false,                /* 默认不在死区，启动后由传感器误差动态判定 */
    .base_azimuth = 0.0f,                /* 基准方位角，启动时从云台读取实际位置 */
    .base_elevation = 0.0f,              /* 基准仰角，启动时从云台读取实际位置 */
    .ex = 0.0f,                          /* X轴误差（左右），由传感器实时更新 */
    .ey = 0.0f,                          /* Y轴误差（上下），由传感器实时更新 */
    .last_position_set_time = 0,         /* 上次发命令时间，初始化时设为当前tick */
    .convergence_failure_count = 0,      /* 卡死计数器，启动时复位 */
    .last_deadband_enter_tick = 0,       /* 死区进入时间，启动时设为当前tick */
    .axis_az_locked = false,             /* 方位轴未锁定，动态根据误差判定 */
    .axis_el_locked = false,             /* 仰角轴未锁定，动态根据误差判定 */
    .active_axis = 0                     /* 自动选择轴，根据误差大小动态切换 */
};

/* FreeRTOS 互斥锁（优化：比临界区更高效，允许中断） */
static SemaphoreHandle_t g_optical_mutex = NULL;  /* 全局互斥锁句柄，用于保护全局上下文结构体g的并发访问 */

/* 可配置的死区阈值（运行时变量） */
static float g_deadband_enter = OPTICAL_ERROR_ENTER_DEFAULT;
static float g_deadband_exit  = OPTICAL_ERROR_EXIT_DEFAULT;

/*!
    \brief    获取互斥锁（加锁函数）
    \detail   在访问全局变量g之前调用，确保线程安全
              - 只在互斥锁已创建且调度器运行时才加锁
              - 使用portMAX_DELAY表示永久等待，直到获取锁
              - inline优化：减少函数调用开销
*/
static inline void optical_lock(void)
{
    /* 检查互斥锁是否已创建 且 FreeRTOS调度器是否正在运行 */
    if (g_optical_mutex != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreTake(g_optical_mutex, portMAX_DELAY);  /* 获取互斥锁，无限等待直到成功 */
    }
}

/*!
    \brief    释放互斥锁（解锁函数）
    \detail   在访问全局变量g之后调用，释放资源
              - 只在互斥锁存在时才释放
              - inline优化：减少函数调用开销
*/
static inline void optical_unlock(void)
{
    if (g_optical_mutex != NULL) {  /* 检查互斥锁是否存在 */
        xSemaphoreGive(g_optical_mutex);  /* 释放互斥锁，允许其他任务获取 */
    }
}

/* ========== 内部辅助函数声明（私有API） ========== */
static void compute_sensor_errors(float* ex, float* ey);  /* 计算四象限传感器的归一化误差（X轴左右，Y轴上下） */
static inline float clamp_elevation(float el);  /* 将仰角限幅到有效范围[0°, 90°] */
static void query_base_position(opt_ctx_t* ctx);  /* 从云台读取当前位置并设置为基准位置 */
static bool calculate_next_target_position(opt_ctx_t* ctx, float* target_az, float* target_el);  /* 根据传感器误差计算下一步目标位置（单轴0.1°步进） */
static void update_deadband_state(opt_ctx_t* ctx, float ex, float ey);  /* 更新死区状态（滞回控制，防止抖动） */
static bool set_gimbal_position_absolute(float azimuth, float elevation);  /* 发送绝对位置命令到云台 */
static float normalize_angle(float angle);  /* 将角度归一化到[0°, 360°)范围 */
static bool get_current_gimbal_position(float* az, float* el);  /* 从缓存读取云台当前位置（避免破坏RS485总线顺序） */
static bool compute_and_store_calibration(void);  /* 计算并存储校准偏差（实际位置-理论位置） */

bool optical_tracking_init(void)
{
    /* 创建互斥锁（优化：代替临界区，降低中断延迟） */
    if (g_optical_mutex == NULL) {  /* 检查互斥锁是否已经创建 */
        g_optical_mutex = xSemaphoreCreateMutex();  /* 创建FreeRTOS互斥锁，用于保护全局变量g */
        if (g_optical_mutex == NULL) {  /* 检查互斥锁创建是否成功 */
            return false;  /* 互斥锁创建失败，返回false表示初始化失败 */
        }
    }
    
    optical_lock();  /* 获取互斥锁，保护后续的全局变量访问 */
    memset(&g, 0, sizeof(g));  /* 将全局上下文结构体g清零，复位所有状态 */
    g.mode = OPT_MODE_DISABLED;  /* 设置控制模式为禁用（初始状态） */
    g.fsm = OPT_FSM_IDLE;  /* 设置状态机为空闲状态（未启动追踪） */
    g.last_position_set_time = systick_get_tick();  /* 记录当前系统时间戳，用于后续冷却时间计算 */
    g.initialized = true;  /* 标记模块已初始化完成，允许后续操作 */
    /* 步骤1: 初始化单轴优先控制状态 */
    g.axis_az_locked = false;  /* 方位轴锁定标志复位为false（未锁定，需要调整） */
    g.axis_el_locked = false;  /* 仰角轴锁定标志复位为false（未锁定，需要调整） */
    g.active_axis = 0;  /* 当前活跃轴设为0（自动模式：根据误差大小动态选择调整轴） */
    optical_unlock();  /* 释放互斥锁，允许其他任务访问 */
    return true;  /* 返回true表示初始化成功 */
}

void optical_tracking_deinit(void)
{
    optical_lock();  /* 获取互斥锁，保护全局变量访问 */
    memset(&g, 0, sizeof(g));  /* 清零全局上下文结构体，清除所有状态和数据 */
    optical_unlock();  /* 释放互斥锁 */
    
    /* 删除互斥锁 */
    if (g_optical_mutex != NULL) {  /* 检查互斥锁是否存在 */
        vSemaphoreDelete(g_optical_mutex);  /* 删除FreeRTOS互斥锁，释放系统资源 */
        g_optical_mutex = NULL;  /* 将互斥锁句柄清空，防止悬空指针 */
    }
}

bool optical_tracking_set_mode(optical_control_mode_t mode)
{
    bool ok = false;  /* 初始化返回值为false（失败状态） */

    optical_lock();  /* 获取互斥锁，确保线程安全 */
    if (g.initialized) {  /* 检查模块是否已初始化 */
        g.mode = mode;  /* 设置新的控制模式（DISABLED/OPTICAL_TRACKING/ASTRONOMICAL_TRACKING） */
        if (mode != OPT_MODE_OPTICAL_TRACKING) {  /* 如果不是光学追踪模式 */
            g.fsm = OPT_FSM_IDLE;  /* 重置状态机为空闲状态 */
            g.in_deadband = false;  /* 清除死区标志（不再处于对准状态） */
        }
        ok = true;  /* 设置成功，返回值改为true */
    }
    optical_unlock();  /* 释放互斥锁 */

    return ok;  /* 返回操作结果 */
}

optical_control_mode_t optical_tracking_get_mode(void)
{
    optical_control_mode_t mode = OPT_MODE_DISABLED;  /* 初始化默认返回值为禁用模式 */

    optical_lock();  /* 获取互斥锁，保护读取操作 */
    if (g.initialized) {  /* 检查模块是否已初始化 */
        mode = g.mode;  /* 读取当前控制模式 */
    }
    optical_unlock();  /* 释放互斥锁 */

    return mode;  /* 返回当前模式（DISABLED/OPTICAL_TRACKING/ASTRONOMICAL_TRACKING） */
}

bool optical_tracking_start(void)
{
    bool started = false;  /* 初始化返回值为false（启动失败） */

    optical_lock();  /* 获取互斥锁，保护全局变量访问 */
    if (g.initialized) {  /* 检查模块是否已初始化 */
        query_base_position(&g);  /* 从云台读取当前位置并设置为基准位置 */
        g.fsm = OPT_FSM_MEASURE;  /* 设置状态机为测量状态（开始追踪流程） */
        g.in_deadband = false;  /* 清除死区标志（启动时认为未对准） */
        g.last_position_set_time = systick_get_tick();  /* 记录当前时间戳，用于冷却时间计算 */
        /* 优化: 重置收敛监控计数器 */
        g.convergence_failure_count = 0;  /* 清零连续调整失败计数器（用于检测卡死状态） */
        g.last_deadband_enter_tick = systick_get_tick();  /* 记录启动时间，用于超时检测 */
        /* 步骤1: 重置单轴优先控制状态（防止上次残留） */
        g.axis_az_locked = false;  /* 清除方位轴锁定标志（启动时两轴都需重新调整） */
        g.axis_el_locked = false;  /* 清除仰角轴锁定标志（启动时两轴都需重新调整） */
        g.active_axis = 0;  /* 重置活跃轴为自动模式（根据误差大小动态选择） */
        started = true;  /* 启动成功，设置返回值为true */
    }
    optical_unlock();  /* 释放互斥锁 */

    return started;  /* 返回启动结果 */
}

void optical_tracking_stop(void)
{
    optical_lock();
    if (g.initialized) {
        g.fsm = OPT_FSM_IDLE;
        g.in_deadband = false;
    }
    optical_unlock();
}

/*!
    \brief    光学追踪主更新函数（需周期调用，建议100ms-500ms）
    \detail   状态机控制流程：
              - IDLE: 非追踪模式，直接返回
              - MEASURE: 冷却时间过后，读取传感器误差，判断死区，选择调整轴，发送云台命令
              - CALIBRATE: 进入死区时执行校准并存储偏差
    \note     本函数是非阻塞的，适合在FreeRTOS任务中循环调用
              冷却机制：每次发送命令后必须等待OPTICAL_SETTLE_MS才能再次调整
*/
void optical_tracking_update(void)
{
    /* 局部变量：减少临界区锁定时间 */
    optical_control_mode_t mode;           /* 当前控制模式的本地副本 */
    opt_fsm_state_t fsm;                   /* 当前状态机状态的本地副本 */
    uint32_t last_move_time;               /* 最近一次下发云台命令的时间戳副本 */
    
    /* 快速读取状态（最小化互斥锁占用时间） */
    optical_lock();                        /* 加锁，保护全局上下文访问 */
    if (!g.initialized) {                  /* 若模块尚未初始化则直接返回 */
        optical_unlock();                  /* 解锁以免阻塞其他任务 */
        return;                            /* 初始化之前不执行任何动作 */
    }
    mode = g.mode;                         /* 缓存当前控制模式，退出临界区后使用 */
    fsm = g.fsm;                           /* 缓存当前状态机状态 */
    last_move_time = g.last_position_set_time; /* 记录最近一次动作的时间戳 */
    optical_unlock();                      /* 解锁，缩短互斥锁占用时长 */
    
    /* 模式检查 - 只在光学追踪模式下运行 */
    if (mode != OPT_MODE_OPTICAL_TRACKING) { /* 非光追模式不做任何调整 */
        return;                            /* 直接返回等待下一次调用 */
    }
    
    uint32_t now = systick_get_tick();     /* 获取当前系统时间戳（ms）用于节流 */
    
    /* ========== FSM 状态机控制 ========== */
    switch (fsm) {                         /* 根据当前状态执行对应逻辑 */
        case OPT_FSM_IDLE:                 /* 空闲状态：未启动或已停止 */
            /* 空闲状态：不执行任何追踪操作 */
            return;                        /* 保持空闲，等待状态切换 */
            
        case OPT_FSM_MEASURE: {            /* 测量状态：正常追踪流程 */
            /* 测量状态：冷却时间过后，读取传感器，检查死区，选择轴，发送命令 */
            
            /* 1. 冷却时间检查：上次发命令后必须等待 OPTICAL_SETTLE_MS */
            if ((now - last_move_time) < OPTICAL_SETTLE_MS) { /* 冷却时间尚未到 */
                return;                    /* 云台尚未稳定，等待下个周期 */
            }
            
            /* 2. 传感器有效性保护 */
            if (!g_sensor_data.valid) {    /* 当前四象限传感器数据无效 */
                return;                    /* 数据不可靠，跳过本次调整 */
            }
            
            /* 3. 读取四象限传感器误差 */
            float ex = 0.0f;               /* X轴误差（左右偏差，-1.0 ~ 1.0） */
            float ey = 0.0f;               /* Y轴误差（上下偏差，-1.0 ~ 1.0） */
            compute_sensor_errors(&ex, &ey); /* 计算传感器误差并写入ex/ey */
            
            /* 4. 更新死区状态（滞回控制） */
            bool was_in_deadband;          /* 更新前的死区状态 */
            bool now_in_deadband;          /* 更新后的死区状态 */
            optical_lock();                /* 加锁以更新共享状态 */
            g.ex = ex;                     /* 缓存最新的X轴误差供其他模块读取 */
            g.ey = ey;                     /* 缓存最新的Y轴误差供其他模块读取 */
            was_in_deadband = g.in_deadband; /* 保存更新前是否在死区 */
            update_deadband_state(&g, ex, ey); /* 根据误差判断是否进入/退出死区 */
            now_in_deadband = g.in_deadband; /* 读取更新后的死区状态 */
            optical_unlock();              /* 解锁，缩短互斥锁持有时间 */
            
            /* 5. 刚进入死区时触发校准 */
            if (!was_in_deadband && now_in_deadband) { /* 检测到首次进入死区 */
                optical_lock();            /* 加锁准备修改状态机 */
                g.fsm = OPT_FSM_CALIBRATE; /* 切换到校准状态，下一周期执行校准 */
                optical_unlock();          /* 解锁 */
                return;                    /* 本周期不再继续，等待校准分支 */
            }
            
            /* 6. 在死区内，不需要调整 */
            if (now_in_deadband) {         /* 已在死区且非首次进入 */
                return;                    /* 维持当前姿态，等待误差增大再调整 */
            }
            
            /* 7. 更新单轴锁定状态并选择优先调整的轴 */
            optical_lock();                /* 加锁以更新锁定状态和活跃轴 */
            float abs_ex = fabsf(ex);      /* 计算X轴误差绝对值供决策使用 */
            float abs_ey = fabsf(ey);      /* 计算Y轴误差绝对值供决策使用 */
            
            /* 更新方位轴锁定状态 */
            if (abs_ex < OPTICAL_AXIS_LOCK_THRESHOLD) { /* 误差足够小，锁定AZ轴 */
                g.axis_az_locked = true;
            } else if (abs_ex > g_deadband_exit) { /* 误差过大，解除AZ锁定 */
                g.axis_az_locked = false;
            }
            
            /* 更新仰角轴锁定状态 */
            if (abs_ey < OPTICAL_AXIS_LOCK_THRESHOLD) { /* 误差足够小，锁定EL轴 */
                g.axis_el_locked = true;
            } else if (abs_ey > g_deadband_exit) { /* 误差过大，解除EL锁定 */
                g.axis_el_locked = false;
            }
            
            /* 选择当前优先调整的轴 */
            if (g.axis_az_locked && g.axis_el_locked) { /* 两轴都稳定 */
                g.active_axis = 0;         /* 设为0，表示即将进入死区 */
            } else if (g.axis_az_locked) { /* 仅方位轴已稳定 */
                g.active_axis = 2;         /* 只调整仰角轴 */
            } else if (g.axis_el_locked) { /* 仅仰角轴已稳定 */
                g.active_axis = 1;         /* 只调整方位轴 */
            } else {                       /* 两轴都未稳定 */
                /* 两轴都未锁定，选择误差较大的轴优先调整 */
                g.active_axis = (abs_ex >= abs_ey) ? 1 : 2; /* 误差大者先调 */
            }
            optical_unlock();              /* 解锁，完成锁定状态更新 */
            
            /* 8. 根据传感器误差计算下一步目标位置（单轴 0.1° 步进） */
            float next_az = 0.0f;          /* 预置下一步目标方位角 */
            float next_el = 0.0f;          /* 预置下一步目标仰角 */
            
            /* calculate_next_target_position内部读取当前云台位置并计算下一步 */
            /* 不需要加锁，因为只读取已稳定的ex/ey/active_axis */
            bool calc_ok = calculate_next_target_position(&g, &next_az, &next_el);
            if (!calc_ok) {                /* 若获取当前云台位置失败 */
                return;                    /* 放弃本次调整，等待下一次 */
            }
            
            /* 9. 收敛监控（检测长期不进入死区的卡死状态） */
            optical_lock();                /* 加锁更新收敛相关计数器 */
            g.convergence_failure_count++; /* 连续未进入死区的计数+1 */
            uint32_t convergence_count = g.convergence_failure_count; /* 读取当前计数 */
            uint32_t time_since_deadband = now - g.last_deadband_enter_tick; /* 距离上次进入死区的时间 */
            optical_unlock();              /* 解锁 */
            
            /* 检测卡死：连续120次调整(约10分钟)未进入死区 或 5分钟内未进入死区 */
            if (convergence_count > 120 || time_since_deadband > 300000) { /* 判断是否触发防卡死 */
                optical_lock();            /* 加锁以复位计数器 */
                g.convergence_failure_count = 0; /* 清零失败计数，避免溢出 */
                g.last_deadband_enter_tick = now; /* 重置死区记录时间，作为报警抑制 */
                optical_unlock();          /* 解锁 */
                /* TODO: 后续可添加sd_logger记录收敛失败事件 */
            }
            
            /* 10. 发送云台位置命令并记录时间（启动新的冷却周期） */
            if (set_gimbal_position_absolute(next_az, next_el)) { /* 向云台发送新的目标位置 */
                optical_lock();            /* 加锁更新最后动作时间 */
                g.last_position_set_time = now;  /* 记录发命令时间，开始冷却 */
                optical_unlock();          /* 解锁 */
            }
            break;                         /* 完成MEASURE分支 */
        }
        
        case OPT_FSM_CALIBRATE: {          /* 校准状态：死区内执行校准 */
            /* 校准状态：存储校准偏差，重置计数器，返回测量状态 */
            
            /* 1. 执行校准 */
            (void)compute_and_store_calibration(); /* 忽略返回值，失败时等待下次机会 */
            
            /* 2. 重置收敛失败计数器 */
            optical_lock();                /* 加锁更新全局状态 */
            g.convergence_failure_count = 0; /* 成功进入死区，重置失败计数 */
            g.last_deadband_enter_tick = now; /* 更新最近进入死区时间戳 */
            g.fsm = OPT_FSM_MEASURE;      /* 校准完成后返回测量状态 */
            optical_unlock();              /* 解锁 */
            break;                         /* 完成CALIBRATE分支 */
        }
        
        default:                           /* 容错：状态机意外值 */
            /* 未知状态：重置为空闲状态 */
            optical_lock();                /* 加锁修正状态 */
            g.fsm = OPT_FSM_IDLE;          /* 复位到IDLE防止状态漂移 */
            optical_unlock();              /* 解锁 */
            break;                         /* 结束默认分支 */
    }
}

bool optical_tracking_get_errors(float* ex, float* ey)
{
    if (!ex || !ey) {                    /* 检查调用方指针有效性，防止空指针引用 */
        return false;                    /* 参数无效时立即返回 */
    }

    bool ok = false;                     /* 默认返回false，表示读取失败 */
    optical_lock();                      /* 加锁，保护共享误差数据 */
    if (g.initialized) {                 /* 仅在模块已初始化时才返回有效值 */
        *ex = g.ex;                      /* 输出缓存的X轴误差 */
        *ey = g.ey;                      /* 输出缓存的Y轴误差 */
        ok = true;                       /* 标记操作成功 */
    }
    optical_unlock();                    /* 解锁，允许其他任务访问 */

    return ok;                           /* 返回结果标志 */
}

void optical_tracking_set_base_position(float azimuth, float elevation)
{
    optical_lock();                      /* 加锁保护基准位置更新 */
    if (g.initialized) {                 /* 确保模块已初始化 */
        g.base_azimuth = normalize_angle(azimuth);   /* 归一化并保存基准方位角 */
        g.base_elevation = clamp_elevation(elevation); /* 限幅并保存基准仰角 */
    }
    optical_unlock();                    /* 解锁 */
}

bool optical_tracking_get_base_position(float* azimuth, float* elevation)
{
    if (!azimuth || !elevation) {        /* 检查输出指针是否有效 */
        return false;                    /* 参数无效直接返回 */
    }

    bool ok = false;                     /* 默认失败 */
    optical_lock();                      /* 加锁，防止并发修改 */
    if (g.initialized) {                 /* 仅在模块初始化后才返回数据 */
        *azimuth = g.base_azimuth;       /* 读取基准方位角 */
        *elevation = g.base_elevation;   /* 读取基准仰角 */
        ok = true;                       /* 标记读取成功 */
    }
    optical_unlock();                    /* 解锁 */
    return ok;                           /* 返回结果 */
}

bool optical_tracking_get_deadband(float* enter_err, float* exit_err)
{
    if (!enter_err || !exit_err) {
        return false;
    }

    optical_lock();
    *enter_err = g_deadband_enter;
    *exit_err  = g_deadband_exit;
    optical_unlock();
    return true;
}

void optical_tracking_set_deadband(float enter_err, float exit_err)
{
    /* 非法值则忽略，保持原值 */
    if (enter_err <= 0.0f || exit_err <= 0.0f ||
        enter_err > 1.0f  || exit_err > 1.0f  ||
        enter_err >= exit_err) {
        return;
    }

    optical_lock();
    g_deadband_enter = enter_err;
    g_deadband_exit  = exit_err;
    optical_unlock();
}

/* ========================================================================== */
/* 内部辅助函数实现                                                           */
/* ========================================================================== */

/*!
    \brief    计算四象限光敏传感器的归一化误差
    \param[out] ex: X轴误差（左右偏差，-1.0表示向左，+1.0表示向右）
              - X轴误差 = (右侧 - 左侧) / (左侧 + 右侧)
              - Y轴误差 = (上方 - 下方) / (上方 + 下方)
              
              归一化范围：[-1.0, 1.0]
              - 0.0 表示对准中心
              - 正值表示太阳在相应方向
              - 负值表示需要向相反方向调整
              
    \note     直接访问全局变量 g_sensor_data (定义在 light_sensor.c)
              - 该变量由光敏传感器ISR自动更新
              - 包含4个电压值：up_voltage, left_voltage, right_voltage, down_voltage
              - valid 标志指示数据是否有效
*/
static void compute_sensor_errors(float* ex, float* ey)
{
    /* 检查数据有效性（直接访问全局变量）*/
    if (!g_sensor_data.valid) {          /* 若传感器数据最近一次采样无效 */
        *ex = 0.0f;                      /* 返回0，避免使用陈旧数据 */
        *ey = 0.0f;                      /* 返回0，避免使用陈旧数据 */
        return;                          /* 无需继续计算 */
    }

    /* 读取传感器电压值（直接从全局变量） */
    float up = g_sensor_data.up_voltage;     /* 上象限电压 */
    float left = g_sensor_data.left_voltage; /* 左象限电压 */
    float right = g_sensor_data.right_voltage; /* 右象限电压 */
    float down = g_sensor_data.down_voltage;   /* 下象限电压 */

    /* 防止除零：设置最小阈值 */
    const float eps = 1e-3f;             /* 小阈值，避免分母趋零 */
    float sum_lr = left + right;         /* 左+右，用于X轴归一化 */
    float sum_ud = down + up;            /* 下+上，用于Y轴归一化 */

    if (sum_lr < eps) sum_lr = eps;      /* 若总和过小，使用阈值替代 */
    if (sum_ud < eps) sum_ud = eps;      /* 若总和过小，使用阈值替代 */

    /* 归一化误差计算 */
    float e_x = (right - left) / sum_lr; /* 右-左除以总和得到左右偏差 */
    float e_y = (up - down) / sum_ud;    /* 上-下除以总和得到上下偏差 */

    /* 限幅到[-1.0, 1.0] */
    if (e_x > 1.0f) e_x = 1.0f;          /* 上限保护 */
    if (e_x < -1.0f) e_x = -1.0f;        /* 下限保护 */
    if (e_y > 1.0f) e_y = 1.0f;          /* 上限保护 */
    if (e_y < -1.0f) e_y = -1.0f;        /* 下限保护 */

    *ex = e_x;                           /* 输出X轴误差 */
    *ey = e_y;                           /* 输出Y轴误差 */
}

static void query_base_position(opt_ctx_t* ctx)
{
    if (ctx == NULL) {                   /* 防御性编程：空指针直接返回 */
        return;
    }

    float az = 0.0f;                     /* 临时变量保存方位角 */
    float el = 0.0f;                     /* 临时变量保存仰角 */
    if (!get_current_gimbal_position(&az, &el)) { /* 尝试读取云台当前位置 */
        az = ctx->base_azimuth;          /* 读取失败时保留原基准方位角 */
        el = ctx->base_elevation;        /* 读取失败时保留原基准仰角 */
    }

    ctx->base_azimuth = normalize_angle(az);   /* 归一化并记录最新基准方位角 */
    ctx->base_elevation = clamp_elevation(el); /* 限幅并记录最新基准仰角 */
}

/*!
    \brief    根据传感器误差计算下一步目标位置
    \param[in/out] ctx: 控制上下文
    \param[out] target_az: 计算得到的目标方位角
    \param[out] target_el: 计算得到的目标仰角
    \retval   bool: 是否成功计算新目标（失败通常是云台位置读取失败）
    \detail   控制策略：
              1. 读取云台当前位置（失败则返回false）
              2. 固定步长调整：
                 - 优先调整误差绝对值较大的轴
                 - 每次固定步进0.1°
                 - 方向: ex>0 → Az减; ey>0 → El增
              3. 角度归一化和限幅后输出
*/
static bool calculate_next_target_position(opt_ctx_t* ctx, float* target_az, float* target_el)
{
    if (!ctx || !target_az || !target_el) {    /* 检查输入输出指针合法性 */
        return false;                         /* 参数错误直接返回 */
    }

    /* 读取当前位置，若失败则返回false，调用方不应发送命令 */
    float current_az = 0.0f;                  /* 当前方位角 */
    float current_el = 0.0f;                  /* 当前仰角 */
    if (!get_current_gimbal_position(&current_az, &current_el)) { /* 尝试从缓存读取云台位置 */
        return false;                          /* 读取失败，等待下次重试 */
    }

    float next_az = current_az;               /* 初始化下一步目标方位角 */
    float next_el = current_el;               /* 初始化下一步目标仰角 */

    /* 步骤4: 根据active_axis只调整一个轴（单轴优先策略） */
    uint8_t axis = ctx->active_axis;          /* 当前选中的优先轴 */
    
    if (axis == 1 || (axis == 0 && fabsf(ctx->ex) >= fabsf(ctx->ey))) {
        /* 调整方位轴（axis=1明确指定，或axis=0时方位误差更大） */
        /* 自适应步距：误差大用大步距，误差小用小步距 */
        float abs_ex = fabsf(ctx->ex);
        float step = (abs_ex > OPTICAL_STEP_SWITCH_ERR) ? 
                     OPTICAL_STEP_LARGE_DEG : OPTICAL_STEP_SMALL_DEG;

        if (ctx->ex > 0.0f) {                /* 传感器判定太阳在右侧 */
            next_az -= step;                 /* 方位角减小，向右旋转 */
        } else {                             /* 太阳在左侧或正中 */
            next_az += step;                 /* 方位角增大，向左旋转 */
        }
        /* 仰角保持不动 */
        next_el = current_el;                /* 保持当前仰角 */
        
    } else if (axis == 2 || (axis == 0 && fabsf(ctx->ey) > fabsf(ctx->ex))) {
        /* 调整仰角轴（axis=2明确指定，或axis=0时仰角误差更大） */
        /* 自适应步距：误差大用大步距，误差小用小步距 */
        float abs_ey = fabsf(ctx->ey);
        float step = (abs_ey > OPTICAL_STEP_SWITCH_ERR) ? 
                     OPTICAL_STEP_LARGE_DEG : OPTICAL_STEP_SMALL_DEG;

        if (ctx->ey > 0.0f) {                /* 传感器判定太阳在上侧 */
            next_el += step;                 /* 仰角增大，云台向上抬 */
        } else {                             /* 太阳在下侧或正中 */
            next_el -= step;                 /* 仰角减小，云台向下压 */
        }
        /* 方位角保持不动 */
        next_az = current_az;                /* 保持当前方位角 */
    }

    /* 角度规范化和限幅 */
    next_az = normalize_angle(next_az);      /* 方位角归一化至0-360° */
    next_el = clamp_elevation(next_el);      /* 仰角限幅至0-90° */

    *target_az = next_az;                    /* 通过指针返回目标方位角 */
    *target_el = next_el;                    /* 通过指针返回目标仰角 */
    
    return true;                             /* 成功计算新目标 */
}

/* TODO: [TEST] 使用可配置的云台仰角范围 - 测试后需恢复为固定值 (5-90度) */
/* Inline helper to clamp elevation to gimbal elevation range [min, max] */
static inline float clamp_elevation(float el)
{
    const config_runtime_t* config = flash_config_get_runtime();

    float elev_min = 5.0f;   /* 默认最小值 */
    float elev_max = 90.0f;  /* 默认最大值 */

    if (config != NULL) {
        elev_min = config->gimbal_elevation_min;
        elev_max = config->gimbal_elevation_max;
    }

    if (el < elev_min) return elev_min;  /* 小于最小值时夹紧 */
    if (el > elev_max) return elev_max;  /* 大于最大值时夹紧 */
    return el;                            /* 落在有效范围内则原值返回 */
}

/*!
    \brief    更新死区状态（滞回控制，防止抖动）
    \param[in/out] ctx: 控制上下文
    \param[in] ex: X轴误差
    \param[in] ey: Y轴误差
    \detail   死区控制原理（标准滞回控制）：
              
              进入条件：|ex| < 0.02 且 |ey| < 0.02（严格，对准精度高）
              退出条件：|ex| > 0.025 或 |ey| > 0.025（宽松，允许一定偏差）
              
              ┌─────────────────────────────────┐
              │   死区滞回（防抖动）             │
              ├─────────────────────────────────┤
              │  误差                           │
              │   ↑                             │
              │0.025├────┐ 退出死区（误差大）   │
              │     │滞回│                      │
              │0.02 ├────┘ 进入死区（误差小）   │
              │     │                           │
              │  0  ├──────────→ 时间           │
              └─────────────────────────────────┘
              
              滞回空间[0.02, 0.025]：
              - 误差从大变小：需要<0.02才进入死区
              - 误差从小变大：需要>0.025才退出死区
              - 在0.02-0.025之间保持原状态，避免频繁切换
*/
static void update_deadband_state(opt_ctx_t* ctx, float ex, float ey)
{
    if (!ctx) {  /* 空指针保护：上下文对象无效则直接返回 */
        return;
    }
    float ax = fabsf(ex);  /* 计算X轴误差的绝对值（左右偏差量） */
    float ay = fabsf(ey);  /* 计算Y轴误差的绝对值（上下偏差量） */

    if (!ctx->in_deadband) {  /* 当前不在死区内：判断是否满足进入条件 */
        /* 不在死区：检查是否可以进入死区 */
        if (ax < g_deadband_enter && ay < g_deadband_enter) {  /* 两轴误差都小于进入阈值 */
            ctx->in_deadband = true;  /* 误差足够小，进入死区（停止调整） */
        }
    } else {  /* 当前在死区内：判断是否满足退出条件 */
        /* 已在死区：检查是否需要退出死区 */
        if (ax > g_deadband_exit || ay > g_deadband_exit) {  /* 任一轴误差超过退出阈值 */
            ctx->in_deadband = false;  /* 误差变大，退出死区（恢复调整） */
        }
    }
}

/*!  
    \brief    发送绝对位置命令到云台
    \param[in] azimuth: 目标方位角（单位：度，范围：0-360）
    \param[in] elevation: 目标仰角（单位：度，范围：0-90）
    \retval   bool: true=命令发送成功，false=命令发送失败
    \note     实际通过RS485 Modbus RTU协议发送，由gimbal_control模块处理
*/
static bool set_gimbal_position_absolute(float azimuth, float elevation)
{
    /* 使用队列方式发送命令，避免阻塞光学追踪任务 */
    bool queued = gimbal_queue_set_position(azimuth, elevation);
    if (!queued) {  /* 队列满时记录失败（由gimbal_process在总线空闲时发送） */
        return false;  /* 命令排队失败，返回false */
    }
    return true;  /* 命令成功加入队列，返回true */
}

/*!
    \brief    将角度归一化到[0°, 360°)范围
    \param[in] angle: 输入角度（可以是任意实数）
    \retval   float: 归一化后的角度（0 <= angle < 360）
    \detail   处理示例：
              - 输入 370° → 输出 10°
              - 输入 -30° → 输出 330°
              - 输入 720° → 输出 0°
    \note     使用fmodf替代while循环，避免大角度值时的性能问题
*/
static float normalize_angle(float angle)
{
    /* 优化：避免潜在的死循环，使用 fmodf 更安全高效 */
    angle = fmodf(angle, 360.0f);  /* 对360取模，将角度限制到(-360, 360)范围 */
    if (angle < 0.0f) {  /* 如果角度为负值 */
        angle += 360.0f;  /* 加360转换到正值范围[0, 360) */
    }
    return angle;  /* 返回归一化后的角度 */
}

/*!
    \brief    从缓存读取云台当前位置
    \param[out] az: 方位角指针（传NULL则跳过该轴，单位：度）
    \param[out] el: 仰角指针（传NULL则跳过该轴，单位：度）
    \retval   bool: true=读取成功，false=缓存无效
    \detail   设计原则：
              - 只读缓存，不主动查询云台（避免破坏RS485总线时序）
              - 缓存由SolarTracking_Task状态机定期更新（约1秒1次）
              - 支持只查询单个轴（传NULL跳过）
    \note     优化点：避免多任务并发访问RS485总线导致冲突
*/
static bool get_current_gimbal_position(float* az, float* el)
{
    /* 支持只查询单个轴（传NULL跳过该轴） */
    /* 从缓存读取云台位置（由SolarTracking_Task状态机定期更新）*/
    /* 不再主动查询，避免破坏RS485总线的严格顺序访问 */
    gimbal_position_t pos;  /* 定义临时结构体存储云台位置 */
    if (gimbal_get_cached_position(&pos)) {  /* 从缓存读取位置（非阻塞） */
        if (az != NULL) {  /* 如果调用者需要方位角 */
            *az = pos.azimuth;  /* 写入方位角到输出参数 */
        }
        if (el != NULL) {  /* 如果调用者需要仰角 */
            *el = pos.elevation;  /* 写入仰角到输出参数 */
        }
        return true;  /* 缓存有效，返回成功 */
    }
    
    /* 缓存无效（尚未查询过），返回失败 */
    return false;  /* 缓存无效（系统刚启动或云台通信失败），返回失败 */
}

/*!
    \brief    计算并存储校准偏差（在死区内调用）
    \retval   bool: 是否成功计算并存储
    \detail   校准流程：
              
              1. 读取云台当前位置（caz, cel）
              2. 从data_hub获取理论太阳位置（避免重复计算）
              3. 计算偏差 = 实际位置 - 理论位置
              4. 存储偏差到Modbus寄存器和Flash
              
              偏差含义：
              - 正偏差：云台实际角度 > 理论角度
              - 负偏差：云台实际角度 < 理论角度
              
              用途：
              - 补偿传感器安装误差
              - 补偿云台机械误差
              - 上位机可读取用于监控
              
              优化：使用data_hub缓存的太阳位置，避免重复天文解算
*/
static bool compute_and_store_calibration(void)
{
    /* 时间窗口检查：校准只在本地时间8:00-8:59时段执行 */
    rtc_datetime_t dt;
    if (!time_get_local_datetime(&dt)) {
        return false;  /* 无法获取时间，不执行校准 */
    }
    if (dt.hour != 8) {
        return false;  /* 不在8:00-8:59时段，跳过校准 */
    }

    /* 步骤1: 获取云台当前位置 */
    float caz = 0.0f;  /* 云台实际方位角（从缓存读取） */
    float cel = 0.0f;  /* 云台实际仰角（从缓存读取） */
    if (!get_current_gimbal_position(&caz, &cel)) {  /* 尝试从缓存读取云台位置 */
        return false;  /* 读取失败（缓存无效），放弃本次校准 */
    }

    /* 步骤2: 从data_hub获取最新的理论太阳位置（云台坐标系） */
    /* 注意：使用缓存数据而非重新计算，提高效率 */
    data_hub_snapshot_t snapshot;  /* 定义快照结构体接收数据 */
    data_hub_get_snapshot(&snapshot);  /* 从data_hub读取最新数据快照（包含太阳位置） */
    
    if (!snapshot.sun_valid) {  /* 检查太阳位置数据有效性标志 */
        /* 太阳位置数据无效，无法校准 */
        return false;  /* 数据无效（可能是GPS/RTC异常），放弃校准 */
    }
    
    float az_ref = snapshot.sun_azimuth;  /* 云台坐标系下的理论方位角（天文解算结果） */
    float el_ref = snapshot.sun_elevation; /* 云台坐标系下的理论仰角（天文解算结果） */

    /* 步骤3: 计算偏差（实际 - 理论） */
    float dAz = caz - az_ref;  /* 计算方位角偏差（正值=实际角度偏大） */
    /* 方位角偏差归一化到[-180°, 180°] */
    while (dAz >= 180.0f) dAz -= 360.0f;  /* 偏差>=180°时减360°（处理跨越0°边界的情况） */
    while (dAz < -180.0f) dAz += 360.0f;  /* 偏差<-180°时加360°（处理跨越0°边界的情况） */

    float dEl = cel - el_ref;  /* 计算仰角偏差（正值=实际角度偏大） */
    
    if (!host_comm_request_calibration_save(dAz, dEl)) {
        return false;
    }
    
    return true;  /* 校准成功，返回true */
}
