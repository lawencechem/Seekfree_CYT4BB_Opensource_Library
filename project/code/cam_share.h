#ifndef _CAM_SHARE_H_
#define _CAM_SHARE_H_
#include "zf_common_headfile.h"

/* ==================== 双核共享信箱(替代会死锁的 IPC pipe) ====================
 * CM7_1 视觉核 写，CM7_0 飞控核 轮询读。
 * 信箱放在 CM7_1 的 SRAM 区固定地址(0x28080000~0x280C0000 内)：
 *   - 由 CM7_1 编译单元用 @ 预留(见 camera.c)，CM7_1 链接器不在此放别的变量；
 *   - 被 CM7_1 启动时的 ECC 初始化覆盖 → CM7_1 读写不会 ECC 故障；
 *   - CM7_0 只【读】这个跨区地址(早先 IPC 已证明 CM7_0 能读 CM7_1 的内存)。
 * 两核 DCache 都已关 → 直接读写 SRAM、天然一致、无 ack、无死锁。
 *
 * 一致性：CM7_1 先写数据、最后写 seq；CM7_0 先读 seq、变了才读数据 → 读到一定是完整一帧。*/

#define CAM_SHARE_ADDR  (0x280A0000U)   /* CM7_1 SRAM区(0x28080000~0x280C0000)内的固定地址 */

typedef struct
{
    volatile uint32 seq;     /* 序列号：CM7_1 每写一帧 +1；CM7_0 用它判新帧 */
    volatile int16  px_u;    /* 水平像素偏移(车相对图像中心) */
    volatile int16  px_v;    /* 垂直像素偏移 */
    volatile int16  area;    /* 诊断：本帧亮点数(=camera.c 的 count)。锁不锁都写，用来判"没看到灯(≈0)/过曝一片(很大)" */
    volatile uint8  valid;   /* 1=锁到目标 0=丢目标 */
    volatile uint8  maxg;    /* 诊断：本帧全图最大灰度(0~255)。判灯多亮/阈值设多少 */
} cam_share_t;

#define CAM_SHARE  ((volatile cam_share_t *)CAM_SHARE_ADDR)

/* CM7_1 上电调一次：清零信箱(把 seq 归 0，避免 CM7_0 一开始读到随机旧值) */
static inline void cam_share_init(void)
{
    CAM_SHARE->seq   = 0;
    CAM_SHARE->px_u  = 0;
    CAM_SHARE->px_v  = 0;
    CAM_SHARE->area  = 0;
    CAM_SHARE->maxg  = 0;
    CAM_SHARE->valid = 0;
}

/* CM7_1 每帧调：写一帧。先写数据、最后写 seq，保证 CM7_0 读到完整一帧 */
static inline void cam_share_write(int16 u, int16 v, uint8 valid, int16 area, uint8 maxg)
{
    CAM_SHARE->px_u  = u;
    CAM_SHARE->px_v  = v;
    CAM_SHARE->area  = area;
    CAM_SHARE->maxg  = maxg;
    CAM_SHARE->valid = valid;
    CAM_SHARE->seq   = CAM_SHARE->seq + 1U;   /* 最后写 seq */
}

#endif
