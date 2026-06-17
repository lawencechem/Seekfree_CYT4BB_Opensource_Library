#include "camera.h"
#include "cam_share.h"

/* 在本核(CM7_1)SRAM区用 @ 预留共享信箱地址，链接器就不会在此放别的变量。
 * 该地址被 CM7_1 启动 ECC 初始化覆盖 → 读写不会 ECC 故障。CM7_0 只跨区读它。 */
__root __no_init volatile cam_share_t cam_mbox_reserve @ 0x280A0000;

/* SDK 摄像头原始灰度图(由 mt9v03x 驱动定义并采集) */
extern uint8 mt9v03x_image[CAM_ROWS][CAM_COLS];

float cam_error_x = 0.0f;
float cam_error_y = 0.0f;
int   cam_area    = 0;
uint8 cam_locked  = 0;
uint8 cam_max_gray = 0;   /* 诊断：本帧扫描区最大灰度，透传给飞控核判"灯多亮/阈值合不合适" */

static float last_cx = 0.0f, last_cy = 0.0f;  /* 上一帧质心(图像坐标) */
static uint8 have_lock = 0;                    /* 是否处于跟踪态(已锁) */

/* 处理一帧：阈值化 + 求亮点质心 + ROI/面积门限。
 * 未锁定：全图搜索(靠滤光片保证画面里只有车灯)；
 * 已锁定：只在上一帧质心附近的窗口里找(信标在窗外被自动排除)。*/
uint8 camera_process(void)
{
    int x0 = 0, y0 = 0, x1 = CAM_COLS, y1 = CAM_ROWS;

    if (have_lock)   /* 跟踪态：限定到上一帧质心附近的 ROI 窗口 */
    {
        x0 = (int)last_cx - ROI_HALF; if (x0 < 0)        x0 = 0;
        x1 = (int)last_cx + ROI_HALF; if (x1 > CAM_COLS) x1 = CAM_COLS;
        y0 = (int)last_cy - ROI_HALF; if (y0 < 0)        y0 = 0;
        y1 = (int)last_cy + ROI_HALF; if (y1 > CAM_ROWS) y1 = CAM_ROWS;
    }

    long sum_x = 0, sum_y = 0;
    int  count = 0;
    int  maxg  = 0;
    for (int y = y0; y < y1; y++)
    {
        for (int x = x0; x < x1; x++)
        {
            uint8 px = mt9v03x_image[y][x];
            if (px > maxg) maxg = px;          /* 诊断：扫描区最大灰度(未锁=全图) */
            if (px > LED_THRESHOLD)
            {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }
    cam_area = count;
    cam_max_gray = (uint8)maxg;

    /* 面积门限：太小=噪点/无目标，太大=异常 → 判丢目标，回到全图重新捕获 */
    if (count < LED_MIN_AREA || count > LED_MAX_AREA)
    {
        cam_locked = 0;
        have_lock  = 0;
        return 0;
    }

    /* 质心(图像坐标) */
    float cx = (float)sum_x / count;
    float cy = (float)sum_y / count;
    last_cx = cx;
    last_cy = cy;
    have_lock = 1;

    /* 相对图像中心的偏移 = 车相对飞机方向(像素) */
    //cam_error_x = cx - (CAM_COLS - 1) / 2.0f;   /* += 右 */
    //cam_error_y = cy - (CAM_ROWS - 1) / 2.0f;   /* += 下 */
    
    cam_error_x = cy - (CAM_ROWS - 1) / 2.0f;   /* += 右 */
    cam_error_y = cx - (CAM_COLS - 1) / 2.0f;   /* += 下 */    
    
    cam_locked = 1;
    return 1;
}
