#include "Optical_flow.h"
#if 0   // PMW3901 optical-flow implementation disabled: switching to UP FLOW 302.
#include "zf_device_pmw3901.h"
#include "math.h"

/*
 * ==================== 鏈枃浠惰亴璐?====================
 * 鍏夋祦閫熷害闃诲凹妯″潡鐨勫疄鐜般€傚皢 PMW3901 浼犳劅鍣ㄧ殑鍘熷鍍忕礌澧為噺锛? * 缁忚繃 鎹㈢畻 鈫?鍘诲櫔 鈫?婊ゆ尝 鈫?PI鎺у埗 鍚庯紝杈撳嚭濮挎€佷慨姝ｈ銆? *
 * ==================== 璋冪敤鏃跺簭锛堝湪 main 涓級 ====================
 *
 *   姣?10ms 涓诲惊鐜? *     鈹溾攢鈹€ Altitude_Control_Task()     // 瀹氶珮浠诲姟锛堟洿鏂?current_height_cm锛? *     鈹溾攢鈹€ of_cnt++
 *     鈹斺攢鈹€ 姣?20ms锛坥f_cnt >= 2锛? *           鈹溾攢鈹€ Optical_Flow_Update(current_height_cm)   // 鈶?璇讳紶鎰熷櫒銆佹崲绠楅€熷害
 *           鈹斺攢鈹€ Optical_Flow_Speed_Damp(0.02, &pitch, &roll)  // 鈶?PI 璁＄畻淇瑙? *
 * 娉ㄦ剰锛歎pdate 蹇呴』鍦?Speed_Damp 涔嬪墠璋冪敤锛屽洜涓?Damp 渚濊禆 Update 浜у嚭鐨?of_vel_x/y
 *
 * ==================== 鍙橀噺鍛藉悕瑙勫垯 ====================
 *   of_      = optical flow锛堝厜娴侊級鍓嶇紑
 *   raw_     = 鍘熷鎹㈢畻鍊硷紙鏈护娉級
 *   lpf_     = low-pass filtered锛堜綆閫氭护娉㈠悗锛? *   vel_     = velocity锛堥€熷害锛? *   _i       = integral锛堢Н鍒?绱姞锛? *   corr     = correction锛堜慨姝ｉ噺锛? */

/* ==================== 鍏ㄥ眬鍙橀噺瀹氫箟 ==================== */

/* 瀵瑰鐘舵€侀噺 鈥斺€?鍏朵粬妯″潡鍙洿鎺ヨ鍙栫敤浜庤皟璇曟墦鍗?*/
float of_vel_x = 0.0f;        // X杞存护娉㈠悗閫熷害锛坈m/s锛夛紝绾﹀畾涓哄墠鍚庢柟鍚戯紝瀵瑰簲 pitch 淇
float of_vel_y = 0.0f;        // Y杞存护娉㈠悗閫熷害锛坈m/s锛夛紝绾﹀畾涓哄乏鍙虫柟鍚戯紝瀵瑰簲 roll 淇
uint8 of_data_valid = 0;      // 鏁版嵁鏈夋晥鏍囧織锛?=閫熷害鍙俊锛?=楂樺害瓒呴檺鎴栦紶鎰熷櫒寮傚父
int16 of_dbg_dx = 0;          // PMW3901 鍘熷 delta_x锛屼緵鏂瑰悜楠岃瘉鎵撳嵃
int16 of_dbg_dy = 0;          // PMW3901 鍘熷 delta_y锛屼緵鏂瑰悜楠岃瘉鎵撳嵃
float of_dbg_raw_vx = 0.0f;   // Raw forward/back velocity before filtering
float of_dbg_raw_vy = 0.0f;   // Raw left/right velocity before filtering
uint16 of_zero_streak = 0;    // Consecutive valid frames with dx/dy both zero
/* PI 鎺у埗鍣ㄥ疄渚?鈥斺€?瀵瑰鏆撮湶锛屾柟渚垮湪涓插彛璋冭瘯鏃跺湪绾挎敼 kp/ki */
PID_t pid_of_vx;              // X杞撮€熷害闃诲凹 PI锛堣緭鍑?鈫?pitch 淇瑙掞級
PID_t pid_of_vy;              // Y杞撮€熷害闃诲凹 PI锛堣緭鍑?鈫?roll 淇瑙掞級

/* 鍐呴儴鐘舵€侀噺 鈥斺€?static 淇濇姢锛屽彧鑳藉湪鏈枃浠跺唴璁块棶 */
static float of_raw_vel_x = 0.0f;     // Raw forward/back velocity
static float of_raw_vel_y = 0.0f;     // Raw left/right velocity
static float of_lpf_vel_x = 0.0f;     // Low-pass filtered forward/back velocity
static float of_lpf_vel_y = 0.0f;     // Y杞翠綆閫氭护娉㈠悗鐨勯€熷害
static uint8 of_invalid_streak = 0;    // 杩炵画楂樺害鏃犳晥甯ц鏁帮紙鐢ㄤ簬琛板噺绛栫暐锛?

/* ==================== Optical_Flow_Init ====================
 *
 * 鍔熻兘锛氬垵濮嬪寲鍏夋祦妯″潡鐨勫叏閮ㄧ姸鎬? * 璋冪敤鏃舵満锛氫笂鐢靛悗鍙皟涓€娆★紝鍦?pmw3901_init() 涔嬪悗
 *
 * 褰撳墠鍙傛暟璇存槑锛? *   kp = 0.035 鈫?閫熷害璇樊 1 cm/s 鏃惰緭鍑?0.035 搴︿慨姝ｈ
 *                渚嬪椋樹簡 10 cm/s锛屼慨姝ｈ = 0.035 脳 10 = 0.35 搴? *   ki = 0.0   鈫?鍏抽棴绉垎锛堥獙璇侀樁娈碉紝閬垮厤闈欐鍋忕疆琚Н鍒嗘斁澶э級
 *   kd = 0.0   鈫?涓嶇敤寰垎锛堥€熷害淇″彿鏈韩宸叉湁浣庨€氭护娉紝涓嶉渶瑕?D 椤癸級
 *   out_limit  鈫?淇瑙掓渶澶?卤2 搴︼紙楠岃瘉闃舵淇濆畧鍊硷級
 *
 * 寮€绉垎鐨勬椂鏈猴細
 *   褰?kp 璋冨ソ鍚庯紝濡傛灉鍙戠幇鎮仠鏃舵湁 1-2 cm/s 鐨勭ǔ鎬佽宸棤娉曟秷闄わ紝
 *   鍙互鎶?ki 浠庡皬鍊硷紙0.005锛夊紑濮嬫參鎱㈠姞锛宨_limit 璁惧埌 3~5 搴︺€? */
void Optical_Flow_Init(void)
{
    /* ---- X杞?PI锛堝搴斾刊浠版柟鍚戠殑閫熷害闃诲凹锛?---- */
    pid_of_vx.kp = 0.020f;
    pid_of_vx.ki = 0.0f;             // 楠岃瘉闃舵鍏抽棴绉垎
    pid_of_vx.kd = 0.0f;             // 涓嶇敤寰垎
    pid_of_vx.i_limit = 0.0f;
    pid_of_vx.out_limit = OF_ANGLE_LIMIT;
    pid_of_vx.integral = 0.0f;

    /* ---- Y杞?PI锛堝搴旀í婊氭柟鍚戠殑閫熷害闃诲凹锛?---- */
    pid_of_vy.kp = 0.020f;
    pid_of_vy.ki = 0.0f;
    pid_of_vy.kd = 0.0f;
    pid_of_vy.i_limit = 0.0f;
    pid_of_vy.out_limit = OF_ANGLE_LIMIT;
    pid_of_vy.integral = 0.0f;

    /* ---- 閫熷害鐘舵€佸叏閮ㄥ綊闆?---- */
    of_vel_x = 0.0f;
    of_vel_y = 0.0f;
    of_raw_vel_x = 0.0f;
    of_raw_vel_y = 0.0f;
    of_lpf_vel_x = 0.0f;
    of_lpf_vel_y = 0.0f;
    of_data_valid = 0;
    of_dbg_dx = 0;
    of_dbg_dy = 0;
    of_dbg_raw_vx = 0.0f;
    of_dbg_raw_vy = 0.0f;
    of_zero_streak = 0;
    of_invalid_streak = 0;
}


/* ==================== Optical_Flow_Update ====================
 *
 * 鍔熻兘锛氳鍙栦竴甯у厜娴佹暟鎹?鈫?鎹㈢畻涓虹墿鐞嗛€熷害 鈫?婊ゆ尝 鈫?鏇存柊鍏ㄥ眬閫熷害
 * 璋冪敤鏃舵満锛氭瘡 20ms 鐢变富寰幆璋冪敤涓€娆? * 鍓嶇疆鏉′欢锛歨eight_cm 鐢卞畾楂樻ā鍧楁彁渚涳紙current_height_cm锛? *
 * 澶勭悊娴佺▼锛? *   鈶?楂樺害闂ㄦ帶 鈥斺€?楂樺害鏃犳晥鏃舵爣璁版暟鎹棤鏁堬紝瀵归€熷害鍋氳“鍑忚€岄潪娓呴浂
 *   鈶?璇诲彇鍘熷鏁版嵁 鈥斺€?璋冪敤 pmw3901_get_motion() 鑾峰彇鍍忕礌澧為噺
 *   鈶?閫熷害鎹㈢畻 鈥斺€?鍍忕礌澧為噺 脳 绯绘暟 脳 楂樺害 / 鏃堕棿 鈫?鐗╃悊閫熷害 (cm/s)
 *   鈶?璺冲彉鍓旈櫎 鈥斺€?瓒呰繃 300cm/s 鐨勫抚瑙嗕负鍣０锛岀敤涓婁竴甯ф护娉㈠€间唬鏇? *   鈶?浣庨€氭护娉?鈥斺€?涓€闃?IIR 婊ゆ尝骞虫粦閫熷害
 *
 * @param height_cm  褰撳墠 TOF 娴嬮噺楂樺害锛堝帢绫筹級锛岃寖鍥?10~220 cm
 */
void Optical_Flow_Update(float height_cm)
{
    /* ======== 绗憼姝ワ細楂樺害闂ㄦ帶 ========
     *
     * PMW3901 鍦ㄤ互涓嬫儏鍐典笉鍙潬锛?     *   - 楂樺害 < 10cm锛氶暅澶村鐒︽ā绯婏紝杩戝湴闈㈢汗鐞嗗お澶?     *   - 楂樺害 > 220cm锛氳秴鍑轰紶鎰熷櫒鏈夋晥璇嗗埆鑼冨洿
     *
     * 鏃犳晥鏃朵笉鑳界洿鎺ユ妸閫熷害娓呴浂锛屽惁鍒欓珮搴︿竴鎭㈠锛屾帶鍒堕噺浼氱獊鐒惰烦鍙樸€?     * 鍋氭硶鏄閫熷害鍋氶€愬抚琛板噺锛堜箻浠ヤ竴涓皬浜?1 鐨勭郴鏁帮級锛?     * 杩欐牱閫熷害浼氳嚜鐒跺钩婊戝湴瓒嬪悜闆躲€?     */
    if (height_cm < OF_MIN_HEIGHT_CM || height_cm > OF_MAX_HEIGHT_CM)
    {
        of_data_valid = 0;

        /* 琛板噺绛栫暐锛氭牴鎹繛缁棤鏁堝抚鏁伴€夋嫨琛板噺閫熺巼
         *   - 鐭殏閬尅锛堚墹5甯?鈮?100ms锛夛細鐢?0.97 缂撴參琛板噺锛?00ms 鍚庝繚鐣欑害 74%
         *     杩欐牱 TOF 鐭殏涓㈠け涓嶄細瀵艰嚧閫熷害淇℃伅澶у箙涓㈠け
         *   - 鎸佺画澶辨晥锛?5甯э級锛氱敤 0.90 鍔犻€熻“鍑忥紝绾?1 绉掑悗闄嶅埌鎺ヨ繎闆?         *     閬垮厤鏃х殑銆佸凡涓嶅彲淇＄殑閫熷害闀挎椂闂存畫鐣欏湪婊ゆ尝鍣ㄤ腑 */
        if (of_invalid_streak < 250)
        {
            of_invalid_streak++;
        }
        float decay = (of_invalid_streak > 5) ? 0.90f : 0.97f;

        /* 鎵€鏈夐€熷害鐘舵€佸悓姝ヨ“鍑忥紙澶栭儴鍙 + 鍐呴儴婊ゆ尝鍣級 */
        of_vel_x *= decay;
        of_vel_y *= decay;
        of_raw_vel_x *= decay;
        of_raw_vel_y *= decay;
        of_lpf_vel_x *= decay;
        of_lpf_vel_y *= decay;

        /* 楂樺害鏃犳晥鏃舵竻鎺夋湰甯у師濮嬪閲忥紝閬垮厤鏃ュ織璇妸涓婁竴甯?dx/dy 褰撴垚褰撳墠鏁版嵁銆?*/
        of_dbg_dx = 0;
        of_dbg_dy = 0;
        of_dbg_raw_vx = of_raw_vel_x;
        of_dbg_raw_vy = of_raw_vel_y;
        of_zero_streak = 0;

        /* 娓呴浂 PI 绉垎锛氶珮搴︽棤鏁堟椂鍋滄绉垎锛岄槻姝㈠湪鏃犳晥鏁版嵁涓婄Н绱敊璇Н鍒?*/
        pid_of_vx.integral = 0.0f;
        pid_of_vy.integral = 0.0f;
        return;
    }

    /* 楂樺害鏈夋晥锛岄噸缃棤鏁堝抚璁℃暟锛屼笅娆″け鏁堥噸鏂板紑濮嬭鏁?*/
    of_invalid_streak = 0;

    /* ======== 绗憽姝ワ細璇诲彇浼犳劅鍣ㄥ師濮嬫暟鎹?========
     * pmw3901_get_motion() 閫氳繃 SPI 璇诲彇 6 瀛楄妭锛?     * 瑙ｆ瀽鍑?pmw3901_delta_x 鍜?pmw3901_delta_y锛坕nt16 绫诲瀷锛屾湁绗﹀彿鍍忕礌澧為噺锛?     * 娉ㄦ剰锛氳鍑芥暟鍐呴儴鏈夌疮绉Н鍒嗭紙pmw3901_delta_x_i锛夛紝鏈ā鍧椾笉浣跨敤 */
    pmw3901_get_motion();
    of_dbg_dx = pmw3901_delta_x;
    of_dbg_dy = pmw3901_delta_y;
    if (pmw3901_delta_x == 0 && pmw3901_delta_y == 0)
    {
        if (of_zero_streak < 1000) of_zero_streak++;
    }
    else
    {
        of_zero_streak = 0;
    }

    /* ======== 绗憿姝ワ細鍍忕礌澧為噺 鈫?鐗╃悊閫熷害鎹㈢畻 ========
     *
     * 鍧愭爣鏄犲皠鍏崇郴锛堟牴鎹墜鎸佸钩绉诲疄娴嬩慨姝ｏ級锛?     *   PMW3901 鐨?raw_x 鈫?椋炴満鍓嶅悗鏂瑰悜 鈫?of_vel_x锛堜刊浠拌酱锛?     *   PMW3901 鐨?raw_y 鈫?椋炴満宸﹀彸鏂瑰悜 鈫?of_vel_y锛堟í婊氳酱锛?     *
     * OF_VEL_X_SIGN / OF_VEL_Y_SIGN 鐢ㄤ簬鏈€缁堟牎姝ｇ鍙锋柟鍚戙€?     */
    {
        const float raw_x = (float)pmw3901_delta_x;   // int16 鈫?float
        const float raw_y = (float)pmw3901_delta_y;

#if OF_USE_RAD_MODEL
        /* 鏂规B锛氳閫熷害妯″瀷
         * scale = 0.0244 rad/count 梅 0.02 s 脳 height_cm
         * 鍗曚綅锛歳ad/count 脳 count/s 脳 cm = cm/s
         * 閫熷害 = 鍍忕礌澧為噺 脳 scale */
        const float scale = OF_PMW3901_RAD_PER_COUNT / OF_UPDATE_DT_S * height_cm;
        of_raw_vel_x = OF_VEL_X_SIGN * (raw_x * scale);
        of_raw_vel_y = OF_VEL_Y_SIGN * (raw_y * scale);
#else
        /* 鏂规A锛氱粡楠岀郴鏁版ā鍨嬶紙褰撳墠榛樿锛?         * scale = 0.0105 cm/count/cm 脳 height_cm 梅 0.02 s
         * 鍚箟锛氬湪 height_cm 楂樺害涓嬶紝姣?count 瀵瑰簲鐨勭墿鐞嗛€熷害 (cm/s)
         * 閫熷害 = 鍍忕礌澧為噺 脳 scale */
        const float scale = OF_COUNT_TO_CMS_PER_CM * height_cm / OF_UPDATE_DT_S;
        of_raw_vel_x = OF_VEL_X_SIGN * (raw_x * scale);
        of_raw_vel_y = OF_VEL_Y_SIGN * (raw_y * scale);
#endif
        of_dbg_raw_vx = of_raw_vel_x;
        of_dbg_raw_vy = of_raw_vel_y;
    }

    /* ======== 绗懀姝ワ細璺冲彉鍓旈櫎 ========
     * PMW3901 鍦ㄧ汗鐞嗙己澶憋紙绾櫧/绾粦鍦伴潰锛夈€佸厜鐓х獊鍙樸€佹垨楂樺害绐佸彉鏃?     * 鍙兘杈撳嚭寮傚父澶х殑鍍忕礌澧為噺銆傚鏋滀笉杩囨护锛屼綆閫氭护娉㈠櫒浼氳"姹℃煋"锛?     * 瀵艰嚧鍚庣画澶氬抚閫熷害閮藉亸澶э紝浜х敓濮挎€佹姈鍔ㄣ€?     *
     * 澶勭悊鏂规硶锛氳秴杩?OF_MAX_VEL_CMS (300 cm/s) 鐨勫抚锛岀敤涓婁竴甯х殑婊ゆ尝鍊兼浛浠ｃ€?     * 杩欑浉褰撲簬"杩欏抚鏁版嵁涓嶅彲淇★紝娌跨敤涓婁竴甯х殑缁撴灉"銆?     */
    if (fabsf(of_raw_vel_x) > OF_MAX_VEL_CMS) of_raw_vel_x = of_lpf_vel_x;
    if (fabsf(of_raw_vel_y) > OF_MAX_VEL_CMS) of_raw_vel_y = of_lpf_vel_y;

    /* ======== 绗懁姝ワ細涓€闃朵綆閫氭护娉?========
     * 鍏紡锛歰utput = output + alpha 脳 (input - output)
     * 绛変环浜庯細output = alpha 脳 input + (1-alpha) 脳 last_output
     *
     * alpha = 0.35 鏃剁殑棰戠巼鐗规€э細
     *   - 浣庨淇″彿锛堢湡瀹炶繍鍔紝<2Hz锛夛細鍑犱箮鏃犺“鍑忓湴閫氳繃
     *   - 楂橀淇″彿锛堥渿鍔ㄥ櫔澹帮紝>10Hz锛夛細琛板噺绾?70%
     *   - 寤惰繜绾?1~2 甯э紙20~40ms锛夛紝鍦ㄥ彲鎺ュ彈鑼冨洿
     *
     * 濡傛灉椋炶鏃跺彂鐜伴€熷害鍝嶅簲澶參锛堥鏈洪浜嗘墠寮€濮嬩慨姝ｄ絾淇婊炲悗锛夛紝
     * 鍙互鎶?alpha 浠?0.35 澧炲ぇ鍒?0.5锛涘鏋滃櫔澹板お澶у氨鍑忓皬鍒?0.2銆?     */
    of_lpf_vel_x += OF_VEL_LPF_ALPHA * (of_raw_vel_x - of_lpf_vel_x);
    of_lpf_vel_y += OF_VEL_LPF_ALPHA * (of_raw_vel_y - of_lpf_vel_y);

    /* 灏嗘护娉㈢粨鏋滆祴缁欏澶栧彉閲忥紝渚?main 涓殑涓插彛鎵撳嵃鍜屽悗缁?PI 浣跨敤 */
    of_vel_x = of_lpf_vel_x;
    of_vel_y = of_lpf_vel_y;
    of_data_valid = 1;
}


/* ==================== Optical_Flow_Speed_Damp ====================
 *
 * 鍔熻兘锛氬熀浜庨€熷害璇樊璁＄畻濮挎€佷慨姝ｈ锛圥I 鎺у埗鍣級
 * 璋冪敤鏃舵満锛氭瘡 20ms锛岀揣璺熷湪 Optical_Flow_Update 涔嬪悗
 *
 * 鎺у埗閫昏緫锛? *   鐩爣閫熷害 = 0锛堟偓鍋滀笉鍔級
 *   璇樊 = 鐩爣閫熷害 - 瀹為檯閫熷害 = -of_vel_x
 *   淇瑙?= kp 脳 璇樊 + ki 脳 绉垎
 *
 * 涓句緥锛? *   椋炴満浠?10 cm/s 鍚戝墠椋橈紙of_vel_x = 10锛? *   璇樊 = -10
 *   淇瑙?= 0.035 脳 (-10) = -0.35 搴? *   鈫?閫氳繃 pitch 淇浜х敓鍙嶅悜鍔犻€熷害锛岀敤鏉ユ姷娑堝墠鍚庢紓绉? *
 * @param dt         璋冪敤鍛ㄦ湡锛堢锛夛紝閫氬父浼?OF_UPDATE_DT_S (0.02f)
 * @param pitch_corr [out] 淇话淇瑙掞紙搴︼級锛岃緭鍑虹粰濮挎€佸鐜? * @param roll_corr  [out] 妯粴淇瑙掞紙搴︼級锛岃緭鍑虹粰濮挎€佸鐜? */
void Optical_Flow_Speed_Damp(float dt, float *pitch_corr, float *roll_corr)
{
    /* 瀹夊叏妫€鏌ワ細鎸囬拡涓虹┖鏃剁洿鎺ヨ繑鍥烇紝涓嶅仛浠讳綍鎿嶄綔 */
    if (pitch_corr == NULL || roll_corr == NULL) return;

    /* 鍏堣緭鍑洪浂鍊硷紝鍚庣画濡傛灉鏈夊紓甯哥洿鎺ヨ繑鍥炰篃涓嶄細杈撳嚭鑴忔暟鎹?*/
    *pitch_corr = 0.0f;
    *roll_corr = 0.0f;

    /* 鍏夋祦鏁版嵁鏃犳晥鏃讹紙楂樺害瓒呴檺 / 浼犳劅鍣ㄥ紓甯革級锛屼笉杈撳嚭浠讳綍淇閲?     * 骞舵竻闆剁Н鍒嗭紝闃叉鏃犳晥鏁版嵁涓婄殑绉垎鍦ㄦ仮澶嶅悗閫犳垚濮挎€佸啿鍑?*/
    if (!of_data_valid)
    {
        pid_of_vx.integral = 0.0f;
        pid_of_vy.integral = 0.0f;
        return;
    }

    /* ======== 閫熷害璇樊璁＄畻 ========
     * 鐩爣閫熷害鎭掍负 0锛堟偓鍋滐級锛屾墍浠ヨ宸?= 0 - 褰撳墠閫熷害 = -褰撳墠閫熷害
     * err_vx > 0 琛ㄧず椋炴満鍦ㄥ悜 X 璐熸柟鍚戦锛岄渶瑕佸悜 X 姝ｆ柟鍚戜慨姝?     * err_vy > 0 琛ㄧず椋炴満鍦ㄥ悜 Y 璐熸柟鍚戦锛岄渶瑕佸悜 Y 姝ｆ柟鍚戜慨姝?*/
    {
        const float err_vx = -of_vel_x;
        const float err_vy = -of_vel_y;

        /* ======== 绉垎椤癸紙褰撳墠鍏抽棴锛宬i=0锛?========
         * 绉垎鍒嗙锛氬彧鍦ㄨ宸皬浜?OF_INT_ERR_GATE_CMS (120 cm/s) 鏃舵墠绱Н绉垎銆?         * 鐩殑锛氶鏈哄揩閫熺Щ鍔ㄦ椂锛堣宸ぇ锛変笉绱Н绉垎锛屽彧鏈夋帴杩戞偓鍋滅姸鎬侊紙璇樊灏忥級
         *       鏃舵墠鐢ㄧН鍒嗘秷闄ょǔ鎬佽宸紝闃叉绉垎鏆磋蛋锛坅nti-windup锛夈€?         *
         * i_limit 闄愬埗浜嗙Н鍒嗘渶澶у€硷紝鐩稿綋浜庨檺鍒朵簡绉垎椤圭殑鏈€澶ц础鐚噺銆?         * 褰撳墠 ki=0锛岃繖娈典唬鐮佷笉浼氱敓鏁堬紝浣嗗凡缁忓啓濂斤紝寮€鍚Н鍒嗘椂鏃犻渶鏀瑰姩銆?*/
        if (fabsf(err_vx) < OF_INT_ERR_GATE_CMS)
        {
            pid_of_vx.integral += err_vx * dt;
            pid_of_vx.integral = f_limit(pid_of_vx.integral, -pid_of_vx.i_limit, pid_of_vx.i_limit);
        }
        if (fabsf(err_vy) < OF_INT_ERR_GATE_CMS)
        {
            pid_of_vy.integral += err_vy * dt;
            pid_of_vy.integral = f_limit(pid_of_vy.integral, -pid_of_vy.i_limit, pid_of_vy.i_limit);
        }

        /* ======== PI 杈撳嚭璁＄畻 ========
         * out = kp 脳 err + ki 脳 integral
         * 鐒跺悗闄愬箙鍒?卤OF_ANGLE_LIMIT锛? 搴︼級锛岄槻姝㈠厜娴佹ā鍧楄緭鍑鸿繃澶т慨姝ｉ噺
         * 骞叉壈姝ｅ父濮挎€佹帶鍒讹紙璧烽闃舵灏や负閲嶈锛?*/
        {
            float out_pitch = pid_of_vx.kp * err_vx + pid_of_vx.ki * pid_of_vx.integral;
            float out_roll = pid_of_vy.kp * err_vy + pid_of_vy.ki * pid_of_vy.integral;

            out_pitch = f_limit(out_pitch, -OF_ANGLE_LIMIT, OF_ANGLE_LIMIT);
            out_roll = f_limit(out_roll, -OF_ANGLE_LIMIT, OF_ANGLE_LIMIT);

            /* 鏈€鍚庝箻浠ョ鍙风郴鏁帮紝琛ュ伩"閫熷害鈫掑Э鎬佽"鐨勬槧灏勬柟鍚?*/
            *pitch_corr = OF_CTRL_PITCH_SIGN * out_pitch;
            *roll_corr = OF_CTRL_ROLL_SIGN * out_roll;
        }
    }
}


/* ==================== Optical_Flow_Reset ====================
 *
 * 鍔熻兘锛氭竻闆跺叏閮ㄥ厜娴佺姸鎬侊紙閫熷害 + 婊ゆ尝鍣?+ 绉垎锛? * 璋冪敤鏃舵満锛? *   - 钀藉湴鏃讹紙flight_state 鍒囧洖 IDLE锛? *   - 鎬ュ仠/鍧犳満淇濇姢瑙﹀彂鏃? *   - 涓嬫璧烽鍓? *
 * 涓轰粈涔堜笉鑳藉彧娓?of_vel_x/y锛? *   鍥犱负 LPF 婊ゆ尝鍣紙of_lpf_vel_x/y锛夐噷杩樹繚瀛樼潃涓婁竴娈电殑閫熷害鍘嗗彶锛? *   濡傛灉涓嶆竻锛屼笅娆¤捣椋炴椂鍓嶅嚑甯х殑婊ゆ尝杈撳嚭浼氬甫鍏ユ棫鐨勯€熷害鍋忕疆锛? *   瀵艰嚧椋炴満鍒氳捣椋炲氨鍚戞煇涓柟鍚戝€炬枩銆? */
void Optical_Flow_Reset(void)
{
    of_vel_x = 0.0f;
    of_vel_y = 0.0f;
    of_raw_vel_x = 0.0f;
    of_raw_vel_y = 0.0f;
    of_lpf_vel_x = 0.0f;
    of_lpf_vel_y = 0.0f;
    of_invalid_streak = 0;
    of_dbg_dx = 0;
    of_dbg_dy = 0;
    of_dbg_raw_vx = 0.0f;
    of_dbg_raw_vy = 0.0f;
    of_zero_streak = 0;
    of_data_valid = 0;

    pid_of_vx.integral = 0.0f;
    pid_of_vy.integral = 0.0f;
}
#endif
