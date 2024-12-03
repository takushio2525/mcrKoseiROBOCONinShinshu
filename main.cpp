
//------------------------------------------------------------------//
// Supported MCU:   RZ/A1H
// File Contents:   kit_kirokukai2021_gr_peach ｸﾛｽﾗｲﾝ,左右ﾊｰﾌﾗｲﾝあり版
// Version number:  Ver.1.00
// Date:            2020.09.08
// Copyright:       Renesas Electronics Corporation
//                 Hitachi Document Solutions Co., Ltd.
//                 宮崎県高等学校教育研究会　工業部会

//                 　　　マイコンカーラリー指導担当代表者会
//------------------------------------------------------------------//

//------------------------------------------------------------------//
// Include
//------------------------------------------------------------------//
#include "DisplayBace.h"
#include "Encoder.hpp"
#include "SdUsbConnect.h"
#include "UnbufferedSerial.h"
#include "image_process.h"
#include "iodefine.h"
#include "mbed.h"
#include <cstdio>
#include <exception>
//------------------------------------------------------------------//
// Define
//------------------------------------------------------------------//
// Motor PWM cycle
#define MOTOR_PWM_CYCLE \
    33332 // Motor PWM period
          // 1ms    P0φ/1  = 0.03us
// Servo PWM cycle
#define SERVO_PWM_CYCLE \
    33332 // SERVO PWM period
          // 16ms   P0φ/16 = 0.48us
#define SERVO_CENTER \
    3020 // 3070        // 1.5ms / 0.48us - 1 = 3124   最初３０５０
// 値を足す　減らすと左
// 3100
#define HANDLE_STEP 18 // 1 degree value

//------------------------------------------------------------------//
// マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効)
//------------------------------------------------------------------//
#define MASK2_2 0x66 // ×○○××○○×
#define MASK2_0 0x60 // ×○○×××××
#define MASK0_2 0x06 // ×××××○○×
#define MASK3_3 0xe7 // ○○○××○○○
#define MASK0_3 0x07 // ×××××○○○
#define MASK3_0 0xe0 // ○○○×××××
#define MASK4_0 0xf0 // ○○○○××××
#define MASK0_4 0x0f // ××××○○○○
#define MASK4_4 0xff // ○○○○○○○○

//------------------------------------------------------------------//
// Define(NTSC-Video)
//------------------------------------------------------------------//
#define VIDEO_INPUT_CH (DisplayBase::VIDEO_INPUT_CHANNEL_0)
#define VIDEO_INT_TYPE (DisplayBase::INT_TYPE_S0_VFIELD)
#define DATA_SIZE_PER_PIC (2u)

//! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
//  in accordance with the frame buffer burst transfer mode.
#define PIXEL_HW (160u) // QVGA
#define PIXEL_VW (120u) // QVGA
#define VIDEO_BUFFER_STRIDE (((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define VIDEO_BUFFER_HEIGHT (PIXEL_VW)

//------------------------------------------------------------------//
// ここから自作defineを追加
//------------------------------------------------------------------//

#define LOG_NUM (4000u) // ログ保存数

#define IMAGE_WIDTH 160                    // 画像のX列数
#define IMAGE_HEIGHT 120                   // 画像のY行数
#define IMAGE_CENTER IMAGE_WIDTH / 2       // 画像の中心
#define IMAGE_RIGHT_EDGE IMAGE_WIDTH - 1   // 画像の右端
#define IMAGE_BOTTOM_EDGE IMAGE_HEIGHT - 1 // 画像の下端

#define MAX_MOTOR_POWER 100

#define LeftBorder 50
#define RightBorder -50

#define CrossRow 40 // クロスラインを見つけるために使う行

#define CrossTop 20     // クロスライン検出範囲　トップ
#define CrossBottom 100 // クロスライン検出範囲　ボトム

//------------------------------------------------------------------//
// Constructor
//------------------------------------------------------------------//
// Create DisplayBase object
DisplayBase Display;

Ticker interrput;
UnbufferedSerial pc(USBTX, USBRX);

DigitalOut LED_R(P6_13);     // LED1 on the GR-PEACH board
DigitalOut LED_G(P6_14);     // LED2 on the GR-PEACH board
DigitalOut LED_B(P6_15);     // LED3 on the GR-PEACH board
DigitalOut USER_LED(P6_12);  // USER_LED on the GR-PEACH board
DigitalIn user_botton(P6_0); // SW1 on the GR-PEACH board

BusIn dipsw(P7_15, P8_1, P2_9, P2_10); // SW1 on Shield board

DigitalOut Left_motor_signal(P4_6);  // Used by motor function
DigitalOut Right_motor_signal(P4_7); // Used by motor function
DigitalIn push_sw(P2_13);            // SW1 on the Motor Drive board
DigitalOut LED_3(P2_14);             // LED3 on the Motor Drive board
DigitalOut LED_2(P2_15);             // LED2 on the Motor Drive board

Encoder encoder;

//------------------------------------------------------------------//
// Prototype
//------------------------------------------------------------------//
void init_Camera(void);
void ChangeFrameBuffer(void);
// void kensyutu(void);

static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type);
static void WaitVfield(const int32_t wait_count);
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type);
static void WaitVsync(const int32_t wait_count);
void init_MTU2_PWM_Motor(void); // Initialize PWM functions
void init_MTU2_PWM_Servo(void); // Initialize PWM functions
void intTimer(void);            // 1ms period
unsigned char user_button_get(void);
void led_m(int time, int r, int g, int b);
void led_m_process(void); // Only function for interrupt

unsigned char sensor_inp(unsigned char mask);
int check_crossline(void);
int check_rightline(void);
int check_leftline(void);
void led_out(int led);
unsigned char pushsw_get(void);
void motor(int accele_l, int accele_r);
void handle(int angle);
unsigned char dipsw_get(void);
unsigned char shikiichi_henkan(int gyou, int s, int sa);
char getImage(int ix, int iy);

int getCompileYear(const char *p);
int getCompileMonth(const char *p);
int getCompileDay(const char *p);
int getCompileHour(const char *p);
int getCompilerMinute(const char *p);
int getCompilerSecond(const char *p);
unsigned long convertBCD_CharToLong(unsigned char hex);

//------------------------------------------------------------------//
// ここから自作関数のプロトタイプ宣言を追加
//------------------------------------------------------------------//
void easyCreateDeviation(int rowNum);
void createLineFlag(int rowNum); // ラインを検出する行数); // ラインフラグを生成する関数　　    switch (counter++)内で画像更新につき一回ごと実行
void createDeviation(void);

void createHandleVal(void);
void createMotorVal(void);
void createBrakeMotorVal(int targetSpeed);

// void crossKensyutu(int LineRow);

// void crank();

//------------------------------------------------------------------//
// Global variable (NTSC-video)
//------------------------------------------------------------------//
static uint8_t FrameBuffer_Video_A[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT] __attribute((section("NC_BSS"), aligned(16))); // 16 bytes aligned!;
static uint8_t FrameBuffer_Video_B[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT] __attribute((section("NC_BSS"), aligned(16))); // 16 bytes aligned!;
uint8_t *write_buff_addr = FrameBuffer_Video_A;
uint8_t *save_buff_addr = FrameBuffer_Video_B;
static volatile int32_t vsync_count;
static volatile int32_t vfield_count;
static volatile int32_t vfield_count2 = 1;
static volatile int32_t vfield_count2_buff;

//------------------------------------------------------------------//
// Global variable for Image process
//------------------------------------------------------------------//
unsigned char ImageData_A[((PIXEL_HW * 2) * PIXEL_VW)];
unsigned char ImageData_B[(PIXEL_HW * PIXEL_VW)];
unsigned char ImageComp_B[(PIXEL_HW * PIXEL_VW)];
unsigned char ImageBinary[(PIXEL_HW * PIXEL_VW)];

//------------------------------------------------------------------//
// Global variable for Trace program
//------------------------------------------------------------------//
const char *C_DATE = __DATE__; // コンパイルした日付
const char *C_TIME = __TIME__; // コンパイルした時間
const char monthStr[] = {"JanFebMarAprMayJunJulAugSepOctNovDec"};
// 月変換テーブル

volatile unsigned long cnt1; // Used within main
volatile unsigned long cnt_printf;
volatile unsigned long cnt_msd;
volatile unsigned long cnt_msdwritetime;
volatile unsigned long cnt_debug;

volatile int pattern;      // Pattern numbers
volatile int initFlag = 1; // Initialize flag

volatile int mled_time, mled_r, mled_g, mled_b;
volatile int debug_mode;

FILE *fp;
struct tm t; // microSDの日付用
volatile unsigned char sensor_bin;
volatile unsigned char bar;
volatile int log_pattern = 901;
volatile int log_mode; // 0:待機 1:ファイルオープン
                       // 2:ログ記録中 3:ログファイルクローズ
                       // ※クローズしないとファイルが保存されない
volatile int msdError;
volatile int msd_handle, msd_l, msd_r;

//------------------------------------------------------------------//
// ここから自作のグローバル変数を追加
//------------------------------------------------------------------//

volatile signed int deviationDifference;

volatile int easyImageData[IMAGE_WIDTH];
volatile int easyDifference[IMAGE_WIDTH];
volatile int easyDeviation;
volatile int easyDifferencePoint[IMAGE_WIDTH];

volatile bool endflag = false;

volatile bool lineflag_center = false;
volatile bool lineflag_left = false;
volatile bool lineflag_right = false;
volatile bool lineflag_cross = false;

volatile signed int allDeviation[IMAGE_HEIGHT];

volatile signed int leftDeviation[IMAGE_WIDTH];
volatile signed int rightDeviation[IMAGE_WIDTH];
volatile signed int difference[IMAGE_HEIGHT][IMAGE_WIDTH];

volatile signed int encoderAcceleration;
volatile signed int lastEncoderSpeed;

volatile int handleVal;

volatile int leftMotor;
volatile int rightMotor;

volatile int leftBrakeMotor;
volatile int rightBrakeMotor;

volatile int crankMotorPowerIN;
volatile int crankMotorPowerOUT;
volatile int crankHandleVal;

volatile int laneMotorPowerLeft;
volatile int laneMotorPowerRight;
volatile int laneHandleVal;

volatile int laneCounterMotorPowerLeft;
volatile int laneCounterMotorPowerRight;
volatile int laneCounterHandleVal;

volatile int laneStraightMotorPower;

volatile int lineSkipDistance;

volatile int crankDistance;
volatile int laneDistance;
volatile int laneAfterDistance;
volatile int laneCounterDistance;
volatile bool resetFlag = true;

volatile int imgArray[IMAGE_HEIGHT][IMAGE_WIDTH]; // 画像データを入れる配列
volatile int LeftLineArray[IMAGE_HEIGHT];         // 中心線の左境界座標
volatile int RightLineArray[IMAGE_HEIGHT];
volatile int CenterLineArray[IMAGE_HEIGHT];
volatile int firstMiddle = 0; // 最下層の中心線の左境界座標

volatile int crankTimer;
volatile int crankHandleRight;
volatile int crankHandleLeft;

volatile bool RightX = false;
volatile bool LeftX = false;
volatile bool Cross = false;

// volatile bool laneChange = false;
// volatile bool Rightcrank = false;
// volatile bool Leftcrank = false;

typedef struct
{
    unsigned int cnt_msdwritetime;
    unsigned int pattern;
    unsigned int convertBCD;
    signed int handle;
    signed int hennsa;
    unsigned int encoder;
    signed int motorL;
    signed int motorR;
    signed int flagL;
    signed int flagK;
    signed int flagR;
    signed int flagS;
    signed int total;

    // int senn(int linegyou)関数内で直接取得
    // ログ取得用(生データ)
    signed int imageData0[160]; // 白線検出時に使用する生データ
    // signed int imageData1[160]; // 白線検出時に使用する生データ

    // signed int imageData2[160]; // 白線検出時に使用する生データ

} SDLOG_T;

SDLOG_T log_data[LOG_NUM]; // ログ保存データ
unsigned int log_no = 0;   // ログの記録番地（時間）

static int imgDeviation[IMAGE_HEIGHT][IMAGE_WIDTH];
static int ExecutionFlag = false;

static int minDefference = 500;

int HandleNum;

///****************************************************************
// Main function
///****************************************************************
int main(void)
{
    int x, y, i;
    char moji_work[128], c;
    int sw_now = 0, sw_before = -1;

    initFlag = 1; // Initialization start

    // 起動時にボタンが押されているとデバッグモードON
    if (user_button_get() != 0)
    {
        debug_mode = 1;
    }

    // Camera start
    init_Camera();
    // wait to stabilize NTSC signal (about 170ms)
    ThisThread::sleep_for(200ms);

    // Initialize MCU functions
    init_MTU2_PWM_Motor();
    init_MTU2_PWM_Servo();
    encoder.init();
    interrput.attach(&intTimer, 1ms);
    pc.baud(230400);
    printf("\033[H"); // デバッグ用画面クリア
    printf("\033[2J");

    // Initialize Micon Car state
    handle(0);
    motor(0, 0);
    led_out(0x0);
    led_m(10, 0, 1, 1); // 10% R=0,G=1,B=1

    t.tm_sec = getCompilerSecond(C_TIME);      // 0-59
    t.tm_min = getCompilerMinute(C_TIME);      // 0-59
    t.tm_hour = getCompileHour(C_TIME);        // 0-23
    t.tm_mday = getCompileDay(C_DATE);         // 1-31
    t.tm_mon = getCompileMonth(C_DATE) - 1;    // 0-11 +1する
    t.tm_year = getCompileYear(C_DATE) - 1900; // -1900する
    time_t seconds = mktime(&t);
    set_time(seconds);

    SdUsbConnect storage("storage");
    // storage.wait_connect(); // 認識するまでここで止まってしまうのでこの命令は使えない
    cnt_msd = 0;
    while (storage.connect() == 0)
    { // STORAGE_NON = 0
        ThisThread::sleep_for(100ms);
        if (cnt_msd >= 1000)
        { // この時間以上microSDの接続が確認できなければエラー
            msdError = 1;
            break;
        }
    }
    if (msdError == 1)
    {
        led_m(50, 1, 0, 0);
        cnt_msd = 0;
        while (cnt_msd < 2000)
        { // 2s待つ
        }
    }

    initFlag = 0; // Initialization end

    // Debug Program
    if (debug_mode == 1)
    {
        led_m(10, 0, 1, 1);

        while (1)
        {
            // LEDにモニターする
            led_out((sensor_inp(0x80) >> 6) | sensor_inp(0x01));
            USER_LED = sensor_inp(0x18) != 0x00 ? 1 : 0;

            // ユーザーボタンを(長めに)押すとデバッグ画面を切り換える
            if (cnt_debug >= 10)
            {
                cnt_debug = 0;
                sw_now = user_button_get();
                if (sw_now == 1 && sw_before == 0)
                {
                    debug_mode++;
                    if (debug_mode >= 5)
                    {
                        debug_mode = 1;
                    }
                    printf("\033[H");
                    printf("\033[2J"); // 画面クリア
                }
                sw_before = sw_now;
            }

            if (cnt_printf >= 200)
            {
                cnt_printf = 0;

                switch (debug_mode)
                {
                case 1:
                    // 補正値で表示 しきい値180以上を"1" 180を変えると、しきい値が変わる
                    // printf("shikii chi 180\r\n");
                    // for (y = 0; y < 30; y++)
                    // {
                    //     printf("%3d:%08ld ", y + 0, convertBCD_CharToLong(shikiichi_henkan(y + 0, 180, 8)));
                    //     printf("%3d:%08ld ", y + 30, convertBCD_CharToLong(shikiichi_henkan(y + 30, 180, 8)));
                    //     printf("%3d:%08ld ", y + 60, convertBCD_CharToLong(shikiichi_henkan(y + 60, 180, 8)));
                    //     printf("%3d:%08ld ", y + 90, convertBCD_CharToLong(shikiichi_henkan(y + 90, 180, 8)));
                    //     printf("\r\n");
                    // }
                    for (x = 0; x < 160; x++)
                    {
                        c = difference[60][x] < -7 ? 1 : 0; // 180を変えるとしきい値が変わる
                        if (difference[60][x] < -7)
                        {
                            printf("\x1b[44m%d\x1b[49m", c);
                        }
                        else
                        {
                            printf("%d", c);
                        }
                    }
                    printf("\033[H");
                    break;

                case 2:
                    printf("%d", RightX);
                    break;

                case 3:
                    // https://www.sejuku.net/blog/24934
                    // 文字の色
                    // \x1b[30m 黒 \x1b[31m 赤 \x1b[32m 緑 \x1b[33m 黄
                    // \x1b[34m 青 \x1b[35m マゼンダ \x1b[36m シアン
                    // \x1b[37m 白 \x1b[39m デフォルトに戻す
                    // 背景色の指定
                    // \x1b[40m 黒 \x1b[41m 赤 \x1b[42m 緑 \x1b[43m 黄
                    // \x1b[44m 青 \x1b[45m マゼンダ \x1b[46m シアン
                    // \x1b[47m 灰 \x1b[49m デフォルトに戻す

                    // 1行飛ばしで表示(しきい値180以上を"1"とする)
                    printf("shi 0         0         0         0         0         0         0         0         0         0         1         1         1         1         1         1        1\r\n");
                    printf("kii 0         1         2         3         4         5         6         7         8         9         0         1         2         3         4         5        5\r\n");
                    printf("180 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\r\n");
                    for (y = 40; y < 120; y += 2)
                    {
                        printf("%03d:", y);
                        for (x = 0; x < 160; x++)
                        {
                            c = getImage(x, y) >= 200 ? 1 : 0; // 180を変えるとしきい値が変わる
                            if (x == -allDeviation[y] + 80)
                            {
                                printf("\x1b[43m%d\x1b[49m", c);
                            }
                            else if (x == -leftDeviation[y] + 80)
                            {
                                printf("\x1b[44m%d\x1b[49m", c);
                            }
                            else if (x == -rightDeviation[y] + 80)
                            {
                                printf("\x1b[41m%d\x1b[49m", c);
                            }
                            else
                            {
                                printf("%d", c);
                            }
                        }
                        printf("  \r\n");
                    }
                    printf("\033[H");
                    break;

                case 4:
                    printf("shi 0         0         0         0         0         0         0         0         0         0         1         1         1         1         1         1        1\r\n");
                    printf("kii 0         1         2         3         4         5         6         7         8         9         0         1         2         3         4         5        5\r\n");
                    printf("180 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\r\n");
                    for (y = 40; y < 120; y += 2)
                    {
                        printf("%03d:", y);
                        for (x = 0; x < 160; x++)
                        {
                            c = getImage(x, y) >= 200 ? 1 : 0; // 180を変えるとしきい値が変わる
                            if (x == LeftLineArray[y] + 80)    // 黄色
                            {
                                printf("\x1b[43m%d\x1b[49m", c);
                            }
                            else if (x == RightLineArray[y] + 80)
                            {
                                printf("\x1b[44m%d\x1b[49m", c);
                            }
                            else if (x == CenterLineArray[y] + 80)
                            {
                                printf("\x1b[41m%d\x1b[49m", c);
                            }

                            else
                            {
                                printf("%d", c);
                            }
                        }
                        printf("  \r\n");
                    }
                    printf("\033[H");
                    break;
                }
            }
        }
    }

    // 通常走行
    // ターミナルに出力とmicroSDへログ保存
    // 走行プログラムは、intTimer関数(1msごとの割り込み)で行う
    while (1)
    {

        // ログ保存処理
        if (endflag == true)
        {
            led_m(50, 1, 0, 0); // スタートバーセットOK状態→緑色点灯

            i = 0;
            if ((fp = fopen("/storage/renban.txt", "r")) != NULL)
            {
                fscanf(fp, "%d", &i);
                fclose(fp);
            }
            if (i < 0 || i > 9999)
            {
                i = 0;
            }
            if ((fp = fopen("/storage/renban.txt", "w")) != NULL)
            {
                fprintf(fp, "%d", i + 1);
                fclose(fp);
            }
            sprintf(moji_work, "/storage/data%04d.csv", i);
            if ((fp = fopen(moji_work, "w")) != NULL)
            {
                fprintf(fp, "Log %d\n", i);
            }

            for (unsigned int i = 0; i < log_no; i++)
            {
                fprintf(fp, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                        log_data[i].cnt_msdwritetime,
                        log_data[i].pattern,
                        log_data[i].convertBCD,
                        log_data[i].handle,
                        log_data[i].hennsa,
                        log_data[i].encoder,
                        log_data[i].motorL,
                        log_data[i].motorR,
                        log_data[i].flagL,
                        log_data[i].flagK,
                        log_data[i].flagR,
                        log_data[i].flagS,
                        log_data[i].total);

                // ログ取得用(生データ)
                // for (int u = 0; u < 3; u++)
                // {
                for (int t = 0; t < 160; t++)
                {
                    fprintf(fp, "%d,", log_data[i].imageData0[t]);
                }

                fprintf(fp, "\n");

                // }
            }
            //         // microSDに保存する内容
            //         // 文字数に制限はないが、増やしすぎると10msごとにならない
            // //        fprintf(fp, "%d,%d,b%08ld,%d,%d,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            // //                int(cnt_msdwritetime % 1000), pattern,
            // //                convertBCD_CharToLong(sensor_inp(0xff)), msd_handle, ALL_hennsa[45],
            // //                encoder.getCnt(), msd_l, msd_r);
            // //    }
            fclose(fp); // fcloseしないと保存されない
            while (true)
            {
                led_m(50, 0, 0, 1); // スタートバーセットOK状態→緑色点灯
            }
        }
    }
}

///****************************************************************
// Initialize functions
///****************************************************************
//------------------------------------------------------------------//
// Initialize MTU2 PWM functions
//------------------------------------------------------------------//
// MTU2_3, MTU2_4
// Reset-Synchronized PWM mode
// TIOC4A(P4_4) :Left-motor
// TIOC4B(P4_5) :Right-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Motor(void)
{
    // Port setting for S/W I/O Control
    // alternative mode

    // MTU2_4 (P4_4)(P4_5)
    GPIOPBDC4 = 0x0000;   // Bidirection mode disabled
    GPIOPFCAE4 &= 0xffcf; // The alternative function of a pin
    GPIOPFCE4 |= 0x0030;  // The alternative function of a pin
    GPIOPFC4 &= 0xffcf;   // The alternative function of a pin
                          // 2nd altemative function/output
    GPIOP4 &= 0xffcf;     //
    GPIOPM4 &= 0xffcf;    // p4_4,P4_5:output
    GPIOPMC4 |= 0x0030;   // P4_4,P4_5:double

    // Module stop 33(MTU2) canceling
    CPGSTBCR3 &= 0xf7;

    // MTU2_3 and MTU2_4 (Motor PWM)
    MTU2TCR_3 = 0x20;            // TCNT Clear(TGRA), P0φ/1
    MTU2TOCR1 = 0x04;            //
    MTU2TOCR2 = 0x40;            // N L>H  P H>L
    MTU2TMDR_3 = 0x38;           // Buff:ON Reset-Synchronized PWM mode
    MTU2TMDR_4 = 0x30;           // Buff:ON
    MTU2TOER = 0xc6;             // TIOC3B,4A,4B enabled output
    MTU2TCNT_3 = MTU2TCNT_4 = 0; // TCNT3,TCNT4 Set 0
    MTU2TGRA_3 = MTU2TGRC_3 = MOTOR_PWM_CYCLE;
    // PWM-Cycle(1ms)
    MTU2TGRA_4 = MTU2TGRC_4 = 0; // Left-motor(P4_4)
    MTU2TGRB_4 = MTU2TGRD_4 = 0; // Right-motor(P4_5)
    MTU2TSTR |= 0x40;            // TCNT_4 Start
}

//------------------------------------------------------------------//
// Initialize MTU2 PWM functions
//------------------------------------------------------------------//
// MTU2_0
// PWM mode 1
// TIOC0A(P4_0) :Servo-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Servo(void)
{
    // Port setting for S/W I/O Control
    // alternative mode

    // MTU2_0 (P4_0)
    GPIOPBDC4 = 0x0000;   // Bidirection mode disabled
    GPIOPFCAE4 &= 0xfffe; // The alternative function of a pin
    GPIOPFCE4 &= 0xfffe;  // The alternative function of a pin
    GPIOPFC4 |= 0x0001;   // The alternative function of a pin
                          // 2nd alternative function/output
    GPIOP4 &= 0xfffe;     //
    GPIOPM4 &= 0xfffe;    // p4_0:output
    GPIOPMC4 |= 0x0001;   // P4_0:double

    // Module stop 33(MTU2) canceling
    CPGSTBCR3 &= 0xf7;

    // MTU2_0 (Motor PWM)
    MTU2TCR_0 = 0x02;            // 22 // TCNT Clear(TGRA), P0φ/16 henkou
    MTU2TIORH_0 = 0x52;          // TGRA L>H, TGRB H>L
    MTU2TMDR_0 = 0x32;           // TGRC and TGRD = Buff-mode
                                 // PWM-mode1
    MTU2TBTM_0 = 0x03;           // バッファ動作転送は TCNT クリア時 tuika
    MTU2TCNT_0 = 0;              // TCNT0 Set 0
                                 // MTU2TGRA_0 = MTU2TGRC_0 = SERVO_PWM_CYCLE;
    MTU2TGRA_0 = MTU2TGRC_0 = 2; // SV パルス立ち上がり
    // PWM-Cycle(16ms)
    // MTU2TGRB_0 = MTU2TGRD_0 = 0; // Servo-motor(P4_0)
    MTU2TGRB_0 = MTU2TGRD_0 = SERVO_CENTER; // SV パルス立ち下がり ※３

    MTU2TSTR |= 0x01; // TCNT_0 Start
}

//------------------------------------------------------------------//
// Initialize Camera function
//------------------------------------------------------------------//
void init_Camera(void)
{
    // NTSC-Video
    DisplayBase::graphics_error_t error;

    // Graphics initialization process
    error = Display.Graphics_init(NULL);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    error = Display.Graphics_Video_init(DisplayBase::INPUT_SEL_VDEC, NULL);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        while (1)
            ;
    }

    // Interrupt callback function setting (Vsync signal input to scaler 0)
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VI_VSYNC, 0, IntCallbackFunc_Vsync);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video capture setting (progressive form fixed)
    error = Display.Video_Write_Setting(
        VIDEO_INPUT_CH,
        DisplayBase::COL_SYS_NTSC_358,
        write_buff_addr,
        VIDEO_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        PIXEL_VW,
        PIXEL_HW);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, IntCallbackFunc_Vfield);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video write process start
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video write process stop
    error = Display.Video_Stop(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video write process start
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Wait vsync to update resister
    WaitVsync(1);

    // Wait 2 Vfield(Top or bottom field)
    WaitVfield(2);
}

//------------------------------------------------------------------//
// ChangeFrameBuffer function
//------------------------------------------------------------------//
void ChangeFrameBuffer(void)
{
    // NTSC-Video
    DisplayBase::graphics_error_t error;

    // Change write buffer
    if (write_buff_addr == FrameBuffer_Video_A)
    {
        write_buff_addr = FrameBuffer_Video_B;
        save_buff_addr = FrameBuffer_Video_A;
    }
    else
    {
        write_buff_addr = FrameBuffer_Video_A;
        save_buff_addr = FrameBuffer_Video_B;
    }
    error = Display.Video_Write_Change(
        VIDEO_INPUT_CH,
        write_buff_addr,
        VIDEO_BUFFER_STRIDE);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type)
{
    (void)int_type;

    if (vfield_count > 0)
    {
        vfield_count--;
    }
    // top or bottom (Change)
    if (vfield_count2 == 0)
        vfield_count2 = 1;
    else
        vfield_count2 = 0;
}

//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void WaitVfield(const int32_t wait_count)
{
    vfield_count = wait_count;
    while (vfield_count > 0)
    {
        // Do nothing
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function for Vsync interruption
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type)
{
    (void)int_type;

    if (vsync_count > 0)
    {
        vsync_count--;
    }
}

//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void WaitVsync(const int32_t wait_count)
{
    vsync_count = wait_count;
    while (vsync_count > 0)
    {
        // Do nothing
    }
}

//------------------------------------------------------------------//
// Interrupt function( intTimer )
//------------------------------------------------------------------//
void intTimer(void)
{
    static int counter = 0; // Only variable for image process
    unsigned char b;
    int myHandleVal;

    cnt1++;
    cnt_msdwritetime++;
    cnt_printf++;
    cnt_msd++;
    cnt_debug++;
    static int beforEncoder = 0;

    // field check
    if (vfield_count2 != vfield_count2_buff)
    {
        vfield_count2_buff = vfield_count2;
        counter = 0;
    }

    // Top field / bottom field
    switch (counter++)
    {
    case 0:
        ImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2); //  0 - 59行を変換
        break;
    case 1:
        ImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2); // 60 - 119行を変換
        break;
    case 2:
        Extraction_Brightness(ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2);
        break;
    case 3:
        Extraction_Brightness(ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2);

        sensor_bin = shikiichi_henkan(60, 180, 8); //  60行目 しきい値180 隣同士の差分が8以下なら0x00にする
        // スタートバー検出 bit7とbit0のみ見る
        bar = (shikiichi_henkan(60, 120, 8) |
               shikiichi_henkan(70, 120, 8) |
               shikiichi_henkan(80, 120, 8) |
               shikiichi_henkan(90, 120, 8) |
               shikiichi_henkan(100, 120, 8)) &
              0x3c;
        break;

    case 5:
        // 画像更新ごとの更新処理
        encoder.update();
        encoderAcceleration = encoder.getCnt() - beforEncoder;
        beforEncoder = encoder.getCnt();
        createDeviation();
        createMotorVal();
        // kensyutu();
        // crossKensyutu(50);
        // crank();

        // CrossLine();

        // if (pattern != 11)
        // {
        //     createLineFlag(encoder.getCnt() + 12); // ラインを検出する行数)
        // }
        // else
        // {
        createLineFlag(45); // ラインを検出する行数)
                            // }
        createHandleVal();

        break;

    case 6:
        MTU2TCNT_0 = 0;
        break;

    case 10:

        if (endflag == false)
        {
            // ログ（RAM）記録
            log_data[log_no].cnt_msdwritetime = cnt_msdwritetime;
            log_data[log_no].pattern = pattern;
            log_data[log_no].convertBCD = deviationDifference;
            log_data[log_no].handle = msd_handle;
            log_data[log_no].hennsa = allDeviation[60]; // 偏差を検出するプログラムを作ったら追加
            log_data[log_no].encoder = encoder.getCnt();
            log_data[log_no].motorL = msd_l /*getImage(-allDeviation[60] + IMAGE_CENTER - 35, 60)*/;
            log_data[log_no].motorR = msd_r;

            log_data[log_no].flagL = lineflag_left;
            log_data[log_no].flagK = lineflag_cross;
            log_data[log_no].flagR = lineflag_right;
            log_data[log_no].flagS = lineflag_center; // センターライン検出フラグを作ったら追加

            log_data[log_no].total = encoder.getTotalCount();
            for (int i = 0; i < 160; i++)
            {
                log_data[log_no].imageData0[i] = getImage(i, 60);
            }

            log_no++;
        }
        break;

    default:
        break;
    }

    // LED(rgb) on the GR-peach board
    led_m_process(); // LEDの点滅処理を行う

    if (debug_mode != 0 || initFlag != 0)
    {
        return; // デバッグモードなら、ここで割り込み終了
                // 走行プログラムは実行しない
    }

    // モータドライブ基板のLED2個は、スタートバーの反応チェック用 スタート時にLEDが点灯するようにセットする
    led_out(((bar & 0x80) >> 6) | (bar & 0x01));

    // GR-PEACHのUSER_LED(赤色)は、sensor_inp関数の中心2個のどれかが"1"なら、点灯する
    USER_LED = sensor_inp(0x18) != 0x00 ? 1 : 0;

    if (pattern >= 11 && pattern <= 100 && user_button_get() == 1 &&
        log_mode == 2)
    {
        pattern = 101;
        log_mode = 3; // ログ保存終了
    }

    switch (pattern)
    {

    case 0:
        // スイッチ入力待ち

        // if (lineflag_cross == 1)
        // {
        led_m(50, 0, 1, 0); // スタートバーセットOK状態→緑色点灯

        if (pushsw_get() == 1)
        {
            pattern = 4;
            log_mode = 1; // ログファイルオープン
            cnt1 = 0;
            encoder.clear();
            break;
        }
        //}
        // else
        // {
        //     led_m(10, 1, 0, 0); // スタートバーセットNG状態→赤色点灯
        //     if (pushsw_get() == 1)
        //     {
        //         pattern = 11;
        //         log_mode = 1; // ログファイルオープン
        //         cnt1 = 0;
        //         cnt1 = 0;
        //         break;
        //     }
        //     led_m(50, 0, 1, 0); // スタートバーセットOK状態→緑色点灯
        // }
        break;

    case 1:

        // 2秒たったらバーに向けてすすむ
        if (cnt1 >= 2000)
        {
            motor(20, 20);
        }

        handle(0);

        // スタートバーに到着したら
        if (lineflag_cross == 1)
        {
            led_m(0, 0, 0, 0);
            cnt_msdwritetime = 0;
            pattern = 2;
            cnt1 = 0;
            encoder.clear();
            break;
        }
        led_m(10, 1, 1, 0);
        break;

    case 2:

        handle(0);
        motor(0, 0);

        if (lineflag_cross == 0 && cnt1 >= 1000)
        {
            pattern = 3;
        }

        break;

    case 3:
        handle(0);
        motor(80, 80);
        if (encoder.getTotalCount() >= 100)
        {
            pattern = 10;
            cnt1 = 0;
        }
        break;

    case 4:
        if (lineflag_cross == 0)
        {
            pattern = 10;
            log_mode = 2; // ログ記録中
        }
        break;

    case 10:
        // スタートバーが開いたかチェック
        if (lineflag_cross == 0)
        {
            // スタート！！
            led_m(0, 0, 0, 0);
            cnt_msdwritetime = 0;
            pattern = 11;
            encoder.setvalue(1000);
            cnt1 = 0;
            break;
        }
        led_m(10, 1, 1, 0);
        break;

    case 11:
        // 通常トレース
        led_m(50, 1, 1, 1);
        if (abs(allDeviation[60]) <= 15)
        {
            if (lineflag_cross)
            {
                pattern = 31;
                cnt1 = 0;
            }
            else if (lineflag_left)
            {
                pattern = 21;
                cnt1 = 0;
            }

            //    if(LeftX && RightX){
            //       pattern = 31;//クランク
            //   } else if(LeftX){
            //         pattern = 21;//レーンチェンジ
            //     }
        }

        myHandleVal = allDeviation[60] * 0.55;

        motor(60, 60);
        handle(myHandleVal);

        if (encoder.getCourseCount() >= 1120 * 54)
        {
            pattern = 101;
            cnt1 = 0;
        }

        break;

    case 21:
        led_m(50, 0, 0, 1);

        motor(60, 60);
        handle(0);

        if (cnt1 >= 150)
        {
            pattern = 22;
            cnt1 = 0;
        }
        break;

    case 22:
        led_m(50, 0, 1, 0);

        myHandleVal = allDeviation[60] * 0.55;
        handle(myHandleVal);
        motor(70, 70);
        if (!lineflag_center)
        {
            pattern = 23;
            cnt1 = 0;
        }
        break;

    case 23:
        motor(70, 70);
        handle(0);
        if (cnt1 >= 150)
        {
            pattern = 24;
            cnt1 = 0;
        }
        break;

    case 24:
        led_m(50, 1, 0, 0);

        motor(50, 75);
        handle(15);

        if (cnt1 >= 150)
        {
            pattern = 25;
            cnt1 = 0;
        }
        break;

    case 25:
        motor(60, 60);
        handle(-8);

        if (cnt1 >= 100)
        {
            pattern = 26;
            cnt1 = 0;
        }
        break;

    case 26:
        motor(60, 60);

        if (cnt1 <= 300)
        {
            handle(3);
        }
        else
        {
            handle(-3);
        }

        if (lineflag_center)
        {
            pattern = 11;
            cnt1 = 0;
        }
        break;

    case 31:
        led_m(50, 0, 0, 1);
        motor(60, 60);
        handle(0);
        if (cnt1 >= 150)
        {
            pattern = 32;
            cnt1 = 0;
        }
        break;

    case 32:
        motor(50, 50);
        myHandleVal = allDeviation[60] * 0.52;
        handle(myHandleVal);

        if (lineflag_right)
        {
            pattern = 41;
            cnt1 = 0;
        }
        else if (lineflag_left)
        {
            pattern = 51;
            cnt1 = 0;
        }
        break;

    case 41:
        led_m(50, 0, 0, 1);
        motor(80, 20);
        handle(-40);
        if (cnt1 >= 430)
        {
            pattern = 42;
            cnt1 = 0;
        }
        break;

    case 42:
        led_m(50, 0, 1, 0);

        if (lineflag_center)
        {
            pattern = 11;
        }
        if (cnt1 <= 300)
        {
            motor(80, 20);
            handle(-40);
        }
        else
        {
            motor(60, 60);
            handle(0);
        }
        break;

    case 51:
        led_m(50, 0, 1, 0);

        motor(20, 80);
        handle(40);

        if (cnt1 >= 430)
        {
            pattern = 52;
        }
        break;

    case 52:
        led_m(50, 1, 0, 0);
        if (cnt1 <= 300)
        {
            motor(20, 80);
            handle(40);
        }
        else
        {
            motor(60, 60);
            handle(0);
        }
        if (lineflag_center)
        {
            pattern = 11;
        }
        break;

        // case 21:
        // break;

        // case 31:
        // for(int y = 60; y < IMAGE_HEIGHT - 1;y++)
        // if(lineL_flag[y]==true){
        //     pattern = 32;
        //     handle(-20);
        // }
        // break;

        // case 32:
        // if(abs(CenterLineArray[40]-80) < 10){
        //     myHandleVal = CenterLineArray[40]*8.5;
        //     pattern = 11;
        // }
        // break;

        // case 21:
        //     // クロスライン検出時の処理
        //     led_m(100, 1, 0, 0);
        //     pattern = 22;
        //     cnt1 = 0;
        //     break;

        // case 22:
        //     // クロスラインを読み飛ばす

        //     motor(10, 10);
        //     myHandleVal = allDeviation[53] * 0.37;
        //     handle(myHandleVal);
        //     if (cnt1 >= lineSkipDistance && !lineflag_left && !lineflag_right)
        //     {
        //         pattern = 23;
        //         cnt1 = 0;
        //         encoder.clear();
        //     }
        //     break;

        // case 23:
        //     // クロスライン後のトレース、クランク検出

        //     if (lineflag_left)
        //     {

        //         // 左クランクと判断]
        //         cnt1 = 0;

        //         led_m(100, 0, 1, 0);
        //         handle(crankHandleVal + 4);
        //         motor(crankMotorPowerIN - 10, crankMotorPowerOUT);
        //         pattern = 31;
        //         encoder.update();

        //         break;
        //     }
        //     if (lineflag_right)
        //     {

        //         // 右クランクと判断
        //         cnt1 = 0;

        //         led_m(100, 0, 0, 1);
        //         handle(-crankHandleVal);
        //         motor(crankMotorPowerOUT, crankMotorPowerIN);
        //         pattern = 41;

        //         break;
        //     }
        //     motor(60, 60);
        //     myHandleVal = allDeviation[53] * 0.40;
        //     handle(myHandleVal);
        //     break;

        // case 31:
        //     // 左クランククリア処理　安定するまで少し待つ
        //     handle(crankHandleVal + 12);
        //     motor(crankMotorPowerIN - 12, crankMotorPowerOUT);

        //     led_m(100, 0, 1, 0);
        //     if (cnt1 >= crankDistance)
        //     {
        //         pattern = 11;
        //         cnt1 = 0;
        //         resetFlag = true;
        //     }
        //     break;
        // case 41:
        //     // 右クランククリア処理　安定するまで少し待つ
        //     handle(-crankHandleVal-12);
        //     motor(crankMotorPowerOUT, crankMotorPowerIN-1);
        //     led_m(100, 0, 1, 0);
        //     if (cnt1 >= crankDistance)
        //     {
        //         pattern = 11;
        //         cnt1 = 0;
        //         resetFlag = true;
        //     }
        //     break;

        // case 51:
        //     // ハーフライン検出時の処理
        //     led_m(100, 0, 1, 0);
        //     pattern = 52;

        //     cnt1 = 0;
        //     break;

        // case 52:
        //     // ハーフラインを読み飛ばす

        //     motor(10, 10);
        //     myHandleVal = allDeviation[50] * 0.39;
        //     handle(myHandleVal);
        //     if (cnt1 >= lineSkipDistance && !lineflag_left && !lineflag_right)
        //     {
        //         pattern = 53;
        //         cnt1 = 0;
        //     }
        //     if (laneHandleVal < 0 && (lineflag_left || lineflag_cross))
        //     {
        //         pattern = 22;
        //         break;
        //     }
        //     if (laneHandleVal > 0 && (lineflag_right || lineflag_cross))
        //     {
        //         pattern = 22;
        //         break;
        //     }
        //     if (lineflag_cross == 1)
        //     {
        //         pattern = 22;
        //         break;
        //     }
        //     break;

        // case 53:
        //     // ハーフライン後のトレース、レーンチェンジ

        //     if (!lineflag_center)
        //     {

        //         pattern = 54;
        //         led_m(100, 1, 1, 0);

        //         cnt1 = 0;
        //         break;
        //     }
        //     motor(30, 30);
        //     myHandleVal = allDeviation[48] * 0.38;
        //     handle(myHandleVal);
        //     break;

        // case 54:

        //     // レーンチェンジ終了のチェック
        //     handle(0);

        //     motor(laneStraightMotorPower, laneStraightMotorPower);
        //     if (cnt1 >= 10)
        //     {

        //         pattern = 55;
        //         led_m(100, 1, 0, 0);
        //         cnt1 = 0;
        //     }
        //     break;

        // case 55:

        //     handle(laneHandleVal);
        //     motor(laneMotorPowerLeft, laneCounterMotorPowerRight);
        //     if (cnt1 > laneDistance)
        //     {

        //         pattern = 11;
        //         led_m(100, 0, 0, 0);
        //         cnt1 = 0;
        //     }
        //     break;

    case 101:
        // 終了　ログ保存中など
        led_m(100, 0, 1, 1);
        if (cnt1 >= 2000)
        {
            handle(0);
        }
        else
        {
            handle(handleVal);
        }
        endflag = true;
        motor(0, 0);
        break;

    default:
        // どれでもない場合は待機状態に戻す
        pattern = 0;
        break;
    }
}

////****************************************************************
// functions ( on GR-PEACH board )
////****************************************************************
//------------------------------------------------------------------//
// user_button_get Function
//------------------------------------------------------------------//
unsigned char user_button_get(void)
{
    return (~user_botton) & 0x1; // Read ports with switches
}

//------------------------------------------------------------------//
// led_m Function
//------------------------------------------------------------------//
void led_m(int time, int r, int g, int b)
{
    mled_time = time;
    mled_r = r;
    mled_g = g;
    mled_b = b;
}

//------------------------------------------------------------------//
// led_m_process Function for only interrupt
//------------------------------------------------------------------//
void led_m_process(void)
{
    static int cnt_led_m = 0;

    if (cnt_led_m < mled_time * 5)
    {
        LED_R = mled_r;
        LED_G = mled_g;
        LED_B = mled_b;
    }
    else
    {
        LED_R = 0;
        LED_G = 0;
        LED_B = 0;
    }

    cnt_led_m++;
    if (cnt_led_m >= 500)
    {
        cnt_led_m = 0;
    }
}

///*****************************************************************
// functions ( on Motor drive board )
///*****************************************************************
//------------------------------------------------------------------//
// led_out Function
//------------------------------------------------------------------//
void led_out(int led)
{
    led = ~led;
    LED_3 = led & 0x1;
    LED_2 = (led >> 1) & 0x1;
}

//------------------------------------------------------------------//
// pushsw_get Function
//------------------------------------------------------------------//
unsigned char pushsw_get(void)
{
    return (!push_sw); // Read ports with switches
}

//------------------------------------------------------------------//
// motor speed control(PWM)
// Arguments: motor:-100 to 100
// Here, 0 is stop, 100 is forward, -100 is reverse
//------------------------------------------------------------------//
void motor(int accele_l, int accele_r)
{
    int sw_data;

    if (accele_l > MAX_MOTOR_POWER)
    {
        accele_l = MAX_MOTOR_POWER;
    }
    if (accele_l < -MAX_MOTOR_POWER)
    {
        accele_l = -MAX_MOTOR_POWER;
    }

    if (accele_r > MAX_MOTOR_POWER)
    {
        accele_r = MAX_MOTOR_POWER;
    }
    if (accele_r < -MAX_MOTOR_POWER)
    {
        accele_r = -MAX_MOTOR_POWER;
    }

    msd_l = accele_l;
    msd_r = accele_r;

    // Left Motor Control
    if (accele_l >= 0)
    {
        // forward
        Left_motor_signal = 0;
        MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_l / 100;
    }
    else
    {
        // reverse
        Left_motor_signal = 1;
        MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * (-accele_l) / 100;
    }

    // Right Motor Control
    if (accele_r >= 0)
    {
        // forward
        Right_motor_signal = 1;
        MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_r / 100;
    }
    else
    {
        // reverse
        Right_motor_signal = 0;
        MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE - 1) * (-accele_r) / 100;
    }
}

//------------------------------------------------------------------//
// handle Function
//------------------------------------------------------------------//
void handle(int angle)
{
    msd_handle = -angle;
    // When the servo move from left to right in reverse, replace "-" with "+"
    MTU2TGRD_0 = SERVO_CENTER - angle * HANDLE_STEP;
}

///*****************************************************************
// functions ( on Shield board )
///*****************************************************************
//------------------------------------------------------------------//
// Dipsw get Function
//------------------------------------------------------------------//
unsigned char dipsw_get(void)
{
    return (dipsw.read() & 0x0f);
}

//------------------------------------------------------------------//
// sensor Function
//------------------------------------------------------------------//
unsigned char sensor_inp(unsigned char mask)
{
    return (sensor_bin & mask); // 中
}

///*********************************************************************
// モジュール名 getCompileYear
// 処理概要     コンパイルした時の年を取得
// 引数　       __DATE__ のポインタ
// 戻り値       年
///*********************************************************************
int getCompileYear(const char *p)
{
    int i;

    i = atoi(p + 7);
    if (i < 1980 || i > 2107)
        i = 2019;

    return i;
}

///*********************************************************************
// モジュール名 getCompileMonth
// 処理概要     コンパイルした時の月を取得
// 引数　       __DATE__ のポインタ
// 戻り値       月
///*********************************************************************
int getCompileMonth(const char *p)
{
    int i, r;

    for (i = 0; i < 12; i++)
    {
        r = strncmp(monthStr + i * 3, p, 3);
        if (r == 0)
            return i + 1;
    }
    return 1;
}

///*********************************************************************
// モジュール名 getCompileDay
// 処理概要     コンパイルした時の日を取得
// 引数　       __DATE__ のポインタ
// 戻り値       日
///*********************************************************************
int getCompileDay(const char *p)
{
    int i;

    i = atoi(p + 4);
    if (i < 1 || i > 31)
        i = 1;

    return i;
}

///*********************************************************************
// モジュール名 getCompileHour
// 処理概要     コンパイルした時の時を取得
// 引数　       __TIME__ のポインタ
// 戻り値       時
///*********************************************************************
int getCompileHour(const char *p)
{
    int i;

    i = atoi(p);
    if (i < 0 || i > 23)
        i = 0;

    return i;
}

///*********************************************************************
// モジュール名 getCompilerMinute
// 処理概要     コンパイルした時の分を取得
// 引数　       __TIME__ のポインタ
// 戻り値       分
///*********************************************************************
int getCompilerMinute(const char *p)
{
    int i;

    i = atoi(p + 3);
    if (i < 0 || i > 59)
        i = 0;

    return i;
}

///*********************************************************************
// モジュール名 getCompilerSecond
// 処理概要     コンパイルした時の秒を取得
// 引数　       __TIME__ のポインタ
// 戻り値       秒
///*********************************************************************
int getCompilerSecond(const char *p)
{
    int i;

    i = atoi(p + 6);
    if (i < 0 || i > 59)
        i = 0;

    return i;
}

///*********************************************************************
// char型データの値をlong型変数に2進数で変換
// 引数　 unsigned char 変換元の8bitデータ
// 戻り値 unsigned long 変換先の変数(0～11111111) ※0か1しかありません
///*********************************************************************
unsigned long convertBCD_CharToLong(unsigned char hex)
{
    int i;
    unsigned long l = 0;

    for (i = 0; i < 8; i++)
    {
        l *= 10;
        if (hex & 0x80)
            l += 1;
        hex <<= 1;
    }

    return l;
}

///*********************************************************************
// クロスライン検出処理
// 戻り値 0:クロスラインなし 1:あり
///*********************************************************************
int check_crossline(void)
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK3_3);
    if (b == 0xe7)
    {
        ret = 1;
    }
    return ret;
}

///*********************************************************************
// 右ハーフライン検出処理
// 戻り値 0:なし 1:あり
///*********************************************************************
int check_rightline(void)
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if (b == 0x0f || b == 0x1f || b == 0x3f || b == 0x7f)
    {
        ret = 1;
    }
    return ret;
}

///*********************************************************************
// 左ハーフライン検出処理
// 戻り値 0:なし 1:あり
///*********************************************************************
int check_leftline(void)
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if (b == 0xf0 || b == 0xf8 || b == 0xfc || b == 0xfe)
    {
        ret = 1;
    }
    return ret;
}

///********************************************************************
// 指定した行数の８点を取得し、しきい値を自動で調整し2進数8ビットに変換する
// 引数：行数(0-119), しきい値, 差
// 戻り値：センサの８ビット
///********************************************************************
unsigned char shikiichi_henkan(int gyou, int s, int sa)
{
    int max, min, i, shiki;
    int d[8];
    int sa_7_6, sa_6_5, sa_5_4, sa_4_3, sa_3_2, sa_2_1, sa_1_0;
    unsigned char ret;

    d[7] = getImage(31, gyou);
    d[6] = getImage(43, gyou);
    d[5] = getImage(54, gyou);
    d[4] = getImage(71, gyou);
    d[3] = getImage(88, gyou);
    d[2] = getImage(105, gyou);
    d[1] = getImage(116, gyou);
    d[0] = getImage(128, gyou);

    min = max = d[0];
    for (i = 1; i < 8; i++)
    {
        if (max <= d[i])
        {
            max = d[i]; // 8個のうち、最大を見つける
        }
        if (min >= d[i])
        {
            min = d[i]; // 8個のうち、最小を見つける
        }
    }

    // 隣同士の差の絶対値
    sa_7_6 = abs(d[7] - d[6]);
    sa_6_5 = abs(d[6] - d[5]);
    sa_5_4 = abs(d[5] - d[4]);
    sa_4_3 = abs(d[4] - d[3]);
    sa_3_2 = abs(d[3] - d[2]);
    sa_2_1 = abs(d[2] - d[1]);
    sa_1_0 = abs(d[1] - d[0]);

    if (max >= s)
    {
        // 最大値がs以上なら、sをしきい値とする
        shiki = s;
    }
    else if (sa_7_6 >= sa || sa_6_5 >= sa || sa_5_4 >= sa ||
             sa_4_3 >= sa || sa_3_2 >= sa || sa_2_1 >= sa || sa_1_0 >= sa)
    {
        // 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 + 最小値　をしきい値とする
        shiki = (max - min) * 7 / 10 + min;
    }
    else
    {
        // 当てはまらなければ、しきい値を256とする、すなわちすべて0となる
        shiki = 256;
    }

    // d[7]～d[0]をbit7～bit0に割り当てる
    ret = 0;
    for (i = 7; i >= 0; i--)
    {
        ret <<= 1;
        ret |= (d[i] >= shiki ? 1 : 0);
    }

    return ret;
}

///********************************************************************
// イメージ領域のx,yの値(0-255)を取得
// 引数：x列数(0-159) , y行数(0-119)
// 戻り値：0～255
///********************************************************************
char getImage(int ix, int iy)
{
    return ImageData_B[ix + 160U * iy];
}

///********************************************************************
// ラインフラグを生成する関数
// 引数なし(自分でプログラムを変えてフラグを生成するために必要な行数を指定できるようにして　　初期６０行目)
// 戻り値：0～255
///********************************************************************

void createLineFlag(int rowNum)
{
    volatile int crosslineWidth = 60; // クロスラインの検出に中心から何列のデータを使うか指定(コースの幅より外側のデータを使わないため)
    volatile int centerWidth = 60;    // 中心線があるかの検出に中心から何列のデータを使うか指定(中心線の幅数)

    volatile int centerRowNum = 50; // ラインを検出する行数

    volatile int brightnessThreshold = 0; // 明るさの閾値明るさは255段階になっていて閾値より下の値が来ていた場合は切り捨てる
    volatile int maxBrightness = 0;
    volatile int bigImageData[IMAGE_WIDTH][IMAGE_HEIGHT]; // 特定の一行のみの画像の明るさデータが格納される配列

    volatile int imageData[IMAGE_WIDTH];  // 特定の一行のみの画像の明るさデータが格納される配列
    volatile int centerData[IMAGE_WIDTH]; // 特定の一行のみの画像の明るさデータが格納される配列

    volatile int leftCount = 0;   // 画像の左側に閾値以上の値がどれくらいあるかをカウントする
    volatile int rightCount = 0;  // 画像の右側に閾値以上の値がどれくらいあるかをカウントする
    volatile int centerCount = 0; // 画像のセンターライン付近に閾値以上の値がどれくらいあるかをカウントする

    // volatile int crossCountThreshold = 60;  // 画像のクロスラインのカウント数の閾値
    volatile int crossCountThreshold = 55;
    volatile int centerCountThreshold = 10; // 画像のセンターラインのカウント数の閾値

    // if (pattern != 11)
    // {
    //     rowNum = 60;
    // }

    // imageDataに画像データを格納する
    for (int y = rowNum; y < rowNum + 3; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            bigImageData[x][y] = getImage(x, y);
            imageData[x] = 0;
            if (maxBrightness < bigImageData[x][y])
            {
                maxBrightness = bigImageData[x][y];
            }
            // imageData[x] = getImage(x, rowNum);
        }
    }
    brightnessThreshold = maxBrightness * 0.76;
    for (int y = rowNum; y < rowNum + 3; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (bigImageData[x][y] > imageData[x])
            {
                imageData[x] = bigImageData[x][y];
            }
        }
    }
    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        centerData[x] = getImage(x, centerRowNum);
    }

    // レフトライン検出処理
    for (int x = IMAGE_CENTER - crosslineWidth / 2; x < IMAGE_CENTER; x++) // 画像の中心-クロスラインの幅の半分(レフトラインの幅)から画像の中心まで
    {
        if (imageData[x] > brightnessThreshold) // その画素が閾値よりも大きい値が来ていたらカウント++
        {
            leftCount++;
        }
    }

    // leftCountがcrossCountThresholdの半分(レフトラインの判定基準)より多く判定されたらフラグtrue
    if (leftCount > crossCountThreshold / 2)
    {
        lineflag_left = true;
    }
    else
    {
        lineflag_left = false;
    }

    // ここからは自分で読んで理解して！

    //  ライトライン検出処理
    for (int x = IMAGE_CENTER; x < IMAGE_CENTER + crosslineWidth / 2; x++)
    {
        if (imageData[x] > brightnessThreshold)
        {
            rightCount++;
        }
    }

    if (rightCount > crossCountThreshold / 2)
    {
        lineflag_right = true;
    }
    else
    {
        lineflag_right = false;
    }

    // クロスライン判定(ライトラインとレフトラインが検出されてたらクロスライン判定)
    if (lineflag_right == true && lineflag_left == true)
    {
        lineflag_cross = true;
    }
    else
    {
        lineflag_cross = false;
    }

    // 中心線があるかの検出
    for (int x = IMAGE_WIDTH / 2 - centerWidth / 2; x < IMAGE_WIDTH / 2 + centerWidth / 2; x++)
    {
        if (centerData[x] > brightnessThreshold)
        {
            centerCount++;
        }
    }

    if (centerCount > centerCountThreshold)
    {
        lineflag_center = true;
    }
    else
    {
        lineflag_center = false;
    }
}
// 偏差を作る関数
void createDeviation(void)
{
    volatile float brightnessThreshold = 0.77;   // 明るさの閾値倍率
    volatile int minasDifferenceThreshold = -13; // 左側差分検出の閾値
    volatile int plusDifferenceThreshold = 7;    // 右側差分検出の閾値

    volatile int differenceThresholdY = 10; // 一行下との検出された場所による外れ値検出の閾値

    volatile signed int allImageData[IMAGE_HEIGHT][IMAGE_WIDTH]; // 画像データが格納された配列
    volatile signed int maxBrightness = 0;                       // 明るさの最大値

    volatile signed int leftExceedingXPositions[IMAGE_HEIGHT][IMAGE_WIDTH];  // 左側の差分が検出された場所(添え字二個目は検出されたのの何個目かを表す)
    volatile signed int leftExceedingXPositionsCount[IMAGE_HEIGHT];          // 左側の差分が検出された個数
    volatile signed int rightExceedingXPositions[IMAGE_HEIGHT][IMAGE_WIDTH]; // 右側の差分が検出された場所(添え字二個目は検出されたのの何個目かを表す)
    volatile signed int rightExceedingXPositionsCount[IMAGE_HEIGHT];         // 右側の差分が検出された個数

    volatile signed int leftYDifference[IMAGE_HEIGHT][IMAGE_WIDTH];  // 左側の複数検出された差分の中心との距離
    volatile signed int rightYDifference[IMAGE_HEIGHT][IMAGE_WIDTH]; // 右側の複数検出された差分の中心との距離

    volatile signed int minLeftXDifference[IMAGE_HEIGHT];  // 左側の複数検出された差分の中でどれが一行下の中心と近いかを検出するための変数
    volatile signed int minRightXDifference[IMAGE_HEIGHT]; // 右側の複数検出された差分の中でどれが一行下の中心と近いかを検出するための変数

    volatile signed int rightCenterCount[IMAGE_HEIGHT]; // 右側の差分が検出された場所の何個目がセンターラインかを示す
    volatile signed int leftCenterCount[IMAGE_HEIGHT];  // 左側の差分が検出された場所の何個目がセンターラインかを示す

    volatile static signed int beforDeviationThreshold = 50;
    // 変数の初期化
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        leftExceedingXPositionsCount[y] = 0;
        rightExceedingXPositionsCount[y] = 0;
        minLeftXDifference[y] = 160;
        minRightXDifference[y] = 160;
    }

    // getimageから配列に格納
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            allImageData[y][x] = getImage(x, y);
            if (allImageData[y][x] > maxBrightness) // 同時に最大光度も記録
            {
                maxBrightness = allImageData[y][x];
            }
        }
    }

    // 最大光度×閾値以上の値だった場所は255にする(差分を大きくするために二値化に近い処理を行う)
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (allImageData[y][x] > maxBrightness * brightnessThreshold)
            {
                allImageData[y][x] = 255;
            }
        }
    }

    // 元データの特定ドットとその右隣のドットの光度の差分を検出する
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (x < IMAGE_RIGHT_EDGE)
            {
                difference[y][x] = allImageData[y][x] - allImageData[y][x + 1];
            }
            else
            {
                difference[y][x] = 0;
            }
        }
    }

    // 差分が検出されたところが線の右端か左端かを判断してそれが何列目かを記録する
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (difference[y][x] < minasDifferenceThreshold)
            {
                leftExceedingXPositions[y][leftExceedingXPositionsCount[y]] = x;
                leftExceedingXPositionsCount[y]++;
            }
            if (difference[y][x] > plusDifferenceThreshold)
            {
                rightExceedingXPositions[y][rightExceedingXPositionsCount[y]] = x;
                rightExceedingXPositionsCount[y]++;
            }
        }
        if (leftExceedingXPositionsCount[y] == 0) // 検出された場所がなかったら中心付近の値を入れる
        {
            leftExceedingXPositions[y][leftExceedingXPositionsCount[y]] = 70;
            leftExceedingXPositionsCount[y]++;
        }
        if (rightExceedingXPositionsCount[y] == 0)
        {
            rightExceedingXPositions[y][rightExceedingXPositionsCount[y]] = 90;
            rightExceedingXPositionsCount[y]++;
        }
    }

    // 画像の一番下の行は検出された場所の一つ目を中心とする
    leftCenterCount[IMAGE_BOTTOM_EDGE] = 0;
    rightCenterCount[IMAGE_BOTTOM_EDGE] = 0;

    // 画像の一番下から数行は差分が検出された場所を真ん中辺りにする
    for (int y = IMAGE_BOTTOM_EDGE; y > IMAGE_BOTTOM_EDGE - 1; y--)
    {
        leftExceedingXPositions[y][0] = IMAGE_CENTER - 10;
        rightExceedingXPositions[y][0] = IMAGE_CENTER + 10;
    }

    // 下から順番に一番下の行の中心線からどの検出された点が一番近いかを検出し、一番近かったものをセンターラインとする
    for (int y = IMAGE_BOTTOM_EDGE - 1; y >= 0; y--)
    {
        for (int count = 0; count < leftExceedingXPositionsCount[y]; count++)
        {
            // 検出されたポイントの個数のcount個目の一列下のセンターラインの検出場所との差＝検出されたポイントの個数のcount個目のX座標-一列下の既に検出された中心の場所
            leftYDifference[y][count] = abs(leftExceedingXPositions[y][count] - leftExceedingXPositions[y + 1][leftCenterCount[y + 1]]);
            if (leftYDifference[y][count] < minLeftXDifference[y])
            {
                // 差分の最小値を記録
                minLeftXDifference[y] = leftYDifference[y][count];
                // 検出されたポイントの何個目が中心かを記録
                leftCenterCount[y] = count;
            }
        }
        for (int count = 0; count < rightExceedingXPositionsCount[y]; count++)
        {
            rightYDifference[y][count] = abs(rightExceedingXPositions[y][count] - rightExceedingXPositions[y + 1][rightCenterCount[y + 1]]);
            if (rightYDifference[y][count] < minRightXDifference[y])
            {
                minRightXDifference[y] = rightYDifference[y][count];
                rightCenterCount[y] = count;
            }
        }
    }

    // 検出された中心線の場所が一列下の中心線との差分が閾値以上だったら外れ値として一列下の値を代入する
    for (int y = IMAGE_BOTTOM_EDGE - 10; y > 0; y--)
    {
        if (abs(leftExceedingXPositions[y][leftCenterCount[y]] - leftExceedingXPositions[y - 1][leftCenterCount[y - 1]]) > differenceThresholdY)
        {
            leftExceedingXPositions[y - 1][leftCenterCount[y - 1]] = leftExceedingXPositions[y][leftCenterCount[y]];
        }
        if (abs(rightExceedingXPositions[y][rightCenterCount[y]] - rightExceedingXPositions[y - 1][rightCenterCount[y - 1]]) > differenceThresholdY)
        {
            rightExceedingXPositions[y - 1][rightCenterCount[y - 1]] = rightExceedingXPositions[y][rightCenterCount[y]];
        }
    }

    // 最終的な画像の中心と中心線のずれ(偏差)をグローバル変数に代入
    allDeviation[IMAGE_HEIGHT - 1] = 0;
    for (int y = IMAGE_HEIGHT - 2; y > 1; y--)
    {
        leftDeviation[y] = IMAGE_CENTER - leftExceedingXPositions[y][leftCenterCount[y]];
        rightDeviation[y] = IMAGE_CENTER - rightExceedingXPositions[y][rightCenterCount[y]];
        allDeviation[y] = leftDeviation[y] + rightDeviation[y];
        if (abs(allDeviation[y + 1] - (IMAGE_CENTER - (leftExceedingXPositions[y][leftCenterCount[y]] + rightExceedingXPositions[y][rightCenterCount[y]]) / 2)) < beforDeviationThreshold)
        {
            allDeviation[y] = IMAGE_CENTER - (leftExceedingXPositions[y][leftCenterCount[y]] + rightExceedingXPositions[y][rightCenterCount[y]]) / 2;
        }
        else
        {
            allDeviation[y] = allDeviation[y + 1];
        }
        // if (resetFlag == true)
        // {
        //     resetFlag = false;
        //     allDeviation[y] = IMAGE_CENTER - (leftExceedingXPositions[y][leftCenterCount[y]] + rightExceedingXPositions[y][rightCenterCount[y]]) / 2;
        // }
    }
}

// void createMotorVal(void)
// {
//     volatile signed int accelerationBrakeGain = 12;
//     volatile signed int targetSpeed = 53;
//     volatile signed int neutralThrottle = 60;
//     volatile signed int brakeThrottle = 0;

//     volatile signed int limitAcceleration = 3;
//     if (targetSpeed < encoder.getCnt())
//     {
//         leftMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;
//         rightMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;
//     }
//     else
//     {
//         leftMotor = MAX_MOTOR_POWER;
//         rightMotor = MAX_MOTOR_POWER;
//     }
// if (encoder.getCnt() >= targetSpeed)
// {
// leftMotor = neutralThrottle;
// rightMotor = neutralThrottle;

// if (encoderAcceleration >= limitAcceleration)
// {
// leftMotor -= brakeThrottle;
// rightMotor -= brakeThrottle;
//}
//}
//}

void createMotorVal(void)
{
    volatile signed int accelerationBrakeGain = 3;
    volatile signed int targetSpeed = 60;
    volatile signed int neutralThrottle = 60;
    volatile signed int brakeThrottle = 0;

    volatile signed int limitAcceleration = 3;
    leftMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;
    rightMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;

    if (encoder.getCnt() >= targetSpeed)
    {
        leftMotor = neutralThrottle;
        rightMotor = neutralThrottle;

        if (encoderAcceleration >= limitAcceleration)
        {
            leftMotor = brakeThrottle;
            rightMotor = brakeThrottle;
        }
    }
}

void createBrakeMotorVal(int targetSpeed)
{
    volatile signed int largeSpeedThreshold = 10;
    volatile signed int mediumSpeedThreshold = 3;
    volatile signed int neutralThrottle = 80;
    volatile signed int acceleratedThrottle = 100;

    float encoderBrakeGain = 7;
    volatile signed int deviationTargetSpeed = encoder.getCnt() - targetSpeed;

    if (encoder.getCnt() >= targetSpeed + largeSpeedThreshold)
    {
        leftBrakeMotor = -100;
        rightBrakeMotor = -100;
    }
    else if (encoder.getCnt() >= targetSpeed + mediumSpeedThreshold)
    {
        leftBrakeMotor = neutralThrottle - deviationTargetSpeed * encoderBrakeGain;
        rightBrakeMotor = neutralThrottle - deviationTargetSpeed * encoderBrakeGain;
    }
    else if (encoder.getCnt() >= targetSpeed)
    {
        leftBrakeMotor = neutralThrottle;
        rightBrakeMotor = neutralThrottle;
    }
    else
    {
        leftBrakeMotor = acceleratedThrottle;
        rightBrakeMotor = acceleratedThrottle;
    }
}
void createHandleVal(void)
{
    volatile signed int highSpeed = 100;
    volatile signed int middleSpeed = 100;

    volatile signed int limitSpeed = 100;

    float straightCurveGain = 0.37;
    float middleCurveGain = 0.5;
    float bigCurveCurveGain = 0.5;

    float middleEncoderGain = 0.7;
    float bigEncoderGain = 0.7;

    float middleConstEncoderGain = 28;
    float bigConstEncoderGain = 25;

    volatile signed int straightDeviation = 0;
    volatile signed int middleCurveDeviation = 100;
    volatile signed int bigCurveDeviation = 100;

    volatile signed int farTraceLine = 40;
    volatile signed int midTraceLine = 40;
    volatile signed int nearTraceLine = 53;

    float midDifferenceGain = 0.2;
    float bigDifferenceGain = 0.7;

    volatile signed int traceLine;
    if (encoder.getCnt() >= highSpeed)
    {
        traceLine = farTraceLine;
    }
    else if (encoder.getCnt() >= middleSpeed)
    {
        traceLine = midTraceLine;
    }
    else
    {
        traceLine = nearTraceLine;
    }

    volatile signed int centerTraceLine = traceLine + 20;
    volatile signed int frontTraceLine = 110;

    deviationDifference = 0 /*abs(allDeviation[frontTraceLine] - allDeviation[traceLine])*/;
    // int allDeviationWa = 0;
    // for (int i = traceLine; i < frontTraceLine; i++)
    // {
    //     allDeviationWa += allDeviation[i];
    // }
    // deviationDifference = abs(allDeviationWa);

    if (abs(allDeviation[traceLine]) <= straightDeviation)
    {
        handleVal = 0;
    }
    else if (abs(allDeviation[traceLine]) <= middleCurveDeviation)
    {
        // if (encoder.getCnt() >= limitSpeed)
        // {
        handleVal = allDeviation[traceLine] * straightCurveGain;

        // }
        // else
        // {
        //     handleVal = allDeviation[traceLine] * abs(middleCurveGain + middleConstEncoderGain + deviationDifference * differenceGain);
        // }
    }
    else if (deviationDifference <= bigCurveDeviation)
    {
        // if (encoder.getCnt() >= limitSpeed)
        // {
        // handleVal = (allDeviation[traceLine]) * abs(encoder.getCnt() * middleEncoderGain + middleConstEncoderGain) / 100;
        // }
        // else
        // {
        handleVal = allDeviation[traceLine] * middleCurveGain;
        // }
    }
    else
    {
        // if (encoder.getCnt() >= limitSpeed)
        // {
        handleVal = (allDeviation[traceLine]) * abs(encoder.getCnt() * bigEncoderGain + deviationDifference) / 100;
        // }
        // else
        // {
        handleVal = allDeviation[traceLine] * bigCurveCurveGain;
        // }
    }
}

void easyCreateDeviation(int rowNum)
{

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        easyImageData[x] = getImage(x, rowNum);
    }

    for (int x = 0; x < IMAGE_WIDTH - 1; x++)
    {
        easyDifference[x] = easyImageData[x] - easyImageData[x + 1];
    }

    int count = 0;
    for (int x = 0; x < IMAGE_WIDTH - 1; x++)
    {
        if (easyDifference[x] < 0)
        {
            easyDifferencePoint[count] = x;
            count++;
        }
    }

    int min = IMAGE_WIDTH;
    for (int c = 0; c < count; c++)
    {
        if (abs(easyDifferencePoint[c] - IMAGE_CENTER) < min)
        {
            min = abs(easyDifferencePoint[c] - IMAGE_CENTER);
            easyDeviation = easyDifferencePoint[c] - IMAGE_CENTER;
        }
    }
}

// void kensyutu(void){
//   for(int y = IMAGE_HEIGHT-1; y >= 0;y--){
//       for(int x = 0;x < IMAGE_WIDTH;x++){
//        imgArray[y][x] = getImage(x,y);
//   if(imgArray[y][x] >= 200){
//       imgArray[y][x] = 255;
//      }
//     }
//   }

// for(int y = IMAGE_HEIGHT-1; y >= 0;y--){
//      for (int x = 0 ; x < IMAGE_WIDTH-1; x++) {
//       imgDeviation[y][x] = imgArray[y][x+1] - imgArray[y][x];//偏差xは0~158
//    }
//   }

//         for (int y = IMAGE_HEIGHT; y >= 0; y--) {
//          LeftLineArray[y] = 0;
//         }

//         LeftLineArray[119] = 80;

// for(int y = IMAGE_HEIGHT-2; y >= 0;y--){//レフトライン検出
//     for(int x = 0;x < IMAGE_WIDTH-1;x++){
//         if(imgDeviation[y][x] > LeftBorder){
//             if(abs(LeftLineArray[y+1] - x) < abs(LeftLineArray[y+1] - LeftLineArray[y])){
//             LeftLineArray[y] = x;
//             if(abs(LeftLineArray[y] - LeftLineArray[y + 1]) >= 2 && y <= 80){
//                 LeftLineArray[y] = LeftLineArray[y+1];
//             }
//             }
//         }
//     }

// }

//       for(int x = 0;x < IMAGE_WIDTH-1;x++){//最下層のレフトライン
//         if(imgDeviation[119][x] > LeftBorder){
//             if(abs(LeftLineArray[119 -1] - x) < abs(LeftLineArray[119-1] - LeftLineArray[119])){
//             LeftLineArray[119] = x;
//             }
//         }
//     }

// for (int y = IMAGE_HEIGHT-1; y >= 0; y--) {
//     RightLineArray[y] = 0;
// }

// for(int y = IMAGE_HEIGHT-1; y >= 0;y--){
//  for(int x = LeftLineArray[y];imgDeviation[y][x] > RightBorder;x++){
//    RightLineArray[y] = x;
//    if(abs(RightLineArray[y] - RightLineArray[y+1])>=2 && y <= 80){
//        RightLineArray[y] = RightLineArray[y+1];
//    }
//  }
// }

// for (int y = IMAGE_HEIGHT-1; y >= 0; y--) {
//     RightLineArray[y] -= 80;
// }

// for (int y = IMAGE_HEIGHT-1; y >= 0; y--) {
//     LeftLineArray[y] -= 80;
// }

//   for (int y = IMAGE_HEIGHT-1; y >= 0; y--) {
//     CenterLineArray[y] = (RightLineArray[y]+LeftLineArray[y]) / 2;
// }
// }

// void crossKensyutu(int LineRow){
//     int countL = 0;
//     int countR = 0;

//     for(int x = RightLineArray[LineRow];x <= RightLineArray[LineRow] + 3;x++){
//         if(imgArray[LineRow][x] >= 200){
//             countR++;
//         }
//     }

//     for(int x = LeftLineArray[LineRow];x >= LeftLineArray[LineRow] - 3;x--){
//         if(imgArray[LineRow][x] >= 200){
//             countR++;
//         }
//     }

//     if(countR >= 3){
//         RightX = true;
//     }else {
//     RightX  = false;
//     }
//     if(countL >= 3){
//         LeftX = true;
//     }else{
//         LeftX = false;
//     }

//     if(RightX && LeftX){
//         Cross = true;
//     }else {
//     Cross = false;
//     }
// }

//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
