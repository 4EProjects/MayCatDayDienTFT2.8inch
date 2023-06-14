#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <AccelStepper.h> // https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#adfb19e3cd2a028a1fe78131787604fd1
#include <MultiStepper.h>
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include "Icon.h"

#include <esp32-hal-gpio.h> // ngắt ngoài
#include <esp_intr_alloc.h>

// GPIO PIN
#define buzzerPin 14        // Còi Buzz
#define stopButton 25       // Ngắt ngoài GPIO25
#define servoPin 16         // Servo
#define STEPPER1_DIR_PIN 26 // STEP1 WIRE
#define STEPPER1_STEP_PIN 27
#define STEPPER2_DIR_PIN 32 // STEP2 BLADE
#define STEPPER2_STEP_PIN 33

// Information
#define Model "Model: BM0623CTD    "
#define SeriaNumber "S/N  : SN01         "
#define FirmwareVision "FW   : V1.0 ESP32   "
#define Author "      Ba Manh 4E    "
#define SDT "     036.788.0317   "
#define Origin "   Made in VietNam  "

int currentAngle = 0; // Góc hiện tại của servo
int targetAngle = 0;  // Góc mong muốn của servo
int increment = 1;    // Giá trị tăng/giảm góc

unsigned long previousMillis = 0; // Thời gian trước đó
unsigned long interval = 15;      // Thời gian chờ giữa các bước di chuyển

uint8_t
    fillScreen1 = 0,
    fillScreenSetting = 0;

// ========= THÔNG SỐ CÀI ĐẶT HỆ THỐNG =========== //
// TFT cài đặt thong số HOME()
float
    WireLength = 0.0,   // Chiều dài của dây
    StripLength1 = 0.0, // Chiều dài tuốt 1
    StripLength2 = 0.0, // Chiều dài tuốt 2
    Diameter = 10.0;    // Đường kính trục con lăn
int16_t
    Quantity = 0; //     Số lượng
// SETTING()
int16_t
    OpenAngle = 0,     //     Góc tự nhiên (độ)
    CloseAngle = 0,    //     Góc cắt      (độ)
    BladeAngle = 0,    //     Góc tuốt     (độ)
    CutSpeed = 50,     //     Tốc độ cắt   (%)
    WireSpeed = 50,    //     Tốc độ dây   (%)
    cuttingSelect = 0, //     lựa chọn loại kéo cắt nào | Servo = 0 hoặc StepMotor = 1
    timeServo1deg = 3; //     Tốc độ khi cắt bằng servo 60deg = 0.1s -> 1deg = 0.1/60
uint8_t
    StartState = 0,
    SettingPage = 0;

// ========= THÔNG SỐ CÀI ĐẶT ĐỘNG CƠ BƯỚC ========== //
const float pi = 3.141592;                            // Số Pi
const float Full_Step = 1.0;                          //
const float Half_Step = 0.5;                          // 1/2
const float Quarter_Step = 0.25;                      // 1/4
const float Eighth_Step = 0.125;                      // 1/8
const float Sixteenth_Step = 0.0625;                  // 1/16
float Microstep = Eighth_Step;                        // Chế độ
float StepPerRound = 200.0 / Microstep;               // Số xung để quay được vòng 360 độ
float LengthPerStep = (pi * 1.8 * Microstep) / 360.0; // Chiều dài l trên một bước (R*pi*n)/180 = (D*pi*n)/360
const uint16_t SetMaxSpeed = 1000.0 / Microstep;      // Set tốc độ tối đa động cơ
const uint16_t DELAY_BETWEEN_CUTS = 200;              // Thời gian chờ cắt sợi tiếp theo

uint8_t
    motorBladeState = 1; // Mặc định khi mở máy
uint16_t soluong = 0;

typedef struct
{
  uint16_t EEPROM_OpenAngle;
  uint16_t EEPROM_CloseAngle;
  uint16_t EEPROM_BladeAngle;
  uint16_t EEPROM_CutSpeed;
  float EEPROM_Diameter;
  uint16_t EEPROM_WireSpeed;
  uint8_t EEPROM_cuttingSelect;
} ParameterSetting;
ParameterSetting EEPROM_Setting;

typedef enum
{
  step1 = 0, // kéo đoạn 1 vào
  step2,     // cắt tuốt đoạn 1
  step3,     // kéo đoạn 2 vào
  step4,     // cắt tuốt đoạn 2
  step5,     // kéo đoạn 3 vào
  step6,     // cắt đoạn 3
  step7,     // cắt đoạn 3
} HANDLE_STEP;
HANDLE_STEP handleStep;

// Keypad start position, key sizes and spacing, Vị trí gốc của cụm bàn phím ảo
#define KEY_X 60 // Centre of key
#define KEY_Y 85
#define KEY_W 62 // Width and height
#define KEY_H 30
#define KEY_SPACING_X 30 // X and Y gap
#define KEY_SPACING_Y 4
#define KEY_TEXTSIZE 1 // Font size multiplier

// Using two fonts since numbers are nice when bold
#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2

// Numeric display box size and location, Kích thước và vị trí của cụm bàn phím ảo
#define DISP_X 1
#define DISP_Y 10
#define DISP_W 310 // 239
#define DISP_H 50
#define DISP_TSIZE 3
#define DISP_TCOLOR TFT_CYAN

// Number length, buffer for storing it and character index
#define NUM_LEN 12
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;

// We have a status line for messages
#define STATUS_X 120 // Centred on this
#define STATUS_Y 65

// Create 15 keys for the keypad
char keyLabel[15][5] = {"New", "Del", "Send", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "#"};
uint16_t keyColor[15] = {TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE};

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button key[15];

// Tọa độ những ô thông số cài đặt
#define x_Open 89
#define y_Open 54
#define w_Open 59
#define h_Open 24
#define x_Close 89
#define y_Close 82
#define w_Close 59
#define h_Close 24
#define x_Blade 89
#define y_Blade 110
#define w_Blade 59
#define h_Blade 24
#define x_cutSpeed 89
#define y_cutSpeed 138
#define w_cutSpeed 59
#define h_cutSpeed 24

#define x_Diameter 242
#define y_Diameter 54
#define w_Diameter 59
#define h_Diameter 24
#define x_wireSpeed 242
#define y_wireSpeed 82
#define w_wireSpeed 59
#define h_wireSpeed 24

Servo myservo; // create servo object to control a servo
AccelStepper extruderStepper(1, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper linMotSteppers(1, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL true // = True nếu muốn cân bằng điểm cảm ứng

// Khai báo biến để lưu trữ trạng thái ngắt
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptTriggered = false; // Cờ báo hiệu ngắt đã xảy ra

uint16_t t1 = 0;

// Hàm xử lý ngắt ngoài
void IRAM_ATTR externalInterruptHandler()
{
  portENTER_CRITICAL(&mux);
  interruptTriggered = true;
  portEXIT_CRITICAL(&mux);
  // esp_restart();
  // myservo.write(OpenAngle);
  StartState = 0;
}

#include "callback.h"
void setup()
{
  // Use serial port
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  pinMode(stopButton, INPUT_PULLUP);

  // Initialise the TFT screen
  tft.init();
  tft.setRotation(3);
  tft.invertDisplay(1);
  touch_calibrate();
  // Khởi tạo Servo
  myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
                                       // using SG90 servo min/max of 500us and 2400us
                                       // for MG995 large servo, use 1000us and 2000us,
                                       // which are the defaults, so this line could be
                                       // "myservo.attach(servoPin);"
  myservo.write(OpenAngle);
  // Khởi tạo EEPROM
  EEPROM.begin(512);
  EEPROM_LoadSetting();
  // Khởi tạo động cơ bước
  linMotSteppers.setMaxSpeed(SetMaxSpeed);  // Đặt tốc độ tối đa, mặc định là rất chậm
  extruderStepper.setMaxSpeed(SetMaxSpeed); // Đặt tốc độ tối đa, mặc định là rất chậm
  // Gán hàm xử lý ngắt ngoài cho chân GPIO
  attachInterrupt(stopButton, externalInterruptHandler, FALLING); // FALLING sườn xuống, RISING sườn lên
}

void loop(void)
{
  TFT_Display();
  // Start
  if (StartState == 1)
  {
    // soluong = Quantity;
    if (Quantity - soluong != 0)
    {
      tft.setTextFont(1);
      tft.setTextSize(2);
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.setCursor(205, 175);
      tft.print(String(soluong + 1) + " ");

      if (StripLength1 == 0 && StripLength2 == 0) // Chỉ cắt, không tuốt
      {
        Serial.println("=== CAT ===");
        moveWire(WireLength);
        catDay();
        soluong++;
      }
      if (cuttingSelect == 0 && StripLength1 != 0 && StripLength2 != 0) // cắt và tuốt 2 đoạn
      {
        Serial.println("=== TUOT CAT TUOT ===");
        moveWire(StripLength1);
        tuotDay();
        moveWire(WireLength);
        tuotDay();
        moveWire(StripLength2);
        catDay();
        soluong++;
      }
      if (cuttingSelect == 0 && StripLength1 != 0 && StripLength2 == 0) // cắt và tuốt đoạn đầu
      {
        Serial.println("=== TUOT CAT ===");
        moveWire(WireLength);
        tuotDay();
        moveWire(StripLength1);
        catDay();
        soluong++;
      }
      if (cuttingSelect == 0 && StripLength2 != 0 && StripLength1 == 0) // cắt và tuốt đoạn sau
      {
        Serial.println("=== CAT TUOT ===");
        moveWire(StripLength2);
        tuotDay();
        moveWire(WireLength);
        catDay();
        soluong++;
      }
      // Handle();
    }
    if (soluong >= Quantity)
    {
      StartState = 0;
      soluong = 0;
    }
  }
  PrintMonitor();
}