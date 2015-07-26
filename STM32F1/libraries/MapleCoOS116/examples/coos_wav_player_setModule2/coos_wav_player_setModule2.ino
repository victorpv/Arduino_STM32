/*
* Multitasking Example with Maple CoOS. Victor Perez 2015
* Can use the stm32duino SPI and ILI9163C libraries. The ones included here just show how to use a semaphore.
* This example also shows how much CPU is used while runing the sketch. In this one it ranges between 28% when rotating
* both cubes, and 100% when it uses all the CPU left to run SQRT calculations.
* The sketch is designed to use all CPU available for short periods to test the task scheduling of high priority tasks
* while a low priority one hogs the CPU.
* The CPU measurement code is taken from this CoOS tree (old version) https://github.com/coocox/CoOS/tree/staticstic_task
* But rather than make it part of the OS, which was problematic, is implemented as additional tasks.
* In OsConfig.h, make sure you allow for enough tasks, i.e.: #define CFG_MAX_USER_TASKS      (10)
*/

#include <MapleCoOS116.h>
#include <SPI.h>
#include ".\TFT_ILI9163C.h"
//#include <TFT_ILI9163C.h>
#include <Adafruit_GFX.h>

#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <TMRpcm.h>           //  also need to include this library...

#include <SdFat.h>

#define SD_ChipSelectPin 8

#define __CS 11
#define __RST 9
#define __DC 10
#define BOARD_LED_PIN 33
#define BOARD_BUTTON_PIN 32
#define SDFAT

#define PRESSEDLEVEL HIGH
#define BUTTON_DEBOUNCE_DELAY 10
#define long_press_time 500
#define SHORT_PRESS 1
#define LONG_PRESS 2

SdFat SD;


TFT_ILI9163C tft = TFT_ILI9163C(__CS, __DC, __RST);

OS_MutexID xSPIFree;

OS_STK   vCube1LoopStk[TASK_STK_SIZE];
OS_STK   vLEDFlashStk[TASK_STK_SIZE];
OS_STK   vReadButtonStk[TASK_STK_SIZE];
OS_STK   vAudioBufferStk[TASK_STK_SIZE * 4];
OS_STK   NewIdleTaskStk[TASK_STK_SIZE];
OS_STK   CoStatTaskStk[TASK_STK_SIZE];
OS_STK   vAudioPlayerStk[TASK_STK_SIZE * 3];

/*
* For the wav player
*/
FatFile file;
SdFile dirFile;

const size_t MAX_FILE_COUNT = 50;
volatile uint16_t fileCount = 0;
volatile uint16_t idx = 0;
uint16_t volatile fileIndex[MAX_FILE_COUNT];
dir_t dir;
char name[13];
const char* FILE_EXT = "WAV";
volatile uint8 button_pressed = 0;


#define error(s) Serial.println(PSTR(s))


volatile U32  CoIdleCtrl = 0;		// Counter for Idle Task
volatile U32  CoIdleCtrMax = 0;		// Max Counter for Idle Task
volatile U8   CoStatRdy = 0;			// Enable Stat task.
volatile S8   CoCPUUsage = 0;			// Use %
volatile S8  CoCPUUsagePeak = 0;		// Peak %
volatile U32 CoIdleCtrRun = 0;		// val. reached by idle ctr at run time in 1 sec

const float sin_d[] = {
  0, 0.17, 0.34, 0.5, 0.64, 0.77, 0.87, 0.94, 0.98, 1, 0.98, 0.94,
  0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34, -0.5, -0.64,
  -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87, -0.77,
  -0.64, -0.5, -0.34, -0.17
};
const float cos_d[] = {
  1, 0.98, 0.94, 0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34,
  -0.5, -0.64, -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87,
  -0.77, -0.64, -0.5, -0.34, -0.17, 0, 0.17, 0.34, 0.5, 0.64, 0.77,
  0.87, 0.94, 0.98
};
const float d = 5;
float cube1_px[] = {
  -d,  d,  d, -d, -d,  d,  d, -d
};
float cube1_py[] = {
  -d, -d,  d,  d, -d, -d,  d,  d
};
float cube1_pz[] = {
  -d, -d, -d, -d,  d,  d,  d,  d
};

float cube1_p2x[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cube1_p2y[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

int cube1_r[] = {
  0, 0, 0
};
const float d2 = 10;

uint16 cube1_x, cube1_y, cube1_color;

static void vLEDFlashTask(void *pdata) {
  for (;;) {
    CoTickDelay(950);
    digitalWrite(BOARD_LED_PIN, HIGH);
    CoTickDelay(50);
    digitalWrite(BOARD_LED_PIN, LOW);
  }
}

static void vCube1LoopTask(void *pdata) {
  CoTimeDelay(0, 0, 4, 0); //Give time to calculate CPU Idle time.
  while (1) {
    CoEnterMutexSection (xSPIFree);
    SPI.setModule(2);
    cube(cube1_px, cube1_py, cube1_pz, cube1_p2x, cube1_p2y, cube1_r, &cube1_x, &cube1_y, &cube1_color);
    SPI.setModule(1);
    CoLeaveMutexSection(xSPIFree);
    CoTickDelay(15);
  }
}


void CoStatInit (void)
{
  CoTimeDelay(0, 0, 0, 100);                    	 // Mi sincronizzo con il timer e con la statistic task
  CoSchedLock();
  CoIdleCtrl    = 0L;                           /* Clear idle counter                                 */
  CoSchedUnlock();
  CoTimeDelay(0, 0, 1, 0);                      /* Determine MAX. idle counter value for 1 second     */
  CoSchedLock();
  CoIdleCtrMax = CoIdleCtrl;                    /* Store maximum idle counter count in 1 second       */
  CoStatRdy    = 1;
  CoSchedUnlock();
}

void NewIdleTask(void* pdata)
{
  /* Add your codes here */
  for (; ;)
  {
    CoSchedLock();
    CoIdleCtrl++;
    CoSchedUnlock();
    //       asm("    wfi");
  }
}


void CoStatTask(void* pdata)
{
  U32	    max, run;
  S8      usage;
  if (CoIdleCtrMax == 0) CoStatInit ();
  /*   while (CoStatRdy == 0)
  	{
  	    CoTickDelay(100);;			// Wait until statistic task is ready
  	}
  */
  max = CoIdleCtrMax / 100;

  for (; ;)
  {
    CoSchedLock();
    CoIdleCtrRun   = CoIdleCtrl;        /* Obtain the of the idle counter for the past second */
    CoIdleCtrl    = 0L;                 /* Reset the idle counter for the next second         */
    CoSchedUnlock();
    run = CoIdleCtrRun;
    if (max > 0L)
    {
      usage = (S8)(100L - (run / max));
      if (usage >= 0)
      {
        CoCPUUsage = usage;
        if (CoCPUUsage > CoCPUUsagePeak)
        {
          CoCPUUsagePeak = CoCPUUsage;
        }
      }
      else CoCPUUsage = 0;
    }
    else CoCPUUsage = 0;
    CoEnterMutexSection( xSPIFree );
    SPI.setModule(2);
    tft.setCursor(0, 0);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.print("USED CPU %=");
    tft.print(CoCPUUsage);
    tft.print(" ");
    SPI.setModule(1);
    CoLeaveMutexSection( xSPIFree );
    CoTimeDelay(0, 0, 1, 0);
  }
}

void play_idx () {
  //  strncpy (name, fileIndex[idx], 13);
  CoEnterMutexSection( xSPIFree );
  SD.vwd()->rewind();
  if (file.open(SD.vwd(), fileIndex[idx], O_READ) == false) error("Play_idx open failed");
  file.getSFN(name);
  file.close();
  /*
  Serial.println(name);
  */
  TMRpcm_play(name);
  CoLeaveMutexSection( xSPIFree );
}

void cube(float *px, float *py, float *pz, float *p2x, float *p2y, int *r, uint16 *x, uint16 *y, uint16 *color) {

  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], BLACK);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], BLACK);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], BLACK);
  }
  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], BLACK);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], BLACK);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], BLACK);

  r[0] = r[0] + 1;
  r[1] = r[1] + 1;
  if (r[0] == 36) r[0] = 0;
  if (r[1] == 36) r[1] = 0;
  if (r[2] == 36) r[2] = 0;
  for (int i = 0; i < 8; i++)
  {
    float px2 = px[i];
    float py2 = cos_d[r[0]] * py[i] - sin_d[r[0]] * pz[i];
    float pz2 = sin_d[r[0]] * py[i] + cos_d[r[0]] * pz[i];

    float px3 = cos_d[r[1]] * px2 + sin_d[r[1]] * pz2;
    float py3 = py2;
    float pz3 = -sin_d[r[1]] * px2 + cos_d[r[1]] * pz2;

    float ax = cos_d[r[2]] * px3 - sin_d[r[2]] * py3;
    float ay = sin_d[r[2]] * px3 + cos_d[r[2]] * py3;
    float az = pz3 - 190;

    p2x[i] = *x + ax * 500 / az;
    p2y[i] = *y + ay * 500 / az;
  }

  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], *color);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], *color);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], *color);
  }
  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], *color);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], *color);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], *color);
}

static void vAudioBuffer(void *pdata) {
  CoTimeDelay(0, 0, 4, 0); //Give time to calculate CPU Idle time.
  while (1) {
    while (TMRpcm_playing) {
      CoEnterMutexSection (xSPIFree);
      //      CoSchedLock();
      TMRpcm_buffer_load();
      //      CoSchedUnlock();
      CoLeaveMutexSection (xSPIFree);
      CoTickDelay(3);
    }
    CoTickDelay(50);
  }
}

static void vReadButton(void *pdata) {
  while (1) {
    if (digitalRead(BOARD_BUTTON_PIN) == PRESSEDLEVEL) {
      uint32 time_press = millis();
      CoTickDelay(BUTTON_DEBOUNCE_DELAY);
      while (digitalRead(BOARD_BUTTON_PIN) == PRESSEDLEVEL) CoTickDelay(50);
      if ((millis() - time_press) >= long_press_time) {
        button_pressed = LONG_PRESS;
      }
      else button_pressed = SHORT_PRESS;
    }
    CoTickDelay(50);
  }
}


static void vAudioPlayer(void *pdata) {
  CoTimeDelay(0, 0, 4, 0); //Give time to calculate CPU Idle time.
  speakerPin = 27; //TMR1 CH1
  speakerPin2 = 26; //TMR1 CH2
  //  TMRpcm_loop(1);
  CoEnterMutexSection (xSPIFree);
  if (!SD.begin(SD_ChipSelectPin, SPI_CLOCK_DIV2)) {  // see if the card is present and can be initialized:
    Serial.println("SD fail. Holding on loop.");
    CoLeaveMutexSection( xSPIFree );
    while (1) {
      CoTickDelay(500);
    }   // don't do anything more if not
  }
  // start at beginning of root directory
  if (!dirFile.open("/", O_READ)) {
    SD.errorHalt("open root failed");
  }
  // find files

    while (fileCount < MAX_FILE_COUNT && SD.vwd()->readDir(&dir) == sizeof(dir)) {
      if (!strncmp((char*)&dir.name[8], FILE_EXT, 3)) {
        fileIndex[fileCount] = (SD.vwd()->curPosition() / 32) - 1;
        Serial.println ((char*)&dir.name);
        fileCount++;
      }
    }
  
  CoLeaveMutexSection( xSPIFree );

  Serial.print("Found count: ");
  Serial.println(fileCount);
  Serial.println("Done searching.");
  if (fileCount == 0) {
    CoEnterMutexSection( xSPIFree );
    SPI.setModule(2);
    tft.setCursor(0, 10);
    tft.setTextColor(WHITE, RED);
    tft.println("No WAV files found.");
    SPI.setModule(1);
    CoLeaveMutexSection( xSPIFree );
    while (1) {
      CoTickDelay(500);
    };   // don't do anything more if not
    // destroy this task; // should change to something else like retrying the sdcard.
  }
  idx = 0;
  play_idx ();

  while (1) {
    CoEnterMutexSection( xSPIFree );
    SPI.setModule(2);
    //tft.setCursor(30, 12);
    tft.setTextColor(YELLOW, BLACK);
    SPI.setModule(1);
    for (uint8 i = 0; i < (fileCount < 4 ? fileCount : 4); i++) {
      SD.vwd()->rewind();
      if (!file.open(SD.vwd(), fileIndex[((idx + i) % fileCount)], O_READ)) {
        error("Listing names open failed");
        Serial.println ((idx + i) % fileCount);
        Serial.println (fileIndex[((idx + i) % fileCount)]);
      }
      file.getSFN(name);
      file.close();
      SPI.setModule(2);
      tft.setCursor(30, (8*i+20));
      tft.print(name);
      uint8 spaces = 12-strlen(name);
      while (spaces){
        tft.print(" ");
        spaces--;
      }
      tft.println();
      SPI.setModule(1);
    }
    CoLeaveMutexSection( xSPIFree );
    while (!button_pressed) CoTickDelay(100); // && TMRpcm_playing
    if (button_pressed == SHORT_PRESS) {
      idx = (idx + 1) % fileCount;
      button_pressed = 0;
    }
    else if (button_pressed == LONG_PRESS) {
      if (TMRpcm_playing) TMRpcm_stopPlayback();
      else play_idx();
      button_pressed = 0;
    }
    else {   // no button pressed, previous song just finished.
      idx = (idx + 1) % fileCount;
      play_idx ();
    }
  }
}

void setup() {
  // initialize the digital pin as an output:
  Serial.begin();
  delay (5000);
  Serial.println ("Running...");
  pinMode(BOARD_LED_PIN, OUTPUT);
  SPI.setModule(2);
  tft.begin();
  tft.fillScreen(BLACK);
  tft.setCursor(20, 125);
  tft.setTextColor(GREEN, BLACK);
  tft.println("(c)Victor Perez");
  tft.setCursor(2, 140);
  tft.println("stm32duino wav player");
  SPI.setModule(1);
  cube1_x = ((tft.width()) / 2);
  cube1_y = ((tft.height()) / 2);
  cube1_color = RED;
  // Clear PDDS and LPDS bits
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS;
  //  set sleepdeep in the system control register
  //  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;
  CoInitOS();
  xSPIFree = CoCreateMutex();
  CoCreateTask(vLEDFlashTask,
               (void *)0 ,
               2,
               &vLEDFlashStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  
  CoCreateTask(vCube1LoopTask,
                 (void *)0 ,
                 3,
                 &vCube1LoopStk[TASK_STK_SIZE - 1],
                 TASK_STK_SIZE
                );
  
  CoCreateTask(NewIdleTask,
               (void *)0 ,
               (CFG_LOWEST_PRIO - 1),
               &NewIdleTaskStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  CoCreateTask(CoStatTask,
               (void *)0 ,
               4,
               &CoStatTaskStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  CoCreateTask(vReadButton,
               (void *)0 ,
               2,
               &vReadButtonStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  CoCreateTask(vAudioBuffer,
               (void *)0 ,
               6,
               &vAudioBufferStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );

  CoCreateTask(vAudioPlayer,
               (void *)0 ,
               5,
               &vAudioPlayerStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );


  CoStartOS();
}

void loop() {
  // Do not write any code here, it would not execute.
}


