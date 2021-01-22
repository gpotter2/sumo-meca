/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
 * File Name          : main.c
 * Author             : danceww
 * Version            : V0.0.1
 * Date               : 08/23/2010
 * Description        : Main program body
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/

//#include "includes.h"
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"

#include "AX12.h"
#include "AXS1.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46

#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB


#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11


#define PORT_LED_AUX			GPIOB
#define PORT_LED_MANAGE			GPIOB
#define PORT_LED_PROGRAM		GPIOB
#define PORT_LED_PLAY			GPIOB
#define PORT_LED_POWER			GPIOC
#define PORT_LED_TX				GPIOC
#define PORT_LED_RX				GPIOC

#define PIN_LED_AUX				GPIO_Pin_12
#define PIN_LED_MANAGE			GPIO_Pin_13
#define PIN_LED_PROGRAM			GPIO_Pin_14
#define PIN_LED_PLAY			GPIO_Pin_15
#define PIN_LED_POWER			GPIO_Pin_13
#define PIN_LED_TX				GPIO_Pin_14
#define PIN_LED_RX				GPIO_Pin_15


#define USART_DXL			    0
#define USART_PC			    2

#define word                    u16
#define byte                    u8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile byte                   gbpRxInterruptBuffer[256]; // dxl buffer
volatile byte                   gbRxBufferWritePointer,gbRxBufferReadPointer;
volatile vu32                   gwTimingDelay,gw1msCounter;
u32                             Baudrate_DXL = 	1000000;
u32                             Baudrate_PC = 57600;
vu16                            CCR1_Val = 100; 		// 1ms
vu32                            capture = 0;
word                            GoalPos[2] = {0, 1023};
//word                            GoalPos[2] = {0, 1023};  //For EX and MX series
word                            Position;
word                            wPresentPos;
byte                            INDEX = 0;
byte                            Voltage;
byte                            id = 1;
byte                            bMoving, CommStatus;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void Timer_configuration(void);
void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void __ISR_DELAY(void);
void USART1_Configuration(u32);
void USART_Configuration(u8, u32);
void DisableUSART1(void);
void ClearBuffer256(void);
byte CheckNewArrive(void);
void PrintCommStatus(int);
void PrintErrorCode(void);
void TxDByte_DXL(byte);
byte RxDByte_DXL(void);
void TxDString(char*);
void TxDWord16(word);
void TxDByte16(byte);
void TxDInt32(u32);
void TxDByte_PC(byte);
void Timer_Configuration(void);
void startTimeCount(void);
void stopTimeCount(void);
u32 getTime(void);
void mDelay(u32);
void mDelayClock(u32);
void mDelayMusic(u32);
void mDelayMusicLogic(u32, int* state);
void musicHandler(void);
void noteLightOn(int);
void noteLightOff(int);
void StartDiscount(s32);
byte CheckTimeOut(void);


////////////////////////////////////////////////////////////
///////////////////// value definitions ////////////////////
////////////////////////////////////////////////////////////

// change IDs here if the IDs of your motors/sensors are different
#define MOTOR_down_left 1
#define MOTOR_down_right 2
#define MOTOR_up_left 4
#define MOTOR_up_right 3
#define SENSOR 100


// state machine constants
#define INIT 0
#define SEEKING 1
#define CHASING 2
#define FLIP 3
#define STOP -1


#define thresholdInfrared 4
#define thresholdIRwhite 255
//#define thresholdIRwhite 9999
#define speed_ini 400
#define speed_max 512
#define speed_very_max 0x3ff


//////////////////////////////////////
/////////////// AX 12 ////////////////
//////////////////////////////////////

// infinite turn mode activation, see technical docu
// parameter: ID of motor
void infiniteTurn(unsigned char id) {
  dxl_write_word(id,  AX12_CTAB_ID_CWAngleLimitLo, 0 ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("problem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
    return;
  }
  dxl_write_word(id,  AX12_CTAB_ID_CCWAngleLimitLo, 0 ) ;
  result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("problem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
    return;
  }
  TxDString("\nCM5 infinite rotation mode set\n");
}


// infinite turn mode desactivation, see technical docu
// parameter: ID of motor
void normalTurn(unsigned char id) {
  dxl_write_word(id,  AX12_CTAB_ID_CWAngleLimitLo, 0 ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("problem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
    return;
  }
  dxl_write_word(id,  AX12_CTAB_ID_CCWAngleLimitLo, 1023 ) ;
  result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("problem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
    return;
  }
  TxDString("\nCM5 normal rotation mode set\n");
}


// set rotation speed of a single motor, only works in infinite turn mode!
// speed is an integer between -1023 and 1023
// parameter motor: ID of motor
// parameter speed: rotation speed, between -1024 and 1024, sign controls direction
// speed 1 = no ratation, speed 0 = maximal speed
void setSpeed(unsigned char id, int speed) {
  int order;
  if(speed >= 0)
    order = speed;
  else
    order = 1024 - speed;
  dxl_write_word(id, AX12_CTAB_ID_MovingSpeedLo, order ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("problem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

// move motor to a given angle, only works when nOt in infinite turn mode
// parameter motor: ID of motor
// parameter: angle is an integer between -1023 and 1023
// no angle should be between 300 and 360 degrees
void setAngle(unsigned char id, int angle, int speed) {
  setSpeed(id, speed);
  int angle_norm;

  if (angle >=0)
    angle_norm = angle;

  else
    angle_norm = 1024 + angle;

  dxl_write_word(id,  AX12_CTAB_ID_GoalPositionLo, angle_norm ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code==");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}


// turns on motor light
// parameter motor: ID of motor
void lightOn(unsigned char id) {
  dxl_write_byte(id, AX12_CTAB_ID_Led, 1 ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code==");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

// turns off motor light
// parameter motor: ID of motor
void lightOff(unsigned char id) {
  dxl_write_byte(id, AX12_CTAB_ID_Led, 0 ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code==");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

// returns the current motor's speed
// This functions does not return anything but stores the speed in its 2nd parameter which lust be a pointer to int
// parameter inId: ID of motor
// parameter outSpeed: pointer to which the speed will be stored
void getSpeed(unsigned char id, unsigned int* outSpeed) {
  *outSpeed = dxl_read_word(id, AX12_CTAB_ID_MovingSpeedLo) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}


// returns the current motor's angle,  infinite turn must be disabled to use this function
// This functions does not return anything but stores the speed in its 2nd parameter which lust be a pointer to int
// parameter inId: ID of motor
// parameter outAngle: pointer to which the angle will be stored
void getAngle(unsigned char id, unsigned int* outAngle) {
  *outAngle = dxl_read_word(id, AX12_CTAB_ID_PresentPosLo) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}


/////////////////////////////////////////////////////
////////////////////////////// AX S1 ////////////////
/////////////////////////////////////////////////////

// returns the obstacle detection flag (using infrared sensors), see technical documentation
// parameter sensor: Id of AX-S1
// parameter boolLight: pointer to store data read from AX-S1
void checkObstacle(unsigned char sensor, unsigned char* infoObst) {
  *infoObst = dxl_read_byte(sensor, AXS1_CTAB_ID_ObstacleDetectionFlag) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }

}


// returns the light detection flag (using visual light sensors), see technical documentation
// parameter sensor: Id of AX-S1
// parameter boolLight: pointer to store data read from AX-S1
void checkLuminosity(unsigned char sensor, unsigned char* info)  {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_LuminosityDetectionFlag) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }

}
// returns the left infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void leftInfraRed(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_LeftIRSensorData) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

// returns the center infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void centerInfraRed(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_CenterIRSensorData) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

// returns the right infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void rightInfraRed(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_RightIRSensorData) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}


// returns the left leight sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter leftLum: pointer to store data read from AX-S1
void leftLuminosity(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor,AXS1_CTAB_ID_LeftLuminosity ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

// returns the central light sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter centerLum: pointer to store data read from AX-S1
void centerLuminosity(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor,AXS1_CTAB_ID_CenterLuminosity ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

// returns the right light sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter rightLum: pointer to store data read from AX-S1
void rightLuminosity(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor,AXS1_CTAB_ID_RightLuminosity ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}


// returns the amount of sound detected
// untested, see documentation of AX-S1!!
void dataSound(unsigned char sensor, unsigned int* info) {
  *info = dxl_read_word(sensor,AXS1_CTAB_ID_SoundData) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}


//helper function
void noteBuzz(unsigned char sensor, int note) {
  dxl_write_byte(sensor, AXS1_CTAB_ID_BuzzerIndex, note) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

//helper function
void timeBuzz(unsigned char sensor, int time) {
  int result=-1 ;
  while (result != COMM_RXSUCCESS)
  {
    dxl_write_byte(sensor, AXS1_CTAB_ID_BuzzerTime, time) ;
    result =  dxl_get_result();
  }
  if( result != COMM_RXSUCCESS	)
  {
    TxDString("\nproblem, code=");
    TxDWord16(result);
    TxDString("!!!\n");
  }
}

void startBuzz(unsigned char sensor, int note){
  // infinite duration buzz
  timeBuzz(sensor,254);
  noteBuzz(sensor, note);
}

void stopBuzz(unsigned char sensor){
  // shut off buzz
  timeBuzz(sensor,0) ;
}

// play a note of given duration on the AX-S1.
// time is in milliseconds, so 500 means half a second
void buzzWithDelay(unsigned char sensor, int note, int time) {
  startBuzz(sensor, note);
  mDelay(time);
  stopBuzz(sensor);
}


/////////////////////////////////////////////////////////////////////////
//////////////////////////   M A I N   L O O P   ////////////////////////
/////////////////////////////////////////////////////////////////////////

void spin(int speedr, int speedl){
  int _speedr = speedr;
  int _speedl = speedl;
  setSpeed(MOTOR_up_left, _speedl);
  setSpeed(MOTOR_up_right, _speedr);
  setSpeed(MOTOR_down_left, _speedl);
  setSpeed(MOTOR_down_right, _speedr);
}

void forward(int speed){
  int _speed = speed;
  setSpeed(MOTOR_up_left, _speed);
  setSpeed(MOTOR_down_left, _speed);
  setSpeed(MOTOR_up_right, -_speed);
  setSpeed(MOTOR_down_right, -_speed);
}

void initSequence(int* state){
  spin(speed_ini, speed_ini);
  mDelayMusic(900);
  forward(speed_max);
  mDelayMusicLogic(2640, state);
  if(*state != INIT)
    return;
  spin(-speed_ini, -speed_ini);
  mDelayMusicLogic(1800, state);
  if(*state != INIT)
    return;
  forward(speed_max);
  mDelayMusicLogic(2500, state);

  *state = SEEKING;
}

int whiteBorderCount = 0;
char detectWhiteBorder(int* state){
  unsigned char field;
  leftInfraRed(SENSOR, &field);
  if(field >= thresholdIRwhite){
    whiteBorderCount++;
    if(whiteBorderCount > 5){
      *state = FLIP;
      return 1;
    }
  } else {
    whiteBorderCount = 0;
  }
  return 0;
}

void goToCenterSequence(int* state){
  TxDString("\nGO TO CENTER\n") ;
  forward(speed_ini);
  // advance for 3s, maybe adapt...
  mDelayMusicLogic(3000, state);
  *state = SEEKING;
}

#define AVANCER 0
#define FLIP_DROITE 1
#define FLIP_GAUCHE 2
void seekSequence(int* state){
  int sous_etat = FLIP_GAUCHE;
  while (*state == SEEKING){
    musicHandler();
    if(sous_etat == AVANCER){
      TxDString("AVANCER\n");
      forward(speed_very_max);
      mDelayMusicLogic(1900, state);
      sous_etat = FLIP_GAUCHE;
    } else if (sous_etat == FLIP_DROITE){
      TxDString("FLIP_DROITE\n");
      spin(speed_max, speed_max);
      mDelayMusicLogic(2400, state);
      sous_etat = AVANCER;
    } else if (sous_etat == FLIP_GAUCHE){
      TxDString("FLIP_GAUCHE\n");
      spin(-speed_max, -speed_max);
      mDelayMusicLogic(1400, state);
      sous_etat = FLIP_DROITE;
    }
    if(*state != SEEKING)
      break;
  }
}

void chaseSequence(int* state){
  unsigned char field;
  // the robot will focus the opponent and try to push him away,
  // as hard as possible
  forward(speed_very_max);
  while (*state == CHASING) {
    musicHandler();
    centerInfraRed(SENSOR, &field);
    //
    TxDString("\nCHASING SENSOR VALUE: ") ;
    TxDByte16(field);
    TxDString("\n") ;
    // if, for whatever reason, the robot does not detect any obstacle anymore
    // it returns to its seeking opponent phase
    if (field<thresholdInfrared/2)
      *state = SEEKING;
    else if (detectWhiteBorder(state)){
      TxDString("EJECT STATE\n");
      forward(speed_max);
      mDelayMusic(2000);
      forward(-speed_max);
      mDelayMusic(4000);
      spin(speed_max, speed_max);
      mDelayMusic(3000);
      forward(speed_ini);
      *state = SEEKING;
      mDelayMusicLogic(1000, state);
    }
  }
}

void flipSequence(int* state){
  while (*state == FLIP){
    *state = SEEKING; 
    forward(-speed_ini);
    mDelayMusic(1000);
    spin(-speed_ini, -speed_ini);
    mDelayMusicLogic(4000, state);
    if (*state != FLIP)
      break;
    forward(speed_ini);
    mDelayMusicLogic(1000, state);
  }
}

void getPinsByNote(int note, GPIO_TypeDef** PORT, int* PIN){
  switch(note % 6){
    case 0: *PORT = PORT_LED_MANAGE;  *PIN = PIN_LED_MANAGE ; break;
    case 1: *PORT = PORT_LED_PROGRAM; *PIN = PIN_LED_PROGRAM ; break;
    case 2: *PORT = PORT_LED_PLAY;    *PIN = PIN_LED_PLAY ; break;
    case 3: *PORT = PORT_LED_TX;      *PIN = PIN_LED_TX ; break;
    case 4: *PORT = PORT_LED_RX;      *PIN = PIN_LED_RX ; break;
    case 5: *PORT = PORT_LED_AUX;     *PIN = PIN_LED_AUX ; break;
  }
}

void noteLightOn(int note){
  GPIO_TypeDef* PORT;
  int PIN;
  getPinsByNote(note, &PORT, &PIN);
  GPIO_ResetBits(PORT, PIN);
}

void noteLightOff(int note){
  GPIO_TypeDef* PORT;
  int PIN;
  getPinsByNote(note, &PORT, &PIN);
  GPIO_SetBits(PORT, PIN);
}

// Music
word music_current_note;
byte playing;
u32 music_next_ts = 3000;
#define NB_NOTES 570
int music_notes[NB_NOTES] = {17,17,29,24,23,22,20,17,20,22,15,15,29,24,23,22,20,17,20,22,14,14,29,24,23,22,20,17,20,22,13,13,29,24,23,22,20,17,20,22,17,17,29,24,23,22,20,17,20,22,15,15,29,24,23,22,20,17,20,22,14,14,29,24,23,22,20,17,20,22,13,13,29,24,23,22,20,17,20,22,20,20,20,20,20,20,17,17,20,20,20,20,22,23,22,20,17,20,22,20,20,20,20,22,23,24,27,24,29,29,29,24,29,27,34,24,24,24,24,24,24,22,22,24,24,24,24,24,22,24,29,24,22,29,24,22,20,27,22,20,19,13,15,17,20,27,1,1,1,1,1,20,17,20,22,23,22,20,17,23,22,20,22,23,24,27,24,23,22,20,17,19,20,22,24,27,28,23,23,22,20,22,20,22,24,32,31,29,31,32,34,31,36,36,35,34,33,32,31,30,29,28,30,1,1,1,1,1,20,17,20,22,23,22,20,17,23,22,20,22,23,24,27,24,23,22,20,17,19,20,22,24,27,28,23,23,22,20,22,20,22,24,32,31,29,31,32,34,31,36,36,35,34,33,32,31,30,29,28,30,1,8,7,5,8,1,8,7,5,5,5,5,17,12,11,10,8,5,8,10,3,3,17,12,11,10,8,5,8,10,2,2,17,12,11,10,8,5,8,10,2,2,17,12,11,10,8,5,8,10,5,5,17,12,11,10,8,5,8,10,3,3,17,12,11,10,8,5,8,10,5,5,32,31,27,31,29,24,27,29,5,5,32,31,27,31,29,24,27,29,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,5,5,5,5,4,4,4,4,4,4,3,3,3,3,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,5,5,5,5,4,4,4,4,4,4,3,3,3,3,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,17,17,29,24,23,22,20,17,20,22,17,17,29,24,23,22,20,17,20,22,13,13,29,24,23,22,20,17,20,22,15,15,29,24,23,22,20,17,20,22,17,17,29,24,23,22,20,17,20,22,17,17,29,24,23,22,20,17,20,22,13,13,29,24,23,22,20,17,20,22,15,15,29,24,23,22,20,17,20,22};
int music_notes_length[NB_NOTES] = {65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,195,65,65,65,195,195,195,260,195,65,65,65,195,195,65,65,65,65,130,195,65,65,65,195,195,195,195,260,195,195,65,65,65,455,520,195,65,65,65,195,195,195,260,195,65,65,65,195,195,195,195,65,195,195,195,195,195,195,195,195,195,195,195,65,195,1040,195,195,65,195,65,65,65,65,65,65,65,65,65,65,65,195,1040,195,65,195,65,65,65,65,65,65,195,195,195,195,195,195,65,65,65,1170,195,195,195,195,455,455,455,455,455,455,975,65,65,65,65,65,65,65,65,975,975,195,195,65,195,65,65,65,65,65,65,65,65,65,65,65,195,1040,195,65,195,65,65,65,65,65,65,195,195,195,195,195,195,65,65,65,1170,195,195,195,195,455,455,455,455,455,455,975,65,65,65,65,65,65,65,65,975,975,1495,455,975,975,4095,1495,455,975,975,4095,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,195,195,65,195,195,195,65,65,65,195,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65,65,65,130,130,130,130,195,65,65,65};
int music_next_notes_delay[NB_NOTES] = {65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,65,65,65,65,65,390,65,65,65,65,65,65,65,65,65,65,260,65,65,65,65,65,65,65,65,130,65,65,65,65,65,65,230,65,65,65,65,65,65,65,390,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,130,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,130,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,65,65,65,130,260,130,130,65,65,65,2000};

// perform music tasks
void musicHandler(){
  //return;
  if(music_current_note == NB_NOTES){
    music_current_note = 0;
  }
  u32 t = getTime();
  if(t >= music_next_ts){
    TxDString("MUSIC NOTE: ");
    TxDByte16(music_current_note);
    TxDString(":");
    TxDByte16(music_notes[music_current_note]);
    TxDString("\n");
    if(playing){
      stopBuzz(SENSOR);
      noteLightOff(music_notes[music_current_note]);
      playing = 0;
      music_next_ts = music_next_ts + music_next_notes_delay[music_current_note];
      music_current_note++;
    } else {
      startBuzz(SENSOR, music_notes[music_current_note]);
      noteLightOn(music_notes[music_current_note]);
      playing = 1;
      music_next_ts = music_next_ts + music_notes_length[music_current_note];
    }
  }
}

int main(void)
{
  // --------------DO NOT TOUCH!!------------------------ //
  // NEVER!!! EVER!!!

  /* System Clocks Configuration */
  RCC_Configuration();

  /* NVIC configuration */
  NVIC_Configuration();

  /* GPIO configuration */
  GPIO_Configuration();

  SysTick_Configuration();

  Timer_Configuration();

  dxl_initialize( 0, 1 );
  USART_Configuration(USART_PC, Baudrate_PC);
  // -----------------UNTIL HERE------------------------- //

  // here, touch all u like :-) 

  // state will define the robot's attitude towards its environment
  // it implements a state machine
  // thats why we call it state :-)
  int state;

  infiniteTurn(MOTOR_up_left);
  infiniteTurn(MOTOR_up_right);
  infiniteTurn(MOTOR_down_left);
  infiniteTurn(MOTOR_down_right);

  //  printf("Resetting motors\n") ;
  setSpeed(MOTOR_up_left, 0);
  setSpeed(MOTOR_up_right, 0);
  setSpeed(MOTOR_down_left, 0);
  setSpeed(MOTOR_down_right, 0);
  
  lightOn(MOTOR_up_right);
  lightOn(MOTOR_up_left);
  lightOn(MOTOR_down_left);
  lightOn(MOTOR_down_right);

  // Initial bait notes
  buzzWithDelay(SENSOR, 31, 303);
  mDelay(43);
  buzzWithDelay(SENSOR, 19, 129);
  mDelay(43);
  buzzWithDelay(SENSOR, 26, 455);
  mDelay(65);
  buzzWithDelay(SENSOR, 24, 650);
  mDelay(43);
  buzzWithDelay(SENSOR, 31, 303);
  mDelay(43);
  buzzWithDelay(SENSOR, 26, 1040);

  //test buzzer
  //startTimeCount();
  //while (1){
  //  u32 t = getTime();
  //  TxDByte16(t / 1000);
  //  TxDString("\n");
  //  if(detectWhiteBorder(&state)){
  //    buzzWithDelay(SENSOR, 30, 500);
  //  }
  //}

  //test capteurs
  //unsigned char field;
  //while(1){
  //  centerInfraRed(SENSOR, &field);
  //  //leftInfraRed(SENSOR, &field);
  //  TxDString("InfraRed, Luminosity: ");
  //  TxDByte16(field);
  //  TxDString("; ");
  //  centerLuminosity(SENSOR, &field);
  //  TxDByte16(field);
  //  TxDString("\n");
  //}

  startTimeCount();

  //test musique
  //while(1){
  //  musicHandler();
  //}

  //test spin
  //spin(speed_max, speed_max);
  //while(1){}

  state = INIT;
  // state should equal INIT only at the beginning of each match
  while(state != STOP)
  {
    if(state == INIT){
      TxDString("INIT\n");
      initSequence(&state);
    }
    if(state == SEEKING){
      TxDString("SEEKING\n");
      seekSequence(&state);
    }
    if(state == CHASING){
      TxDString("CHASING\n");
      chaseSequence(&state);
    }
    if(state == FLIP){
      TxDString("FLIP\n");
      flipSequence(&state);
    }
  }

  while (1) {} ;

  return 0;
}

// --------------DO NOT TOUCH!!------------------------ //
// NEVER!!! EVER!!!

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : Configures the different system clocks.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

  /* Enable peripheral clocks --------------------------------------------------*/

  /* Enable USART1 and GPIOB clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);

  /* Enable USART3 clocks */
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2, ENABLE);

  PWR_BackupAccessCmd(ENABLE);
}

/*******************************************************************************
 * Function Name  : NVIC_Configuration
 * Description    : Configures Vector Table base location.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  // Set the Vector Table base location at 0x20000000
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  // VECT_TAB_FLASH
  // Set the Vector Table base location at 0x08003000
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
#endif

  // Configure the NVIC Preemption Priority Bits
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable the TIM2 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * Function Name  : GPIO_Configuration
 * Description    : Configures the different GPIO ports.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  // PORTB CONFIG
  GPIO_InitStructure.GPIO_Pin = 	PIN_ENABLE_TXD | PIN_ENABLE_RXD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
  GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);

  GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
  GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}

void USART1_Configuration(u32 baudrate)
{
  USART_Configuration(USART_DXL, baudrate);
}

void USART_Configuration(u8 PORT, u32 baudrate)
{

  USART_InitTypeDef USART_InitStructure;

  USART_StructInit(&USART_InitStructure);


  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  if( PORT == USART_DXL )
  {
    USART_DeInit(USART1);
    mDelay(10);
    /* Configure the USART1 */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TC, ENABLE);

    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);
  }

  else if( PORT == USART_PC )
  {
    USART_DeInit(USART3);
    mDelay(10);
    /* Configure the USART3 */
    USART_Init(USART3, &USART_InitStructure);

    /* Enable USART3 Receive and Transmit interrupts */
    //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART3, USART_IT_TC, ENABLE);

    /* Enable the USART3 */
    USART_Cmd(USART3, ENABLE);
  }
}

void DisableUSART1(void)
{
  USART_Cmd(USART1, DISABLE);
}

void ClearBuffer256(void)
{
  gbRxBufferReadPointer = gbRxBufferWritePointer = 0;
}

byte CheckNewArrive(void)
{
  if(gbRxBufferReadPointer != gbRxBufferWritePointer)
    return 1;
  else
    return 0;
}

void TxDByte_DXL(byte bTxdData)
{
  GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
  GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

  USART_SendData(USART1,bTxdData);
  while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

  GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
  GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}

byte RxDByte_DXL(void)
{
  byte bTemp;

  while(1)
  {
    if(gbRxBufferReadPointer != gbRxBufferWritePointer) break;
  }

  bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
  gbRxBufferReadPointer++;

  return bTemp;
}


// Print communication result
void PrintCommStatus(int CommStatus)
{
  switch(CommStatus)
  {
    case COMM_TXFAIL:
      TxDString("COMM_TXFAIL: Failed transmit instruction packet!\n");
      break;

    case COMM_TXERROR:
      TxDString("COMM_TXERROR: Incorrect instruction packet!\n");
      break;

    case COMM_RXFAIL:
      TxDString("COMM_RXFAIL: Failed get status packet from device!\n");
      break;

    case COMM_RXWAITING:
      TxDString("COMM_RXWAITING: Now recieving status packet!\n");
      break;

    case COMM_RXTIMEOUT:
      TxDString("COMM_RXTIMEOUT: There is no status packet!\n");
      break;

    case COMM_RXCORRUPT:
      TxDString("COMM_RXCORRUPT: Incorrect status packet!\n");
      break;

    default:
      TxDString("This is unknown error code!\n");
      break;
  }
}

// Print error bit of status packet
void PrintErrorCode()
{
  if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    TxDString("Input voltage error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    TxDString("Angle limit error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    TxDString("Overheat error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    TxDString("Out of range error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    TxDString("Checksum error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    TxDString("Overload error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    TxDString("Instruction code error!\n");
}

void TxDString(char *bData)
{
  while (*bData)
    TxDByte_PC(*bData++);
}

void TxDWord16(word wSentData)
{
  TxDByte16((wSentData >> 8) & 0xff);
  TxDByte16(wSentData & 0xff);
}

void TxDInt32(u32 bSentData){
  TxDWord16((bSentData >> 16) & 0xffff);
  TxDWord16(bSentData & 0xffff);
}

void TxDByte16(byte bSentData)
{
  byte bTmp;

  bTmp = ((byte) (bSentData >> 4) & 0x0f) + (byte) '0';
  if (bTmp > '9')
    bTmp += 7;
  TxDByte_PC(bTmp);
  bTmp = (byte) (bSentData & 0x0f) + (byte) '0';
  if (bTmp > '9')
    bTmp += 7;
  TxDByte_PC(bTmp);
}

void TxDByte_PC(byte bTxdData)
{
  USART_SendData(USART3,bTxdData);
  while( USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET );
}

void Timer_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);

  TIM_DeInit(TIM2);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val ;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

void TimerInterrupt_1ms(void) //OLLO CONTROL
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // 1ms//
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    capture = TIM_GetCapture1(TIM2);
    TIM_SetCompare1(TIM2, capture + CCR1_Val);

    if(gw1msCounter > 0)
      gw1msCounter--;
  }
}

/*__interrupt*/
void RxD0Interrupt(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(USART1);
}

void SysTick_Configuration(void)
{
  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
  SysTick_SetReload(9000);

  /* Enable SysTick interrupt */
  SysTick_ITConfig(ENABLE);
}

void __ISR_DELAY(void)
{
  gwTimingDelay++;
}

void startTimeCount(){
  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);
  gwTimingDelay = 0;
}

u32 getTime(){
  return gwTimingDelay;
}

void stopTimeCount(){
  /* Disable SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Disable);
  /* Clear SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Clear);
}

void mDelay(u32 nTime)
{
  startTimeCount();
  while(gwTimingDelay < nTime);
  stopTimeCount();
}


void mDelayClock(u32 nTime)
{
  u32 end_ts = getTime() + nTime;
  while(gwTimingDelay < end_ts);
}

void mDelayMusic(u32 nTime)
{
  u32 end_ts = getTime() + nTime;
  while(gwTimingDelay < end_ts){
    musicHandler();
  }
}

void mDelayMusicLogic(u32 nTime, int* state)
{
  u32 end_ts = getTime() + nTime;
  unsigned char field;
  while(gwTimingDelay < end_ts && !detectWhiteBorder(state)){
    musicHandler();
    centerInfraRed(SENSOR, &field);
    //
    TxDString("\nSEEKING SENSOR VALUE: ");
    TxDByte16(field);
    TxDString("\n");
    //
    if (field >= thresholdInfrared){
      *state = CHASING;
      break;
    }
  }
}

void StartDiscount(s32 StartTime)
{
  gw1msCounter = StartTime;
}

u8 CheckTimeOut(void)
{
  // Check timeout
  // Return: 0 is false, 1 is true(timeout occurred)

  if(gw1msCounter == 0)
    return 1;
  else
    return 0;
}
