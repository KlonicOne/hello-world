#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <timers.h>

// Tasks
void taskGetMotion(void *pvParameters);
void taskProcessing(void *pvParameters);
void taskServoControl(void *pvParameters);

// Helper functions
void setServoPosAngle_deg(int angle_deg, Servo *p_servo);
void timer1_callback(TimerHandle_t timer);

// Defines
#define LID_INITIAL_POS_DEG (90u)
#define MOTIONFILTER_SIZE (10u)
#define MOTIONFILTER_DETECT_BUSY (0.8f)
#define MOTIONFILTER_DETECT_FREE (0.2f)
#define BUSY_TIME_MIN (10u)
#define FREE_TIME_MIN (30u)
#define LID_CLOSING_TIME (2u)

// Classes
class SwTimer {
public:
  SwTimer();          // Constructor
  virtual ~SwTimer(); // Destructor

  void startTimer(void);
  void stopTimer(void);
  bool isTimeExceeded(int diffTimeToCheck);
  bool isTimerRunning(void);

private:
  int timeReference;
  bool timerRunning;
};

// Globals
int systime_s = 0;

int motionInPin = 7;       // motion sensor pin 7
int motionDetectedRaw = 0; // variable to store the read value
float motionDetectedFilt = 0.0;
int motionDetectedRawBuffer[MOTIONFILTER_SIZE];

enum tlcState {
  TLC_IDLE = 0,
  TLC_ARMING,
  TLC_ARMED,
  TLC_RELEASING,
  TLC_RELEASED
} tlcState,
    tlcState_nextState;

TimerHandle_t timer1;

SwTimer timerToiletBusy;
SwTimer timerToiletFree;
SwTimer timerLidClosing;

Servo lidServo;
int offset_lid = 0;
int angle_lid_open = 90;
int angle_lid_close = 0;
enum lidServoState { OPEN_LID = 0, CLOSE_LID, MAX_LID_STATE } lidServoState;

// Init
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  // Init pins
  pinMode(motionInPin, INPUT); // sets the digital pin 7 as input
  // Init servo
  lidServo.attach(3);
  lidServo.write(LID_INITIAL_POS_DEG);

  // Init tasks
  xTaskCreate(taskGetMotion, "Measurement", // A name just for humans
              128, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL, 3, // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                       // highest, and 0 being the lowest.
              NULL);

  xTaskCreate(taskProcessing, "Processing", // A name just for humans
              128, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL, 1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                       // highest, and 0 being the lowest.
              NULL);

  // Now set up two tasks to run independently.
  xTaskCreate(taskServoControl, "ServoControl", // A name just for humans
              128, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL, 2, // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                       // highest, and 0 being the lowest.
              NULL);

  // Init SW functions
  timer1 = xTimerCreate("TimerOneSec", pdMS_TO_TICKS(1000), pdTRUE, 0,
                        timer1_callback);
  if (timer1 == NULL) {
    Serial.println("Timer can not be created");
  } else {
    // Start timer
    if (xTimerStart(timer1, 0) == pdPASS) { // Start the scheduler
      vTaskStartScheduler();
    }
  }

  // Init locals
  tlcState = TLC_IDLE;
  tlcState_nextState = TLC_IDLE;
  lidServoState = OPEN_LID;
  timerToiletBusy.stopTimer();
  timerToiletFree.stopTimer();
  timerLidClosing.stopTimer();
}

void loop() {
  // Empty. Things are done in Tasks.
}

void taskGetMotion(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    int motionDetectionBufferSum = 0;

    motionDetectedRaw = digitalRead(motionInPin); // read the input pin

    // Need last element free to add new value to buffer, so shift to end
    for (int i = (MOTIONFILTER_SIZE - 1); i >= 1; i--) {
      motionDetectedRawBuffer[i] = motionDetectedRawBuffer[i - 1];
      motionDetectionBufferSum += motionDetectedRawBuffer[i];
    }
    // append new value at the front
    motionDetectedRawBuffer[0] = motionDetectedRaw;
    motionDetectionBufferSum += motionDetectedRaw;
    // Average
    motionDetectedFilt = (float)motionDetectionBufferSum / MOTIONFILTER_SIZE;

    // Debug out
    // Serial.print(motionDetectedFilt);
    // Serial.println();

    vTaskDelay(10); // one tick delay (15ms) in between reads for stability
  }
}

void taskProcessing(void *pvParameters) {
  (void)pvParameters;

  for (;;) {

    // Take over next state as current state
    tlcState = tlcState_nextState;

    switch (tlcState) {
    case (TLC_IDLE):
      lidServoState = OPEN_LID;
      if (motionDetectedFilt >= MOTIONFILTER_DETECT_BUSY) {
        // Toilet busy
        tlcState_nextState = TLC_ARMING;
        timerToiletBusy.startTimer();
      } else {
        timerToiletBusy.stopTimer();
        timerToiletFree.stopTimer();
        timerLidClosing.stopTimer();
      }
      break;

    case (TLC_ARMING):
      lidServoState = OPEN_LID;
      if (motionDetectedFilt <= MOTIONFILTER_DETECT_FREE) {
        // Toilet free for short time only busy go back do nothing
        tlcState_nextState = TLC_IDLE;
        timerToiletBusy.stopTimer();
      } else if (timerToiletBusy.isTimerRunning()) {
        // Busy timer running
        if (timerToiletBusy.isTimeExceeded(BUSY_TIME_MIN)) {
          // Toilet busy for enough time
          tlcState_nextState = TLC_ARMED;
          timerToiletBusy.stopTimer();
        }
      } else {
        // Timer not running, but it should so back to TLC_IDLE
        tlcState_nextState = TLC_IDLE;
      }
      break;

    case (TLC_ARMED):
      lidServoState = OPEN_LID;
      if (motionDetectedFilt <= MOTIONFILTER_DETECT_FREE) {
        tlcState_nextState = TLC_RELEASING;
        timerToiletFree.startTimer();
      }
      break;

    case (TLC_RELEASING):
      lidServoState = OPEN_LID;
      if (motionDetectedFilt >= MOTIONFILTER_DETECT_BUSY) {
        tlcState_nextState = TLC_ARMED;
        timerToiletFree.stopTimer();
      } else if (timerToiletFree.isTimerRunning()) {
        if (timerToiletFree.isTimeExceeded(FREE_TIME_MIN)) {
          // Toilet free again so close lid in next state
          tlcState_nextState = TLC_RELEASED;
          timerToiletFree.stopTimer();
          timerLidClosing.stopTimer();
        }
      } else {
        // Timer should be running if not go back to armed
		timerToiletFree.stopTimer();
        tlcState_nextState = TLC_ARMED;
      }
      break;

    case (TLC_RELEASED):
      if (motionDetectedFilt <= MOTIONFILTER_DETECT_FREE) {
        if (timerLidClosing.isTimerRunning()) {
          // Only if value doesn't show motion we close the lid
          lidServoState = CLOSE_LID;
          if (timerLidClosing.isTimeExceeded(LID_CLOSING_TIME)) {
            tlcState_nextState = TLC_IDLE;
            timerLidClosing.stopTimer();
          }
        } else {
          timerLidClosing.startTimer();
        }
      } else {
        // We saw again some movement back to armed
        timerLidClosing.stopTimer();
        tlcState_nextState = TLC_ARMED;
      }
      break;

    default:
      tlcState = TLC_IDLE;
    }

	// Debug out
	// Serial.print("stat ");
	// Serial.print(tlcState);
	// Serial.print("; nxtstat ");
	// Serial.print(tlcState_nextState);
	// Serial.println();

    vTaskDelay(20); // one tick delay (15ms) in between reads for stability
  }
}

void taskServoControl(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  for (;;) {
    switch (lidServoState) {
    case (OPEN_LID):
      setServoPosAngle_deg((angle_lid_open - offset_lid), &lidServo);
      break;
    case (CLOSE_LID):
      setServoPosAngle_deg((angle_lid_close - offset_lid), &lidServo);
      break;
    default:
      lidServoState = OPEN_LID;
    }
    vTaskDelay(25); // one tick delay (15ms) in between reads for stability
  }
}

// ---------------------------------------------------

void setServoPosAngle_deg(int angle_deg, Servo *p_servo) {
  // Do limiting
  if (angle_deg < 5) {
    angle_deg = 5;
  } else if (angle_deg > 175) {
    angle_deg = 175;
  } else {
    // do nothing
  }
  // write out
  p_servo->write(angle_deg);
}

void timer1_callback(TimerHandle_t timer) { systime_s++; }

// Class member implementation -------------

SwTimer::SwTimer(){

};

SwTimer::~SwTimer(){

};

void SwTimer::startTimer(void) {
  this->timeReference = systime_s;
  this->timerRunning = true;
};

void SwTimer::stopTimer(void) {
  this->timeReference = 0;
  this->timerRunning = false;
};

bool SwTimer::isTimeExceeded(int diffTimeToCheck) {
  int targetTime = this->timeReference + diffTimeToCheck;
  bool timerElapsed = false;

  if (this->timerRunning && (targetTime <= systime_s)) {
    // timer elapsed
    timerElapsed = true;
  }

  return (timerElapsed);
};

bool SwTimer::isTimerRunning(void) { return (this->timerRunning); };
