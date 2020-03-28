#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <U8glib.h>
#include <Servo.h>

// Tasks
void
taskGetDistanceMeasurement (void *pvParameters);
void
taskDisplayManager (void *pvParameters);
void
taskServoControl (void *pvParameters);

// Task handler
TaskHandle_t Handle_DistanceMeasurement;
TaskHandle_t Handle_DisplayManager;

// Helper functions
void
draw (void);
void
setServoPosAngle_deg (int angle_deg, Servo* p_servo);

// Defines
#define MAX_READ_BYTES (64)
#define COMMAND_BYTE_LENGTH (4)

// Globals
uint8_t commandMeasure[COMMAND_BYTE_LENGTH] = { 0xC0, 0x40, 0x00, 0xEE };
uint8_t commandBacklightOff[COMMAND_BYTE_LENGTH] = { 0xC0, 0x48, 0x00, 0x62 };
uint8_t readData[MAX_READ_BYTES] = { 0 };
uint16_t u16_distance_mm = 0;
Servo servo_plate;
int offset_plate = 0;
int angle_plate = 0;
Servo servo_laser;
int offset_laser = 0;
int angle_laser = 0;

// Display constructor
U8GLIB_SH1106_128X64 u8g (U8G_I2C_OPT_NONE); // I2C / TWI

// Init
void
setup () {
  // initialize serial communication at 9600 bits per second:
  Serial.begin (9600);
  Serial1.begin (9600);

  while (!Serial) {
    ; // wait for serial port to connect.
  }

  xTaskCreate (taskGetDistanceMeasurement, (const portCHAR *) "Measurement" // A name just for humans
               , 128 // This stack size can be checked & adjusted by reading the Stack Highwater
               , NULL, 3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
               , &Handle_DistanceMeasurement);

  xTaskCreate (taskDisplayManager, (const portCHAR *) "Measurement" // A name just for humans
               , 128 // This stack size can be checked & adjusted by reading the Stack Highwater
               , NULL, 1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
               , &Handle_DisplayManager);

  // Now set up two tasks to run independently.
  xTaskCreate (taskServoControl, (const portCHAR *) "ServoControl" // A name just for humans
               , 128 // This stack size can be checked & adjusted by reading the Stack Highwater
               , NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
               , NULL);

  // Color mode
  u8g.setColorIndex (1);         // pixel on

  // Backlight off
  Serial.write (commandBacklightOff, COMMAND_BYTE_LENGTH);

  servo_plate.attach (3);
  servo_plate.write (90);
  servo_laser.attach (5);
  servo_laser.write (90);
}

void
loop () {
  // Empty. Things are done in Tasks.
}

void
taskGetDistanceMeasurement (void *pvParameters)  // This is a task.
    {
  (void) pvParameters;
  for (;;) {
    uint8_t numBytes = 0;
    uint8_t i = 0;
    uint16_t buffer = 0;

    numBytes = Serial1.available ();

    if (numBytes > 0 && numBytes <= MAX_READ_BYTES) {
      for (i = 0; i < numBytes; i++) {
        readData[i] = Serial1.read ();
        // debug out
        //Serial.print(readData[i], HEX);
        //Serial.print(" ");
      }
      //Serial.println();
      // distance
      buffer = ((uint16_t) readData[7] << 8) + (uint16_t) readData[6];
      u16_distance_mm = (uint16_t) (buffer * 0.05);
    }
    else {
      numBytes = 0;
      u16_distance_mm = 0;
    }

    // Send data to laser tool
    Serial1.write (commandMeasure, COMMAND_BYTE_LENGTH);

    // Print out data
    Serial.print (xTaskGetTickCount ());
    Serial.print (",");
    Serial.print (angle_plate, DEC);
    Serial.print (",");
    Serial.print (angle_laser, DEC);
    Serial.print (",");
    Serial.println (u16_distance_mm);

    vTaskDelay (75);  // one tick delay (15ms) in between reads for stability
  }
}

void
taskDisplayManager (void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    u8g.firstPage ();
    do {
      draw ();
    }
    while (u8g.nextPage ());

    vTaskDelay (10);  // one tick delay (15ms) in between reads for stability
  }
}

void
taskServoControl (void *pvParameters)  // This is a task.
    {
  (void) pvParameters;

  for (;;) {
    static bool up_b = true;

    // change pos for plate
    if (up_b == true)
    {
      angle_plate++;
      angle_laser++;
      if (angle_plate >= 180)
      {
        up_b = false;
      }
    }
    else
    {
      angle_plate--;
      angle_laser--;
      if (angle_plate <= 0)
      {
        up_b = true;
      }
    }

    setServoPosAngle_deg ((angle_plate - offset_plate), &servo_plate);
    setServoPosAngle_deg ((angle_laser - offset_laser), &servo_laser);
    vTaskDelay (15);  // one tick delay (15ms) in between reads for stability
  }
}

// ---------------------------------------------------
// Own helper functions
void
draw (void) {
  char cstr[10] = { 0 };

  // prepare string to out
  sprintf (cstr, "%d", u16_distance_mm);

  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont (u8g_font_unifont); // some noise pattern
  u8g.drawStr (10, 20, "PLR15 [mm]:");
  u8g.setFont (u8g_font_fub17);
  u8g.drawStr (20, 50, (const char*) cstr);
}

void
setServoPosAngle_deg (int angle_deg, Servo* p_servo) {
  // Do limiting
  if (angle_deg < 5) {
    angle_deg = 5;
  }
  else if (angle_deg > 175) {
    angle_deg = 175;
  }
  else {
    // do nothing
  }
  // write out
  p_servo->write (angle_deg);
}
