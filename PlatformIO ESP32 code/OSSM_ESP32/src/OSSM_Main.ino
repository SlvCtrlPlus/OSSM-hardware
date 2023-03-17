#include <Arduino.h>          // Basic Needs
#include <ESP_FlexyStepper.h> // Current Motion Control
#include <Encoder.h>          // Used for the Remote Encoder Input
#include <Wire.h>             // Used for i2c connections (Remote OLED Screen)

#include "OSSM_Config.h" // START HERE FOR Configuration
#include "OSSM_PinDef.h" // This is where you set pins specific for your board
#include "OssmUi.h"      // Separate file that helps contain the OLED screen functions
#include "Stroke_Engine_Helper.h"
#include "Utilities.h" // Utility helper functions - wifi update and homing
#include "SlvCtrlPlus.h"

// Homing
volatile bool g_has_not_homed = true;
bool REMOTE_ATTACHED = false;

// OSSM name setup
const char *ossmId = "OSSM1";
volatile int encoderButtonPresses = 0; // increment for each click
volatile long lastEncoderButtonPressMillis = 0;

IRAM_ATTR void encoderPushButton()
{
    // TODO: Toggle position mode
    // g_encoder.write(0);       // Reset on Button Push
    // ossm.g_ui.NextFrame();         // Next Frame on Button Push

    // debounce check
    long currentTime = millis();
    if ((currentTime - lastEncoderButtonPressMillis) > 200)
    {
        // run interrupt if not run in last 50ms
        encoderButtonPresses++;
        lastEncoderButtonPressMillis = currentTime;
    }
}

// Current command state
// volatile float strokePercentage = 0;
// volatile float ossm.speedPercentage = 0;
// volatile float deceleration = 0;

// Create tasks for checking pot input or web server control, and task to handle
// planning the motion profile (this task is high level only and does not pulse
// the stepper!)
TaskHandle_t getInputTask = nullptr;
TaskHandle_t motionTask = nullptr;
TaskHandle_t estopTask = nullptr;
TaskHandle_t oledTask = nullptr;

// Declarations
// TODO: Document functions
// void setLedRainbow(CRGB leds[]);
void getUserInputTask(void *pvParameters);
void motionCommandTask(void *pvParameters);
void estopResetTask(void *pvParameters);

bool stopSwitchTriggered = 0;

// create the OSSM hardware object
OSSM ossm;

SlvCtrlPlus slvCtrlPlus(&ossm);

///////////////////////////////////////////
////
////  VOID SETUP -- Here's where it's hiding
////
///////////////////////////////////////////

void setup()
{
    ossm.startLeds();
    Serial.begin(9600);
    LogDebug("\n Starting");
    pinMode(ENCODER_SWITCH, INPUT_PULLDOWN); // Rotary Encoder Pushbutton
    attachInterrupt(digitalPinToInterrupt(ENCODER_SWITCH), encoderPushButton, RISING);

    ossm.setup();

    delay(100);

    ossm.findHome();

    ossm.setRunMode();

    // Kick off the http and motion tasks - they begin executing as soon as they
    // are created here! Do not change the priority of the task, or do so with
    // caution. RTOS runs first in first out, so if there are no delays in your
    // tasks they will prevent all other code from running on that core!
    xTaskCreatePinnedToCore(getUserInputTask,   /* Task function. */
                            "getUserInputTask", /* name of task. */
                            10000,              /* Stack size of task */
                            NULL,               /* parameter of the task */
                            1,                  /* priority of the task */
                            &getInputTask,      /* Task handle to keep track of created task */
                            0);                 /* pin task to core 0 */
    delay(100);
    xTaskCreatePinnedToCore(motionCommandTask,   /* Task function. */
                            "motionCommandTask", /* name of task. */
                            20000,               /* Stack size of task */
                            NULL,                /* parameter of the task */
                            1,                   /* priority of the task */
                            &motionTask,         /* Task handle to keep track of created task */
                            0);                  /* pin task to core 0 */

    delay(100);
    xTaskCreatePinnedToCore(estopResetTask,   /* Task function. */
                            "estopResetTask", /* name of task. */
                            10000,            /* Stack size of task */
                            NULL,             /* parameter of the task */
                            1,                /* priority of the task */
                            &estopTask,       /* Task handle to keep track of created task */
                            0);               /* pin task to core 0 */

    delay(100);
    ossm.g_ui.UpdateMessage("OSSM Ready to Play");

    slvCtrlPlus.comm_setup();
} // Void Setup()

///////////////////////////////////////////
////
////
////   VOID LOOP - Hides here
////
////
///////////////////////////////////////////

void loop()
{
    switch (ossm.rightKnobMode)
    {
        case MODE_STROKE:
            ossm.g_ui.UpdateState("STROKE", static_cast<int>(ossm.speedPercentage),
                                  static_cast<int>(ossm.strokePercentage + 0.5f));
            break;
        case MODE_DEPTH:
            ossm.g_ui.UpdateState("DEPTH", static_cast<int>(ossm.speedPercentage),
                                  static_cast<int>(ossm.depthPercentage + 0.5f));
            break;
        case MODE_SENSATION:
            ossm.g_ui.UpdateState("SENSTN", static_cast<int>(ossm.speedPercentage),
                                  static_cast<int>(ossm.sensationPercentage + 0.5f));
            break;
        case MODE_PATTERN:
            ossm.g_ui.UpdateState("PATTRN", static_cast<int>(ossm.speedPercentage),
                                  ossm.strokePattern * 100 / (ossm.strokePatternCount - 1));
            break;
    }
    ossm.g_ui.UpdateScreen();
    slvCtrlPlus.comm_loop();
}

///////////////////////////////////////////
////
////
////  freeRTOS multitasking
////
////
///////////////////////////////////////////

void estopResetTask(void *pvParameters)
{
    for (;;)
    {
        if (stopSwitchTriggered == 1)
        {
            while ((ossm.getAnalogAveragePercent(SPEED_POT_PIN, 50) > 2))
            {
                vTaskDelay(1);
            }
            stopSwitchTriggered = 0;
            vTaskResume(motionTask);
            vTaskResume(getInputTask);
        }
        vTaskDelay(100);
    }
}

// Task to read settings from server - only need to check this when in WiFi
// control mode
void getUserInputTask(void *pvParameters)
{
    for (;;) // tasks should loop forever and not return - or will throw error in
             // OS
    {
        // LogDebug("Speed: " + String(ossm.speedPercentage) + "\% Stroke: " + String(ossm.strokePercentage) +
        //          "\% Distance to target: " + String(ossm.stepper.getDistanceToTargetSigned()) + " steps?");

        ossm.updateAnalogInputs();

        ossm.speedPercentage > 1 ? ossm.stepper.releaseEmergencyStop() : ossm.stepper.emergencyStop();

        // We should scale these values with initialized settings not hard coded
        // values!
        if (ossm.speedPercentage > commandDeadzonePercentage)
        {
            ossm.stepper.setSpeedInMillimetersPerSecond(ossm.maxSpeedMmPerSecond * ossm.speedPercentage / 100.0);
            ossm.stepper.setAccelerationInMillimetersPerSecondPerSecond(
                ossm.maxSpeedMmPerSecond * ossm.speedPercentage * ossm.speedPercentage / ossm.accelerationScaling);
            // We do not set deceleration value here because setting a low decel when
            // going from high to low speed causes the motor to travel a long distance
            // before slowing. We should only change decel at rest
        }
        vTaskDelay(50); // let other code run!
    }
}

void motionCommandTask(void *pvParameters)
{
    for (;;) // tasks should loop forever and not return - or will throw error in
             // OS
    {
        switch (ossm.activeRunMode)
        {
            case ossm.simpleMode:
                ossm.runPenetrate();
                break;

            case ossm.strokeEngineMode:
                ossm.runStrokeEngine();
                break;
        }
    }
}
