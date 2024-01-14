#include "Utilities.h"

#include <string>

#include "Stroke_Engine_Helper.h"

void OSSM::setup()
{
    LogDebug("Software version");
    LogDebug(SW_VERSION);
    g_ui.Setup();
    delay(50);
    String message = "";
    message += "V";
    message += SW_VERSION;
    message += " Booting up!";
    OssmUi::UpdateMessage(message);
#ifdef INITIAL_SETUP
    FastLED.setBrightness(150);
    fill_rainbow(ossmleds, NUM_LEDS, 34, 1);
    FastLED.show();
    writeEepromSettings();
    updateFirmware();
#endif
    readEepromSettings();
    initializeStepperParameters();
    initializeInputs();
    strcpy(Id, ossmId);
    delay(500);
}

[[noreturn]] void OSSM::runPenetrate()
{
    // poll at 200Hz for when motion is complete
    for (;;)
    {
        while ((stepper.getDistanceToTargetSigned() != 0) || (strokePercentage <= commandDeadzonePercentage) ||
               (speedPercentage <= commandDeadzonePercentage))
        {
            vTaskDelay(5); // wait for motion to complete and requested stroke more than zero
        }

        float targetPosition = (strokePercentage / 100.0f) * maxStrokeLengthMm;
        float currentStrokeMm = abs(targetPosition);

        stepper.setDecelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage * speedPercentage /
                                                               accelerationScaling);
        stepper.setTargetPositionInMillimeters(targetPosition);
        vTaskDelay(2);
        // LogDebugFormatted("Moving stepper to position %ld \n", static_cast<long int>(targetPosition));

        while ((stepper.getDistanceToTargetSigned() != 0) || (strokePercentage <= commandDeadzonePercentage) ||
               (speedPercentage <= commandDeadzonePercentage))
        {
            vTaskDelay(5); // wait for motion to complete, since we are going back to
                           // zero, don't care about stroke value
        }
        targetPosition = 0;
        // Serial.printf("Moving stepper to position %ld \n", targetPosition);
        vTaskDelay(1);
        stepper.setDecelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage * speedPercentage /
                                                               accelerationScaling);
        stepper.setTargetPositionInMillimeters(targetPosition);
        vTaskDelay(1);
        // if (currentStrokeMm > 1)
        numberStrokes++;
        travelledDistanceMeters += (0.002 * currentStrokeMm);
        updateLifeStats();
    }
}
void OSSM::handleStopCondition() // handles e-stop condition
{
    // check is speed is greater than deadzone value and emergency stop if not
    if (speedPercentage <= commandDeadzonePercentage)
    {
        if (!isStopped)
        {
            LogDebug("Speed: " + String(speedPercentage) + "\% Stroke: " + String(strokePercentage) +
                     "\% Distance to target: " + String(stepper.getDistanceToTargetSigned()) + " steps");
            stepper.emergencyStop(true);
            LogDebug("Emergency Stop");
            isStopped = true;
            delay(100);
        }
    }
    else
    {
        // release emergency stop
        if (isStopped)
        {
            LogDebug("Speed: " + String(speedPercentage) + "\% Stroke: " + String(strokePercentage) +
                     "\% Distance to target: " + String(stepper.getDistanceToTargetSigned()) + " steps");
            stepper.releaseEmergencyStop();
            LogDebug("Emergency Stop Released");
            isStopped = false;
            delay(100);
        }
    }
}

bool isChangeSignificant(float oldPct, float newPct)
{
    return oldPct != newPct && (abs(newPct - oldPct) > 2 || newPct == 0 || newPct == 100);
}

float calculateSensation(float sensationPercentage)
{
    return float((sensationPercentage * 200.0) / 100.0) - 100.0f;
}

[[noreturn]] void OSSM::runStrokeEngine()
{
    stepper.stopService();

    machineGeometry strokingMachine = {.physicalTravel = abs(maxStrokeLengthMm), .keepoutBoundary = 6.0};
    StrokeEngine Stroker;

    Stroker.begin(&strokingMachine, &servoMotor);
    Stroker.thisIsHome();

    float lastSpeedPercentage = speedPercentage;
    float lastStrokePercentage = strokePercentage;
    float lastDepthPercentage = depthPercentage;
    float lastSensationPercentage = sensationPercentage;
    int lastEncoderButtonPresses = encoderButtonPresses;
    strokePattern = 0;
    strokePatternCount = Stroker.getNumberOfPattern();

    Stroker.setSensation(calculateSensation(sensationPercentage), true);

    Stroker.setPattern(int(strokePattern), true);
    Stroker.setDepth(0.01f * depthPercentage * abs(maxStrokeLengthMm), true);
    Stroker.setStroke(0.01f * strokePercentage * abs(maxStrokeLengthMm), true);
    Stroker.moveToMax(10 * 3);
    LogDebug(Stroker.getState());
    //    OssmUi::UpdateMessage(Stroker.getPatternName(strokePattern));

    for (;;)
    {
        LogDebug("looping");
        if (isChangeSignificant(lastSpeedPercentage, speedPercentage))
        {
            LogDebugFormatted("changing speed: %f\n", speedPercentage * 3);
            if (speedPercentage == 0)
            {
                Stroker.stopMotion();
            }
            else if (Stroker.getState() == READY)
            {
                Stroker.startPattern();
            }

            Stroker.setSpeed(speedPercentage * 3, true); // multiply by 3 to get to sane thrusts per minute speed
            lastSpeedPercentage = speedPercentage;
        }

        int buttonPressCount = encoderButtonPresses - lastEncoderButtonPresses;
        if (!modeChanged && buttonPressCount > 0 && (millis() - lastEncoderButtonPressMillis) > 200)
        {
            LogDebugFormatted("switching mode pre: %i %i\n", rightKnobMode, buttonPressCount);

            if (buttonPressCount > 1)
            {
                rightKnobMode = MODE_PATTERN;
            }
            else if (strokePattern == 0)
            {
                rightKnobMode += 1;
                if (rightKnobMode > 1)
                {
                    rightKnobMode = 0;
                }
            }
            else
            {
                rightKnobMode += 1;
                if (rightKnobMode > 2)
                {
                    rightKnobMode = 0;
                }
            }

            LogDebugFormatted("switching mode: %i\n", rightKnobMode);

            modeChanged = true;
            lastEncoderButtonPresses = encoderButtonPresses;
        }

        if (lastStrokePercentage != strokePercentage)
        {
            float newStroke = 0.01f * strokePercentage * abs(maxStrokeLengthMm);
            LogDebugFormatted("change stroke: %f %f\n", strokePercentage, newStroke);
            Stroker.setStroke(newStroke, true);
            lastStrokePercentage = strokePercentage;
        }

        if (lastDepthPercentage != depthPercentage)
        {
            float newDepth = 0.01f * depthPercentage * abs(maxStrokeLengthMm);
            LogDebugFormatted("change depth: %f %f\n", depthPercentage, newDepth);
            Stroker.setDepth(newDepth, false);
            lastDepthPercentage = depthPercentage;
        }

        if (lastSensationPercentage != sensationPercentage)
        {
            float newSensation = calculateSensation(sensationPercentage);
            LogDebugFormatted("change sensation: %f, %f\n", sensationPercentage, newSensation);
            Stroker.setSensation(newSensation, false);
            lastSensationPercentage = sensationPercentage;
        }

        if (!modeChanged && changePattern != 0)
        {
            strokePattern += changePattern;

            if (strokePattern < 0)
            {
                strokePattern = Stroker.getNumberOfPattern() - 1;
            }
            else if (strokePattern >= Stroker.getNumberOfPattern())
            {
                strokePattern = 0;
            }

            LogDebug(Stroker.getPatternName(strokePattern));

            Stroker.setPattern(int(strokePattern), false); // Pattern, index must be < Stroker.getNumberOfPattern()
            //            OssmUi::UpdateMessage(Stroker.getPatternName(strokePattern));

            modeChanged = true;
        }

        vTaskDelay(400);
    }
}

String getPatternJSON(StrokeEngine Stroker)
{
    String JSON = "[{\"";
    for (size_t i = 0; i < Stroker.getNumberOfPattern(); i++)
    {
        JSON += String(Stroker.getPatternName(i));
        JSON += "\": ";
        JSON += String(i, DEC);
        if (i < Stroker.getNumberOfPattern() - 1)
        {
            JSON += "},{\"";
        }
        else
        {
            JSON += "}]";
        }
    }
    LogDebug(JSON);
    return JSON;
}

void OSSM::setRunMode()
{
    int initialEncoderFlag = encoderButtonPresses;
    int runModeVal;
    int encoderVal;
    while (initialEncoderFlag == encoderButtonPresses)
    {
        encoderVal = abs(g_encoder.read());
        runModeVal = (encoderVal % (2 * runModeCount)) / 2; // scale by 2 because encoder counts by 2
        LogDebug("encoder: ");
        LogDebug(encoderVal);
        LogDebugFormatted("%d encoder count \n", encoderVal);
        LogDebugFormatted("%d runModeVal \n", runModeVal);
        switch (runModeVal)
        {
            case simpleMode:
                OssmUi::UpdateMessage("Simple Penetration");
                activeRunMode = simpleMode;
                break;

            case strokeEngineMode:
                OssmUi::UpdateMessage("Stroke Engine");
                activeRunMode = strokeEngineMode;
                break;

            default:
                OssmUi::UpdateMessage("Simple Penetration");
                activeRunMode = simpleMode;
                break;
        }
    }
    g_encoder.write(0); // reset encoder to zero
}

void OSSM::initializeStepperParameters()
{
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    float stepsPerMm = motorStepPerRevolution / (pulleyToothCount * beltPitchMm);
    stepper.setStepsPerMillimeter(stepsPerMm);
    stepper.setLimitSwitchActive(LIMIT_SWITCH_PIN);
    stepper.startAsService(); // Kinky Makers - we have modified this function
    // from default library to run on core 1 and suggest you don't run anything else on that core.
}

void OSSM::initializeInputs()
{
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(WIFI_RESET_PIN, INPUT_PULLDOWN);
    pinMode(WIFI_CONTROL_TOGGLE_PIN, LOCAL_CONTROLLER); // choose between WIFI_CONTROLLER and LOCAL_CONTROLLER
    // Set analog pots (control knobs)
    pinMode(SPEED_POT_PIN, INPUT);
    adcAttachPin(SPEED_POT_PIN);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); // allows us to read almost full 3.3V range
}

bool OSSM::findHome()
{
    maxStrokeLengthMm = sensorlessHoming();
    if (maxStrokeLengthMm > 20)
    {
        return true;
    }
    return false;

    LogDebug("Homing returning");
}

float OSSM::sensorlessHoming()
{
    // find retracted position, mark as zero, find extended position, calc total length, subtract 2x offsets and
    // record length.
    //  move to offset and call it zero. homing complete.

    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

    float currentLimit = 1.5;
    currentSensorOffset = (getAnalogAveragePercent(36, 1000));
    float current;
    float measuredStrokeMm;
    stepper.setAccelerationInMillimetersPerSecondPerSecond(1000);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(10000);

    OssmUi::UpdateMessage("Finding Home Sensorless");

    // disable motor briefly in case we are against a hard stop.
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    delay(600);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    delay(100);

    int limitSwitchActivated = digitalRead(LIMIT_SWITCH_PIN);
    LogDebugFormatted(
        "%d,%f,%d",
        getAnalogAveragePercent(36, 500) - currentSensorOffset,
        stepper.getCurrentPositionInMillimeters(),
        limitSwitchActivated
    );

    // find reverse limit

    stepper.setSpeedInMillimetersPerSecond(25);
    stepper.setTargetPositionInMillimeters(-400);
    current = getAnalogAveragePercent(36, 200) - currentSensorOffset;

    while (current < currentLimit && limitSwitchActivated != 0)
    {
        current = getAnalogAveragePercent(36, 25) - currentSensorOffset;
        limitSwitchActivated = digitalRead(LIMIT_SWITCH_PIN);
        LogDebugFormatted("%f,%f,%d", current, stepper.getCurrentPositionInMillimeters(), limitSwitchActivated);
    }
    if (limitSwitchActivated == 0)
    {
        stepper.setTargetPositionToStop();
        delay(100);
        stepper.setSpeedInMillimetersPerSecond(10);
        stepper.moveRelativeInMillimeters((1 * maxStrokeLengthMm) - strokeZeroOffsetmm);
        LogDebug("OSSM has moved out, will now set new home?");
        stepper.setCurrentPositionAsHomeAndStop();
        return -maxStrokeLengthMm;
    }
    stepper.setTargetPositionToStop();
    stepper.moveRelativeInMillimeters(strokeZeroOffsetmm); //"move to" is blocking
    stepper.setCurrentPositionAsHomeAndStop();
    OssmUi::UpdateMessage("Checking Stroke");
    delay(100);

    // find forward limit

    stepper.setSpeedInMillimetersPerSecond(25);
    stepper.setTargetPositionInMillimeters(400);
    delay(300);
    current = getAnalogAveragePercent(36, 200) - currentSensorOffset;
    while (current < currentLimit)
    {
        current = getAnalogAveragePercent(36, 25) - currentSensorOffset;
        if (stepper.getCurrentPositionInMillimeters() > 90)
        {

            LogDebugFormatted("%f,%f", current, stepper.getCurrentPositionInMillimeters());
        }
    }

    stepper.setTargetPositionToStop();
    stepper.moveRelativeInMillimeters(-strokeZeroOffsetmm);
    measuredStrokeMm = -stepper.getCurrentPositionInMillimeters();
    stepper.setCurrentPositionAsHomeAndStop();
    LogDebugFormatted("Sensorless Homing complete!  %f mm", measuredStrokeMm);
    OssmUi::UpdateMessage("Homing Complete");
    // digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    // delay(500);
    // digitalWrite(MOTOR_ENABLE_PIN, LOW);
    LogDebugFormatted("Stroke: %f", measuredStrokeMm);
    return measuredStrokeMm;
}
void OSSM::sensorHoming()
{
    // find limit switch and then move to end of stroke and call it zero
    stepper.setAccelerationInMillimetersPerSecondPerSecond(300);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(10000);

    LogDebug("OSSM will now home");
    OssmUi::UpdateMessage("Finding Home Switch");
    stepper.setSpeedInMillimetersPerSecond(15);
    stepper.moveToHomeInMillimeters(1, 25, 300, LIMIT_SWITCH_PIN);
    LogDebug("OSSM has homed, will now move out to max length");
    OssmUi::UpdateMessage("Moving to Max");
    stepper.setSpeedInMillimetersPerSecond(10);
    stepper.moveToPositionInMillimeters((-1 * maxStrokeLengthMm) - strokeZeroOffsetmm);
    LogDebug("OSSM has moved out, will now set new home");
    stepper.setCurrentPositionAsHomeAndStop();
    LogDebug("OSSM should now be home and happy");
}

int OSSM::readEepromSettings()
{
    LogDebug("read eeprom");
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, hardwareVersion);
    EEPROM.get(4, numberStrokes);
    EEPROM.get(12, travelledDistanceMeters);
    EEPROM.get(20, lifeSecondsPoweredAtStartup);

    if (numberStrokes == NAN || numberStrokes <= 0)
    {
        hardwareVersion = HW_VERSION;
        numberStrokes = 0;
        travelledDistanceMeters = 0;
        lifeSecondsPoweredAtStartup = 0;
        writeEepromSettings();
    }

    return hardwareVersion;
}

void OSSM::writeEepromSettings()
{
    // Be very careful with this so you don't break your configuration!
    LogDebug("write eeprom");
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, HW_VERSION);
    EEPROM.put(4, 0);
    EEPROM.put(12, 0);
    EEPROM.put(20, 0);
    EEPROM.commit();
    LogDebug("eeprom written");
}
void OSSM::writeEepromLifeStats()
{
    // Be very careful with this so you don't break your configuration!
    LogDebug("writing eeprom life stats...");
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(4, numberStrokes);
    EEPROM.put(12, travelledDistanceMeters);
    EEPROM.put(20, lifeSecondsPowered);
    EEPROM.commit();
    LogDebug("eeprom written");
}

void OSSM::updateLifeStats()
{
    float minutes;
    float hours;
    float days;
    double travelledDistanceKilometers;

    travelledDistanceKilometers = (0.001 * travelledDistanceMeters);
    lifeSecondsPowered = (0.001 * millis()) + lifeSecondsPoweredAtStartup;
    minutes = lifeSecondsPowered / 60;
    hours = minutes / 60;
    days = hours / 24;
    if ((millis() - lastLifeUpdateMillis) > 5000)
    {
        LogDebugFormatted("\n%dd %dh %dm %ds \n", ((int(days))), (int(hours) % 24), (int(minutes) % 60),
                      (int(lifeSecondsPowered) % 60));
        LogDebugFormatted("%.0f strokes \n", numberStrokes);
        LogDebugFormatted("%.2f kilometers \n", travelledDistanceKilometers);
        LogDebugFormatted("%.2fA avg current \n", averageCurrent);
        lastLifeUpdateMillis = millis();
    }
    if ((millis() - lastLifeWriteMillis) > 180000)
    {
        // write eeprom every 3 minutes
        writeEepromLifeStats();
        lastLifeWriteMillis = millis();
    }
}

void OSSM::startLeds()
{
    // int power = 250;
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(ossmleds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(100);
    for (int hueShift = 0; hueShift < 350; hueShift++)
    {
        int gHue = hueShift % 255;
        fill_rainbow(ossmleds, NUM_LEDS, gHue, 25);
        FastLED.show();
        delay(4);
    }
}

void OSSM::updateAnalogInputs()
{
    speedPercentage = getAnalogAveragePercent(SPEED_POT_PIN, 50);

    if (modeChanged)
    {
        switch (rightKnobMode)
        {
            case MODE_STROKE:
                setEncoderPercentage(strokePercentage);
                break;
            case MODE_DEPTH:
                setEncoderPercentage(depthPercentage);
                break;
            case MODE_SENSATION:
                setEncoderPercentage(sensationPercentage);
                break;
            case MODE_PATTERN:
                changePattern = 0;
                setEncoderPercentage(50);
                break;
        }

        modeChanged = false;
    }
    else
    {
        switch (rightKnobMode)
        {
            case MODE_STROKE:
                strokePercentage = getEncoderPercentage();
                break;
            case MODE_DEPTH:
                depthPercentage = getEncoderPercentage();
                break;
            case MODE_SENSATION:
                sensationPercentage = getEncoderPercentage();
                break;
            case MODE_PATTERN:
                float patternPercentage = getEncoderPercentage();
                if (patternPercentage >= 52)
                {
                    changePattern = 1;
                }
                else if (patternPercentage <= 48)
                {
                    changePattern = -1;
                }
                else
                {
                    changePattern = 0;
                }
                break;
        }
    }

    immediateCurrent = getCurrentReadingAmps(20);
    averageCurrent = immediateCurrent * 0.02 + averageCurrent * 0.98;
}

float OSSM::getCurrentReadingAmps(int samples)
{
    float currentAnalogPercent = getAnalogAveragePercent(36, samples) - currentSensorOffset;
    float current = currentAnalogPercent * 0.13886f;
    // 0.13886 is a scaling factor determined by real life testing. Convert percent full scale to amps.
    return current;
}
float OSSM::getVoltageReading(int samples) {}

void OSSM::setEncoderPercentage(float percentage)
{
    const int encoderFullScale = 100;
    if (percentage < 0)
    {
        percentage = 0;
    }
    else if (percentage > 100)
    {
        percentage = 100;
    }

    int position = int(encoderFullScale * percentage / 100);
    g_encoder.write(position);
}

float OSSM::getEncoderPercentage()
{
    const int encoderFullScale = 100;
    int position = g_encoder.read();
    float outputPositionPercentage;
    if (position < 0)
    {
        g_encoder.write(0);
        position = 0;
    }
    else if (position > encoderFullScale)
    {
        g_encoder.write(encoderFullScale);
        position = encoderFullScale;
    }

    outputPositionPercentage = 100.0f * float(position) / float(encoderFullScale);

    return outputPositionPercentage;
}

float OSSM::getAnalogAveragePercent(int pinNumber, int samples)
{
    float sum = 0;
    float average;
    float percentage;
    for (int i = 0; i < samples; i++)
    {
        // TODO: Possibly use fancier filters?
        sum += analogRead(pinNumber);
    }
    average = float(sum) / float(samples);
    // TODO: Might want to add a deadband
    percentage = 100.0f * average / 4096.0f; // 12 bit resolution
    return percentage;
}

bool OSSM::waitForAnyButtonPress(float waitMilliseconds)
{
    float timeStartMillis = millis();
    int initialEncoderFlag = encoderButtonPresses;
    LogDebug("Waiting for button press");
    while ((digitalRead(WIFI_RESET_PIN) == LOW) && (initialEncoderFlag == encoderButtonPresses))
    {
        if ((millis() - timeStartMillis) > waitMilliseconds)
        {
            LogDebug("button not pressed");

            return false;
        }
        delay(10);
    }
    LogDebug("button pressed");
    return true;
}
