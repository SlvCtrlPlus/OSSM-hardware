#include "SlvCtrlPlus.h"
#include "pattern.h"
#include <Arduino.h>

const char* DEVICE_TYPE = "ossm";
const int FW_VERSION = 10000; // 1.00.00
const int PROTOCOL_VERSION = 10000;
OSSM* SlvCtrlPlus::ossm = NULL;
bool SlvCtrlPlus::analogInput = true;

SlvCtrlPlus::SlvCtrlPlus(OSSM* ossm) : serialCommands(&Serial, serial_command_buffer, sizeof(serial_command_buffer), "\n", " ") {
    SlvCtrlPlus::ossm = ossm;
}

void SlvCtrlPlus::serial_printf(Stream *serial, const char* format, ...) {
  va_list args;
  va_start(args, format);

  int bufferSize = vsnprintf(NULL, 0, format, args);
  bufferSize++; // safe byte for \0

  char buffer[bufferSize];

  vsnprintf(buffer, bufferSize, format, args);

  va_end(args);

  serial->print(buffer);
}

void SlvCtrlPlus::commandUnrecognized(SerialCommands* sender, const char* cmd)
{
    serial_printf(sender->GetSerial(), "Unrecognized command [%s]\n", cmd);
}

void SlvCtrlPlus::setup()
{
    // Who needs this simple mode anyway? There's the "simple stroke" pattern (index: 0) for that
    ossm->activeRunMode = ossm->strokeEngineMode;

    // Add commands
    serialCommands.SetDefaultHandler(commandUnrecognized);
    serialCommands.AddCommand(new SerialCommand("introduce", &commandIntroduce));
    serialCommands.AddCommand(new SerialCommand("attributes", &commandAttributes));
    serialCommands.AddCommand(new SerialCommand("status", &commandStatus));
    serialCommands.AddCommand(new SerialCommand("set-analoginput", &commandSetAnalogInput));
    serialCommands.AddCommand(new SerialCommand("set-speed", &commandSetSpeed));
    serialCommands.AddCommand(new SerialCommand("set-depth", &commandSetDepth));
    serialCommands.AddCommand(new SerialCommand("set-stroke", &commandSetStroke));
    serialCommands.AddCommand(new SerialCommand("set-sensation", &commandSetSensation));
    serialCommands.AddCommand(new SerialCommand("set-pattern", &commandSetPattern));

    serialCommands.GetSerial()->write(0x07);
}

void SlvCtrlPlus::loop()
{
    serialCommands.ReadSerial();
}

void SlvCtrlPlus::commandIntroduce(SerialCommands* sender)
{
    serial_printf(sender->GetSerial(), "introduce;%s,%d,%d\n", DEVICE_TYPE, FW_VERSION, PROTOCOL_VERSION);
}

void SlvCtrlPlus::commandAttributes(SerialCommands* sender) {
    int strokePatternCount = ossm->strokePatternCount;

    String strokePatternValues = "";

    if (strokePatternCount > 0) {
        strokePatternValues += "0";

        for (int i = 1; i < strokePatternCount; i++) {
            strokePatternValues += "|";
            strokePatternValues += String(i);
        }
    }

    serial_printf(
        sender->GetSerial(),
        "attributes;pattern:rw[%s],analoginput:rw[bool],speed:rw[0-100],depth:rw[0-100],stroke:rw[0-100],sensation:rw[0-100]\n",
        strokePatternValues.c_str()
    );
}

void SlvCtrlPlus::commandStatus(SerialCommands* sender) {
    serial_printf(
        sender->GetSerial(),
        "status;pattern:%d,analoginput:%d,speed:%f,depth:%f,stroke:%f,sensation:%f\n",
        ossm->strokePattern,
        analogInput,
        round(ossm->speedPercentage),
        round(ossm->depthPercentage),
        round(ossm->strokePercentage),
        round(ossm->sensationPercentage)
    );
}

void SlvCtrlPlus::commandSetAnalogInput(SerialCommands* sender) {
    char* adcStr = commandGetArg(sender, "analoginput");

    if (NULL == adcStr) return;

    int adc = atoi(adcStr);

    if (adc > 1 || adc < 0) {
        serial_printf(sender->GetSerial(), "set-analoginput;%s;status:failed,reason:adc_out_of_range\n", adcStr);
        return;
    }

    SlvCtrlPlus::analogInput = (1 == adc);

    serial_printf(sender->GetSerial(), "set-analoginput;%s;status:successful\n", adcStr);
}

void SlvCtrlPlus::commandSetSpeed(SerialCommands* sender)
{
    commandSetGeneric<float>(sender, "speed", ossm->speedPercentage, 0, 100);
}

void SlvCtrlPlus::commandSetDepth(SerialCommands* sender)
{
    commandSetGeneric<float>(sender, "depth", ossm->depthPercentage, 0, 100);
}

void SlvCtrlPlus::commandSetStroke(SerialCommands* sender)
{
    commandSetGeneric<float>(sender, "stroke", ossm->strokePercentage, 0, 100);
}

void SlvCtrlPlus::commandSetSensation(SerialCommands* sender)
{
    commandSetGeneric<float>(sender, "sensation", ossm->sensationPercentage, 0, 100);
}

void SlvCtrlPlus::commandSetPattern(SerialCommands* sender)
{
    commandSetGeneric<int>(sender, "pattern", ossm->strokePattern, 0, 100);
}

char* SlvCtrlPlus::commandGetArg(SerialCommands* sender, const char* attr)
{
    char* valueStr = sender->Next();

    if (valueStr == NULL) {
        serial_printf(sender->GetSerial(), "set-%s;;status:failed,reason:%s_param_missing\n", attr, attr);
        return NULL;
    }

    return valueStr;
}

template<typename T>
void SlvCtrlPlus::commandSetGeneric(SerialCommands* sender, const char* attr, T &var, T min, T max)
{
    char* valueStr = commandGetArg(sender, attr);

    if (NULL == valueStr) return;

    int value = atoi(valueStr);

    if (value < min || value > max) {
        serial_printf(sender->GetSerial(), "set-%s;%d;status:failed,reason:%s_out_of_range\n", attr, value, attr);
        return;
    }

    var = value;

    serial_printf(sender->GetSerial(), "set-%s;%d;status:successful\n", attr, value);
}
