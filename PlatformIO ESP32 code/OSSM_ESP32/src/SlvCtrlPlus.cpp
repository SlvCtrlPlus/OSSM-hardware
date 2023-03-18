#include "SlvCtrlPlus.h"
#include <Arduino.h>

const char* DEVICE_TYPE = "ossm";
const int FW_VERSION = 10000; // 1.00.00
const int PROTOCOL_VERSION = 10000;
OSSM* SlvCtrlPlus::ossm = NULL;

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

void SlvCtrlPlus::comm_setup()
{
    // Add commands
    serialCommands.SetDefaultHandler(SlvCtrlPlus::commandUnrecognized);
    serialCommands.AddCommand(new SerialCommand("introduce", &SlvCtrlPlus::commandIntroduce));
    serialCommands.AddCommand(new SerialCommand("attributes", &SlvCtrlPlus::commandAttributes));
    serialCommands.AddCommand(new SerialCommand("status", &SlvCtrlPlus::commandStatus));

    serialCommands.GetSerial()->write(0x07);
}

void SlvCtrlPlus::comm_loop()
{
    serialCommands.ReadSerial();
}

void SlvCtrlPlus::commandIntroduce(SerialCommands* sender)
{
    serial_printf(sender->GetSerial(), "introduce;%s,%d,%d\n", DEVICE_TYPE, FW_VERSION, PROTOCOL_VERSION);
}

void SlvCtrlPlus::commandAttributes(SerialCommands* sender) {
    serial_printf(sender->GetSerial(), "attributes;speed:ro[int],depth:ro[int],stroke:ro[int],sensation:ro[int]\n");
}

void SlvCtrlPlus::commandStatus(SerialCommands* sender) {
    serial_printf(
        sender->GetSerial(),
        "status;speed:%f,depth:%f,stroke:%f,sensation:%f\n",
        SlvCtrlPlus::ossm->speedPercentage,
        SlvCtrlPlus::ossm->depthPercentage,
        SlvCtrlPlus::ossm->strokePercentage,
        SlvCtrlPlus::ossm->sensationPercentage
    );
}
