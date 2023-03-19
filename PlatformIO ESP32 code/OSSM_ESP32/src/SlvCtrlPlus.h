#ifndef SLVCTRLPLUS_H
#define SLVCTRLPLUS_H

#include <SerialCommands.h>
#include "Utilities.h"

class SlvCtrlPlus
{
   public:
    static bool analogInput;

    SlvCtrlPlus(OSSM* ossm);
    void setup();
    void loop();

   private:
    char serial_command_buffer[32];
    SerialCommands serialCommands;
    static OSSM* ossm;


    static void serial_printf(Stream *serial, const char* format, ...);

    static void commandUnrecognized(SerialCommands* sender, const char* cmd);
    static void commandIntroduce(SerialCommands* sender);
    static void commandAttributes(SerialCommands* sender);
    static void commandStatus(SerialCommands* sender);

    static void commandSetAnalogInput(SerialCommands* sender);

    template<typename T>
    static void commandSetGeneric(SerialCommands* sender, const char* attr, T &var, T min, T max);
    static char* commandGetArg(SerialCommands* sender, const char* attr);
    static void commandSetSpeed(SerialCommands* sender);
    static void commandSetDepth(SerialCommands* sender);
    static void commandSetStroke(SerialCommands* sender);
    static void commandSetSensation(SerialCommands* sender);
    static void commandSetPattern(SerialCommands* sender);
};

#endif
