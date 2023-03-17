#ifndef SLVCTRLPLUS_H
#define SLVCTRLPLUS_H

#include <SerialCommands.h>
#include "Utilities.h"

class SlvCtrlPlus
{
   public:
    char serial_command_buffer[32];
    SerialCommands serialCommands;
    static OSSM* ossm;

    SlvCtrlPlus(OSSM* ossm);
    void comm_setup();
    void comm_loop();
    void static serial_printf(Stream *serial, const char* format, ...);

    void static commandUnrecognized(SerialCommands* sender, const char* cmd);
    void static commandIntroduce(SerialCommands* sender);
    void static commandStatus(SerialCommands* sender);
};

#endif
