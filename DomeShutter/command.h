#ifndef _command_h_
#define _command_h_

extern unsigned long lastCmdTime;

void cmdOpenShutter();
void cmdOpenBoth();
void cmdClose();
void cmdAbort();
void cmdExit();
void cmdStatus();
void cmdGetVBat();
void cmdPing();

#endif
