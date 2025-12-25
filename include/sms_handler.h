#ifndef SMS_HANDLER_H
#define SMS_HANDLER_H

#include <Arduino.h>
#include "types.h"

// ================== SMS Cooldown Array ==================
extern unsigned long lastSMSSentTime[7];

// ================== SMS Functions ==================
bool checkSMSCooldown(SMSAlertType type);
void loadSMSCooldown();
bool enqueueSMS(const char *msg, SMSAlertType type);
bool sendSMS(const char *msg);
String readUnreadSMS();
void processIncomingSMS(const String &resp);
#endif // SMS_HANDLER_H
