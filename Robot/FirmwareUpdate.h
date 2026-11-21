#pragma once
#if __has_include("FirmwareIdentity.h") && !ROBOT_LINE_ONLY
#define ROBOT_OTA_CONFIGURED 1
void firmwareInfo(char* output, size_t capacity);
bool firmwareAuthorized(const char* manifest);
void receiveFirmware(int socket, const char* manifest);
#else
#define ROBOT_OTA_CONFIGURED 0
#endif
