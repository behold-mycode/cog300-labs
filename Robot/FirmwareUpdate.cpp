#include "Robot.h"
#include "FirmwareUpdate.h"
#if defined(ARDUINO_UNOR4_WIFI) && ROBOT_ENABLE_WIFI && ROBOT_OTA_CONFIGURED
#include "FirmwareIdentity.h"
#include <Ed25519.h>
#include <SHA256.h>
#include <OTAUpdate.h>
#include <WiFiS3.h>
#include <WiFiFileSystem.h>

namespace {
constexpr char path[] = "/robot-update.bin";
constexpr uint32_t maxImage = 0x40000 - 0x4000;
char challenge[33]{};
uint32_t issued;

bool idle() {
  return mode == MANUAL && motorLeft == 0 && motorRight == 0 &&
         !motorDemandActive() && !controlSession && wirelessControlToken() == 0;
}

int nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

bool decode(const char* hex, uint8_t* bytes, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    int hi = nibble(*hex++), lo = nibble(*hex++);
    if (hi < 0 || lo < 0) return false;
    bytes[i] = (hi << 4) | lo;
  }
  return true;
}

uint32_t imageSize(const char* manifest) {
  char size[9]{};
  memcpy(size, manifest, 8);
  return strtoul(size, nullptr, 16);
}

bool stage(WiFiClient& client, uint32_t size) {
  WiFiFileSystem fs;
  uint8_t block[240]; // ESP bridge file writes use an 8-bit length.
  uint32_t offset = 0, started = millis(), last = started;
  while (offset < size && millis() - started < 240000 && millis() - last < 10000) {
    size_t wanted = min(uint32_t(sizeof(block)), size - offset);
    int n = client.read(block, wanted);
    if (n <= 0) { delay(2); continue; }
    if (fs.writefile(path, (const char*)block, n, offset ? WIFI_FILE_APPEND : WIFI_FILE_WRITE) != size_t(n)) return false;
    offset += n;
    last = millis();
  }
  return offset == size;
}

bool verifyStored(uint32_t size, const char* hex) {
  SHA256 hash;
  for (uint32_t offset = 0; offset < size;) {
    uint32_t wanted = min(uint32_t(512), size - offset);
    // On the last read, also check for an unexpected trailing byte.
    std::string data;
    if (!modem.write("+FS:", data, "AT+FS=0,%d,%s,%lu,%lu\r\n", WIFI_FILE_READ,
                     path, (unsigned long)offset, (unsigned long)(wanted + (offset + wanted == size))) ||
        data.size() != wanted) return false;
    hash.update(data.data(), data.size());
    offset += wanted;
  }
  uint8_t actual[32], expected[32];
  hash.finalize(actual, sizeof(actual));
  return decode(hex, expected, sizeof(expected)) && !memcmp(expected, actual, sizeof(actual));
}
}

void firmwareInfo(char* output, size_t capacity) {
  if (idle() && (!challenge[0] || millis() - issued >= 60000)) {
    for (int i = 0; i < 4; ++i) {
      long word = random(0x7fffffff);
      if (word < 0) { challenge[0] = 0; break; }
      snprintf(challenge + 8*i, 9, "%08lx", (unsigned long)word);
    }
    issued = millis();
  }
  snprintf(output, capacity, "{\"build\":\"%s\",\"challenge\":\"%s\"}", ROBOT_BUILD_ID, idle() ? challenge : "");
}

bool firmwareAuthorized(const char* manifest) {
  if (!idle() || !challenge[0] || millis() - issued >= 60000) return false;
  uint8_t signature[64], fields[36];
  if (!decode(manifest, fields, sizeof(fields)) || !decode(manifest + 72, signature, sizeof(signature)) ||
      imageSize(manifest) < 256 || imageSize(manifest) > maxImage) return false;
  char message[104];
  memcpy(message, challenge, 32);
  memcpy(message + 32, manifest, 72);
  if (!Ed25519::verify(signature, ROBOT_UPDATE_KEY, message, sizeof(message))) return false;
  challenge[0] = 0;
  return true;
}

void receiveFirmware(int socket, const char* manifest) {
  // Exclusive use of the modem until completion. No control loop can start motors here.
  stopRobot();
  WiFiClient client(socket);
  std::string response;
  if (!modem.write("+MOUNTFS:", response, "AT+MOUNTFS=0\r\n")) {
    client.print("HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\nContent-Length: 0\r\n\r\n");
    return;
  }
  client.print("HTTP/1.1 101 Switching Protocols\r\nConnection: Upgrade\r\nUpgrade: robot-firmware/1\r\n\r\n");
  uint32_t size = imageSize(manifest);
  if (!stage(client, size) || !verifyStored(size, manifest + 8)) {
    client.print("STAGING_FAILED\n");
    return;
  }
  client.print("FLASHING\n");
  OTAUpdate ota;
  ota.update(path); // The bridge resets this processor on success.
}
#endif
