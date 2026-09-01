#include "omega_buffer.h"
#include "omega_config.h"

#include <FS.h>
#include <SPIFFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static RTC_DATA_ATTR OfflinePayload ringBuffer[MAX_OFFLINE_PACKETS];
static RTC_DATA_ATTR int ringHead = 0;
static RTC_DATA_ATTR int ringTail = 0;
static RTC_DATA_ATTR int ringCount = 0;

static SemaphoreHandle_t ringMutex = NULL;
static bool mirrorMounted = false;
static uint32_t mirrorFailures = 0;

static const char* MIRROR_PATH = "/buffer.bin";
static const char* MIRROR_TMP_PATH = "/buffer.bin.tmp";

struct __attribute__((packed)) MirrorHeader {
  uint32_t magic;
  int32_t head;
  int32_t tail;
  int32_t count;
};

static bool ringLock() {
  if (ringMutex == NULL) {
    return true;
  }
  return xSemaphoreTake(ringMutex, pdMS_TO_TICKS(BUFFER_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static void ringUnlock() {
  if (ringMutex != NULL) {
    xSemaphoreGive(ringMutex);
  }
}

static void mirrorWrite() {
  if (!mirrorMounted) {
    return;
  }
  File tmp = SPIFFS.open(MIRROR_TMP_PATH, FILE_WRITE);
  if (!tmp) {
    mirrorFailures++;
    return;
  }
  MirrorHeader header = {MIRROR_MAGIC, (int32_t)ringHead, (int32_t)ringTail,
                         (int32_t)ringCount};
  const size_t expected = sizeof(header) + (size_t)ringCount * sizeof(OfflinePayload);
  size_t written = tmp.write((const uint8_t*)&header, sizeof(header));
  int slot = ringTail;
  for (int i = 0; i < ringCount; i++) {
    written += tmp.write((const uint8_t*)&ringBuffer[slot], sizeof(OfflinePayload));
    slot = (slot + 1) % MAX_OFFLINE_PACKETS;
  }
  tmp.close();
  if (written != expected) {
    mirrorFailures++;
    return;
  }
  SPIFFS.remove(MIRROR_PATH);
  if (!SPIFFS.rename(MIRROR_TMP_PATH, MIRROR_PATH)) {
    mirrorFailures++;
  }
}

static int mirrorFileSize(const char* path) {
  if (!mirrorMounted || !SPIFFS.exists(path)) {
    return 0;
  }
  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    return 0;
  }
  const int size = (int)file.size();
  file.close();
  return size;
}

static int mirrorRestoreFrom(const char* path) {
  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    return -1;
  }
  MirrorHeader header;
  if (file.read((uint8_t*)&header, sizeof(header)) != sizeof(header)) {
    file.close();
    return -1;
  }
  if (header.magic != MIRROR_MAGIC) {
    file.close();
    return -1;
  }
  if (header.count <= 0 || header.count > MAX_OFFLINE_PACKETS ||
      header.tail < 0 || header.tail >= MAX_OFFLINE_PACKETS ||
      header.head < 0 || header.head >= MAX_OFFLINE_PACKETS) {
    file.close();
    return -1;
  }

  int restored = 0;
  int slot = header.tail;
  for (int i = 0; i < header.count; i++) {
    OfflinePayload payload;
    if (file.read((uint8_t*)&payload, sizeof(payload)) != sizeof(payload)) {
      break;
    }
    ringBuffer[slot] = payload;
    restored++;
    slot = (slot + 1) % MAX_OFFLINE_PACKETS;
  }
  file.close();

  ringTail = header.tail;
  ringHead = (header.tail + restored) % MAX_OFFLINE_PACKETS;
  ringCount = restored;
  return restored;
}

void bufferInit() {
  if (ringMutex == NULL) {
    ringMutex = xSemaphoreCreateMutex();
  }
  bool mounted = SPIFFS.begin(false);
  if (!mounted) {
    Serial.println("BUFFER: SPIFFS mount FAILED - formatting. Any backlog "
                   "held in flash is lost.");
    mounted = SPIFFS.begin(true);
  }
  if (!mounted) {
    Serial.println("BUFFER: SPIFFS unusable - outage backlog will not "
                   "survive a restart this boot");
    return;
  }
  mirrorMounted = true;

  int restored = 0;
  if (SPIFFS.exists(MIRROR_PATH)) {
    restored = mirrorRestoreFrom(MIRROR_PATH);
  } else if (SPIFFS.exists(MIRROR_TMP_PATH)) {
    restored = mirrorRestoreFrom(MIRROR_TMP_PATH);
    if (restored > 0) {
      Serial.println("BUFFER: live mirror missing, recovered the backlog from "
                     "the temp mirror");
      mirrorWrite();
    }
  }
  Serial.printf("BUFFER: mount=1 total=%u used=%u mirror=%d tmp=%d bytes=%d restored=%d\n",
                (unsigned)SPIFFS.totalBytes(), (unsigned)SPIFFS.usedBytes(),
                (int)SPIFFS.exists(MIRROR_PATH), (int)SPIFFS.exists(MIRROR_TMP_PATH),
                mirrorFileSize(MIRROR_PATH), restored);
}

uint32_t bufferMirrorFailures() {
  return mirrorFailures;
}

void bufferSave(const OfflinePayload& payload) {
  if (payload.isHeartbeat) {
    return;
  }
  if (!ringLock()) {
    Serial.println("BUFFER: lock timeout on save - event NOT buffered");
    return;
  }
  const bool outageStarting = (ringCount == 0);
  ringBuffer[ringHead] = payload;
  ringHead = (ringHead + 1) % MAX_OFFLINE_PACKETS;
  if (ringCount < MAX_OFFLINE_PACKETS) {
    ringCount++;
  } else {
    ringTail = (ringTail + 1) % MAX_OFFLINE_PACKETS;
  }
  mirrorWrite();
  if (outageStarting) {
    Serial.printf("BUFFER: outage started, mirror now %d byte(s), fails=%u\n",
                  mirrorFileSize(MIRROR_PATH), (unsigned)mirrorFailures);
  }
  ringUnlock();
}

int bufferCount() {
  return ringCount;
}

int bufferPeekOldest(OfflinePayload& out) {
  if (!ringLock()) {
    return BUFFER_NO_SLOT;
  }
  int slot = BUFFER_NO_SLOT;
  if (ringCount > 0) {
    slot = ringTail;
    out = ringBuffer[ringTail];
  }
  ringUnlock();
  return slot;
}

void bufferPopOldest(int slot_token) {
  if (slot_token == BUFFER_NO_SLOT) {
    return;
  }
  if (!ringLock()) {
    return;
  }
  if (ringCount > 0 && ringTail == slot_token) {
    ringTail = (ringTail + 1) % MAX_OFFLINE_PACKETS;
    ringCount--;
  }
  ringUnlock();
}

void bufferSyncMirror() {
  if (!ringLock()) {
    return;
  }
  if (ringCount == 0) {
    if (mirrorMounted) {
      SPIFFS.remove(MIRROR_PATH);
      SPIFFS.remove(MIRROR_TMP_PATH);
    }
  } else {
    mirrorWrite();
  }
  ringUnlock();
}
