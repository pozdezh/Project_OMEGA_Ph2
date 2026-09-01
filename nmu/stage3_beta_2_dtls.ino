#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <Preferences.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>

#include "config.h"
#include "omega_config.h"
#include "omega_audio.h"
#include "omega_buffer.h"
#include "omega_net.h"
#include "omega_tasks.h"

static int64_t daysFromCivilUTC(int year, int month, int day) {
  // Timezone-independent civil-date -> days-since-epoch. mktime() assumes
  // its input is LOCAL time to the calling process; the build MACHINE's
  // local clock (whatever timezone the developer happens to be in) is not
  // the device's clock, so mktime() silently produced a build epoch off by
  // the build machine's UTC offset (confirmed: exactly +2h on a UTC+2
  // build host). This formula (Howard Hinnant's days_from_civil) has no
  // notion of timezone at all - same result on any build machine.
  year -= month <= 2;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

static time_t buildEpoch() {
  static const char MONTHS[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char month_name[4] = {0};
  int day = 1, year = 1970, hour = 0, minute = 0, second = 0;
  if (sscanf(__DATE__, "%3s %d %d", month_name, &day, &year) != 3) {
    return 0;
  }
  if (sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second) != 3) {
    return 0;
  }
  const char* found = strstr(MONTHS, month_name);
  if (found == nullptr) {
    return 0;
  }
  const int month = (int)((found - MONTHS) / 3) + 1;
  const int64_t days = daysFromCivilUTC(year, month, day);
  return (time_t)(days * 86400 + hour * 3600 + minute * 60 + second);
}

static void applyBuildTimeClockFloor() {
  // A clock the server set and that survived the restart is the best time
  // this unit will ever have. The floor is a guess; it must never overwrite
  // a fact. Measured on NMU_22 2026-08-27: without this the floor pushed a
  // correct 04:19:26 forward to 06:14:03 on every self-heal reboot.
  if (netClockSynced()) {
    return;
  }
  const time_t floor_epoch = buildEpoch() - BUILD_EPOCH_TZ_SLACK_S;
  if (floor_epoch <= 0) {
    return;
  }
  time_t now = 0;
  time(&now);
  if (now >= floor_epoch && now <= floor_epoch + CLOCK_PLAUSIBLE_WINDOW_S) {
    return;
  }
  struct timeval tv = {.tv_sec = floor_epoch, .tv_usec = 0};
  settimeofday(&tv, nullptr);
}

static void haltBlinking(uint32_t period_ms, const char* reason) {
  uint32_t last_print = 0;
  uint32_t last_retry = millis();
  while (true) {
    if (millis() - last_print >= HALT_REPRINT_MS) {
      time_t now = 0;
      time(&now);
      Serial.printf("FATAL(halted): %s | clock=%lu heap=%u reset=%d\n",
                    reason, (unsigned long)now, (unsigned)ESP.getFreeHeap(),
                    (int)esp_reset_reason());
      Serial.flush();
      last_print = millis();
    }
    digitalWrite(LED_PIN, HIGH);
    delay(period_ms);
    digitalWrite(LED_PIN, LOW);
    delay(period_ms);
    if (millis() - last_retry >= 30000UL) {
      Serial.println("FATAL: retrying boot automatically");
      Serial.flush();
      ESP.restart();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(SERIAL_USB_ENUMERATE_MS);

  Serial.printf("\nBOOT: reset_reason=%d (1=power-on 4=panic 6=task-wdt 9=brownout 11=usb)\n",
                (int)esp_reset_reason());
  Serial.printf("BOOT: heap=%u largest_dma_block=%u\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));

  {
    Preferences post;
    if (post.begin("omega_pm", false)) {
      Serial.printf("POSTMORTEM: previous boot ended reason=%d after %us\n",
                    post.getInt("last_reset", -1),
                    (unsigned)post.getUInt("last_uptime", 0));
      post.putInt("last_reset", (int)esp_reset_reason());
      post.end();
    }
  }
  Serial.flush();

  // Before anything network-shaped starts: the recovery rungs are derived
  // from the heartbeat, so a unit rebooting into an outage must recover the
  // value the server taught it rather than fall back to the compiled default.
  netLoadPersistedHeartbeat();
  // Before the floor is applied: the floor would overwrite an absent
  // clock with a plausible-looking value and hide the power cut.
  netValidateRestoredClock();

  {
    time_t before = 0;
    time(&before);
    Serial.printf("BOOT: clock_before_floor=%lu\n", (unsigned long)before);
  }
  applyBuildTimeClockFloor();
  {
    time_t now = 0;
    time(&now);
    Serial.printf("BOOT: clock=%lu (build floor applied)\n", (unsigned long)now);
  }

  analogReadResolution(12);
  btStop();

  setCpuFrequencyMhz(CPU_MHZ_PINNED);

  bool audio_ready = false;
  for (int attempt = 1; attempt <= AUDIO_INIT_ATTEMPTS && !audio_ready; attempt++) {
    audio_ready = audioInit();
    if (!audio_ready) {
      Serial.printf("BOOT: audioInit attempt %d/%d failed\n", attempt, AUDIO_INIT_ATTEMPTS);
      Serial.flush();
      delay(AUDIO_INIT_RETRY_MS);
    }
  }
  if (!audio_ready) {
    haltBlinking(HALT_BLINK_AUDIO_MS,
                 "audioInit failed (ADC calibration or DMA buffer)");
  }
  Serial.println("BOOT: audio OK");

  int rc = netInit();
  if (rc != 0) {
    static char net_reason[96];
    snprintf(net_reason, sizeof(net_reason),
             "dtls.begin failed (rc=%d) - check omega_certs.h", rc);
    haltBlinking(HALT_BLINK_NET_MS, net_reason);
  }
  Serial.printf("Config OK: id=%s server=%s:%d\n", deviceID.c_str(),
                OMEGA_SERVER_IP, OMEGA_SERVER_PORT);

  bufferInit();
  tasksInit();

  {
    const uint32_t boot_jitter_ms = esp_random() % BOOT_JITTER_MAX_MS;
    Serial.printf("BOOT: spreading first handshake by %u ms\n",
                  (unsigned)boot_jitter_ms);
    Serial.flush();
    const uint32_t started = millis();
    while (millis() - started < boot_jitter_ms) {
      delay(100);
      esp_task_wdt_reset();
    }
  }

  for (int i = 0; i < 3; i++) {
    if (ensureSession()) {
      break;
    }
    killWiFi();
    delay(1000);
  }
  Serial.printf("Boot session: serverFound=%d\n", serverFound);

  tasksMarkBootComplete();

  esp_task_wdt_config_t wdt_config = {.timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
                                      .idle_core_mask = (1 << 0) | (1 << 1),
                                      .trigger_panic = true};
  esp_err_t wdt_init_rc = esp_task_wdt_init(&wdt_config);
  esp_err_t wdt_recfg_rc = esp_task_wdt_reconfigure(&wdt_config);
  Serial.printf("WDT: init_rc=%d reconfigure_rc=%d (0 = applied)\n",
                (int)wdt_init_rc, (int)wdt_recfg_rc);

  xTaskCreatePinnedToCore(NetworkTask, "NetTask", NET_TASK_STACK_BYTES, NULL,
                          NET_TASK_PRIORITY, &netTaskHandle, NET_TASK_CORE);
  xTaskCreatePinnedToCore(SentryTask, "SentryTask", SENTRY_TASK_STACK_BYTES, NULL,
                          SENTRY_TASK_PRIORITY, NULL, SENTRY_TASK_CORE);
  vTaskDelete(NULL);
}

void loop() {}
