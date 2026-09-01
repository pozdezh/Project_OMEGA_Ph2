#ifndef OMEGA_CONFIG_H
#define OMEGA_CONFIG_H

#define LED_PIN 13
#define MIC_PIN 8
#define ADC_CHANNEL ADC_CHANNEL_7
#define ADC_UNIT ADC_UNIT_1
#define SAMPLE_RATE 8000
#define CHUNK_SIZE 1000

static const float MIC_SENS = 0.63f;
static const float REF_DB = 94.0f;
static const float WAKEUP_MARGIN_DB = 6.0f;
static const float SUSTAIN_MARGIN_DB = 1.5f;
static const int MAX_SILENCE = 4;
static const int MAX_CHUNKS = 80;
static const int SYNC_MEASUREMENT_CHUNKS = 8;
static const uint32_t AMBIENT_POLL_MS = 20;
static const uint32_t SENTRY_LOOP_DELAY_MS = 130;

static const int MAX_OFFLINE_PACKETS = 100;
static const uint32_t NETWORK_QUEUE_DEPTH = 8;
static const uint32_t BUFFER_LOCK_TIMEOUT_MS = 5000;
// Bumped to ...33 on 2026-08-27 when OfflinePayload gained capture_ms.
// The magic guards the RECORD LAYOUT, not just the file: an old mirror
// read with the new struct would misparse every field it restored, so a
// layout change must always come with a bump.
static const uint32_t MIRROR_MAGIC = 0x4F4D4234UL;

static const uint32_t TIME_VALID_EPOCH = 1700000000UL;
static const uint32_t RTC_STATE_MAGIC = 0x4F4D5253UL;
static const uint32_t WDT_TIMEOUT_SECONDS = 30;
static const uint32_t SERIAL_USB_ENUMERATE_MS = 2000;
static const int AUDIO_INIT_ATTEMPTS = 3;
static const uint32_t AUDIO_INIT_RETRY_MS = 250;
static const uint32_t HALT_BLINK_AUDIO_MS = 100;
static const uint32_t HALT_BLINK_NET_MS = 500;
static const uint32_t HALT_REPRINT_MS = 2000;
static const uint32_t CLOCK_PLAUSIBLE_WINDOW_S = 315360000UL;
// __DATE__/__TIME__ are the BUILDER'S LOCAL time, but buildEpoch() assembles
// them as if they were UTC. Built anywhere east of Greenwich that puts the
// "floor" in the future - +6877 s when this was measured - and a floor ahead
// of real time is not a floor, it is a clock that invents the future. A day
// of slack makes it a true lower bound no matter where it was compiled, and
// costs nothing: its only job is to be far enough past 1970 for certificate
// validity checks to work before the server answers.
static const int32_t BUILD_EPOCH_TZ_SLACK_S = 86400;
static const int MAX_MISSED_ACKS = 5;

static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
static const uint32_t DISCOVERY_TIMEOUT_MS = 2500;
static const uint32_t SYNC_RETRY_INTERVAL_MS = 300000;
// 5 min, not 30. This is the value a unit uses when it has not yet been
// told otherwise - which is precisely during a reboot it is trying to
// recover from - and the recovery rungs are DERIVED from it. At 30 min the
// derived reboot rung was 105 minutes, so a freshly rebooted NMU that still
// could not reach its server sat silent for nearly two hours. The default
// must therefore be the SAFE-FAST value, not the quiet-site value.
static const uint32_t DEFAULT_HEARTBEAT_INTERVAL_MS = 300000;
static const uint32_t AGGRESSIVE_RECONNECT_WINDOW_MS = 300000;
static const uint32_t RECONNECT_JITTER_MIN_MS = 100;
static const uint32_t RECONNECT_JITTER_MAX_MS = 2000;
static const uint32_t BOOT_JITTER_MAX_MS = 30000;

// Self-recovery ladder. See ARCHITECTURE.md 25. The _MS values are floors;
// each rung is also held clear of this unit's own heartbeat by the _BEATS_X10
// multipliers (tenths), because silence shorter than a heartbeat is not
// evidence of a fault - it is what a quiet room looks like.
static const uint32_t OUTAGE_RADIO_RESET_MS = 300000;
static const uint32_t OUTAGE_REBOOT_MS = 900000;
static const uint32_t OUTAGE_RADIO_RESET_BEATS_X10 = 15;

// How soon a buffered record is retried on its OWN clock, rather than waiting
// for the next noise event or heartbeat to carry it out. Starts at the floor
// so a brief glitch costs seconds; doubles while the link stays down so a
// dead server is not hammered by the whole fleet; snaps back on any ACK.
// Getting the time BEFORE taking the first sample, rather than learning it
// from the first sample's own reply. A record captured before the clock is
// known has to be stamped 0 and dated by inference; one captured after it is
// known carries its own true time. So the moment a session comes up, the unit
// spends one tiny authenticated record purely to ask what time it is.
//
// The time deliberately does NOT come from discovery, which would be earlier
// and cheaper: discovery is unauthenticated, so anything it says is a claim by
// an unverified stranger. A skewed clock accepted there would silently
// mis-date every record the unit ever writes. Time is only ever taken from
// inside the authenticated DTLS session.
static const uint32_t CLOCK_PROBE_INTERVAL_MS = 5000;

// ...but never wait forever. If the server will not supply a time (its own
// clock is below the sanity floor), sampling must still start. Those records
// are stamped 0 and dated honestly by the server on arrival.
static const uint32_t SYNC_CLOCK_WAIT_MS = 30000;

static const uint32_t BACKLOG_RETRY_MIN_MS = 5000;
static const uint32_t BACKLOG_RETRY_MAX_MS = 60000;
static const uint32_t OUTAGE_REBOOT_BEATS_X10 = 35;

#define CPU_MHZ_IDLE 80
#define CPU_MHZ_CAPTURE 160
#define CPU_MHZ_CRYPTO 240
#define CPU_FREQ_SCALING_ENABLED 1
#define CPU_MHZ_PINNED CPU_MHZ_CRYPTO

#define RADIO_SLEEP_ENABLED 1

static const uint32_t NET_TASK_STACK_BYTES = 32768;
static const uint32_t SENTRY_TASK_STACK_BYTES = 8192;
static const int NET_TASK_PRIORITY = 1;
static const int SENTRY_TASK_PRIORITY = 2;
static const int NET_TASK_CORE = 0;
static const int SENTRY_TASK_CORE = 1;

#endif  // OMEGA_CONFIG_H
