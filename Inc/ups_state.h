/**
 * @file ups_state.h
 * @brief UPSPlus State Machine and Data Structure Definitions
 * 
 * This header defines the architectural foundation for the UPSPlus firmware:
 * - State machine enumerations (power, charger, learning, button)
 * - Authoritative state structure (single source of truth)
 * - Snapshot buffer mechanism for I2C coherence
 * - Flash persistence structure
 * - Scheduler and timing structures
 * 
 * Core Principles:
 * 1. Single Source of Truth: One authoritative state structure
 * 2. Explicit State Machine: All behavior driven by documented state transitions
 * 3. Canonical Scheduler: TIM1 10ms interrupt serves as single timebase
 * 4. ISR Safety: No flash writes in ISRs; minimal work in ISRs
 * 5. Data Coherence: Multi-byte registers read from single atomic snapshot
 *
 * ---------------------------------------------------------------------------
 * Characterization Results (single reference for verification claims)
 * ---------------------------------------------------------------------------
 * The following were confirmed during characterization (user verification
 * on legacy firmware / hardware). Inline "VERIFIED" or "Confirmed by
 * characterization" below refer to this block. If disputed later, re-run
 * characterization (e.g., test script reading reg 0x14, reserved region,
 * ADC scaling, temperature units) and update this section.
 *
 * - VBUS/USBIN ADC scaling: __LL_ADC_CALC_DATA_TO_VOLTAGE produces connector-
 *   referenced millivolts; charger thresholds (CHARGER_PRESENT_ON/OFF_MV) are correct.
 * - Reserved region reads (0x2E-0xEF, 0xFC-0xFF): legacy returns 0x00; writes ACK'd but discarded.
 * - Register 0x14 (battery percent MSB): legacy always returns 0x00.
 * - Temperature (0x0B-0x0C): integer degrees Celsius (°C); __LL_ADC_CALC_TEMPERATURE yields °C.
 * - ADC channel ordering: matches MX_ADC_Init sequence (see ADC section).
 */

#ifndef UPS_STATE_H
#define UPS_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>  /* For offsetof() used in CRC offset macros */
/* Note: Boolean values use uint8_t (0/1) for embedded efficiency, not stdbool.h bool type */

/* C/C++ compatibility for static assertions */
#ifndef __cplusplus
#define STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#define STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#endif

/*===========================================================================*/
/*                              CONSTANTS                                     */
/*===========================================================================*/

/* Default Values */
#define DEFAULT_VBAT_FULL_MV          4200    /* Default full battery voltage (mV) */
#define DEFAULT_VBAT_EMPTY_MV         3000    /* Default empty battery voltage (mV) */
#define DEFAULT_VBAT_PROTECT_MV       2800    /* Default protection voltage (mV) */
#define DEFAULT_LOW_BATTERY_PERCENT   10      /* Default low battery percentage */
#define DEFAULT_LOAD_ON_DELAY_SEC     60      /* Default load on delay (seconds) */
#define DEFAULT_SAMPLE_PERIOD_MIN     2       /* Default sample period (minutes) */
#define DEFAULT_AUTO_POWER_ON         0       /* Default auto power on disabled */

/* Voltage Limits */
/* Note: VBAT_MIN_VALID_MV applied in ADC_ProcessSample() before updating authoritative_state.battery_voltage_mv */
#define VBAT_MIN_VALID_MV             1000    /* Minimum valid battery voltage reading (ADC sanity filter) */
#define VBAT_PROTECT_MIN_MV           2800    /* Minimum allowed protection voltage */
#define VBAT_PROTECT_MAX_MV           4500    /* Maximum allowed protection voltage */
#define VBAT_FULL_MAX_MV              4500    /* Maximum allowed full voltage */
#define VBAT_EMPTY_MAX_MV             4500    /* Maximum allowed empty voltage */

/* Charger Detection Thresholds */
/* 
 * IMPORTANT: These thresholds are in POST-SCALED, CONNECTOR-REFERENCED millivolts
 * as produced by the existing ADC conversion macros (same domain as uVBUSVolt/uUSBINVolt).
 * They are NOT raw ADC counts or pre-scaled values.
 * 
 * Confirmed by characterization: ADC scaling as above (see block at top of file).
 * 
 * Detection algorithm:
 * 1. Use MAX(usbc_voltage_mv, microusb_voltage_mv) as charger voltage
 * 2. Stability counting MUST be gated by adc_sample_seq with per-consumer last_seen_seq
 * 3. Hysteresis: present_on (4200mV) for detection, present_off (3800mV) for removal
 * 4. Require CHARGER_STABILITY_SAMPLES consecutive qualified samples before state change
 */
#define CHARGER_PRESENT_ON_MV         4200    /* Threshold for charger detected (post-scaled mV); verified */
#define CHARGER_PRESENT_OFF_MV        3800    /* Threshold for charger removed (post-scaled mV); verified */
/* Note: CHARGER_STABILITY_SAMPLES counts ADC samples (adc_sample_seq), not 10ms ticks */
#define CHARGER_STABILITY_SAMPLES     3       /* Consecutive ADC samples for charger state change */

/* Charger Voltage Helper - Locks MAX(VBUS, USBIN) rule in code.
 * Used for charger presence detection, not battery charge current. */
/* Note: Function definition moved after authoritative_state_t definition (see below) */

/* Protection Logic */
#define PROTECTION_HYSTERESIS_MV      50      /* Hysteresis for protection recovery (mV) */
/* Note: PROTECTION_SAMPLES_REQUIRED counts ADC samples (adc_sample_seq), not 10ms ticks */
#define PROTECTION_SAMPLES_REQUIRED   3       /* Consecutive ADC samples below threshold */
/* Protection cut timeout: force MT_EN cut if flash save stalls */
#define PROTECTION_FORCE_CUT_TIMEOUT_SEC   30u
#define PROTECTION_FORCE_CUT_TIMEOUT_TICKS ((uint32_t)(PROTECTION_FORCE_CUT_TIMEOUT_SEC * TICKS_PER_1S))

/* Battery Percentage */
#define MIN_VOLTAGE_DELTA_MV          50      /* Minimum voltage range for valid % calculation */
#define BATTERY_PERCENT_HYSTERESIS    1       /* Minimum change to update percentage */

/* Timing Constants (in 10ms ticks unless otherwise noted)
 * Canonical scheduler: TIM1 tick period is fixed at 10ms (see Core Principles). Do not change
 * TIM1 period without updating TICK_PERIOD_MS and all derived constants. All other "10ms tick"
 * comments in this file refer to this definition. */
#define TICK_PERIOD_MS                10      /* TIM1 tick period in milliseconds */
/* Derived tick constants - self-consistent with TICK_PERIOD_MS */
#define TICKS_PER_100MS               (100 / TICK_PERIOD_MS)      /* Ticks per 100ms */
#define TICKS_PER_500MS               (500 / TICK_PERIOD_MS)      /* Ticks per 500ms (ADC trigger) */
#define TICKS_PER_1S                  (1000 / TICK_PERIOD_MS)    /* Ticks per second */
#define MEASUREMENT_WINDOW_TICKS      (1500 / TICK_PERIOD_MS)    /* 1.5 seconds measurement window */
#define BUTTON_DEBOUNCE_TICKS         (50 / TICK_PERIOD_MS)      /* 50ms debounce */
/* Note: >= BUTTON_LONG_PRESS_TICKS (exactly 10s) counts as long press per plan boundary */
#define BUTTON_LONG_PRESS_TICKS       (10000 / TICK_PERIOD_MS)   /* 10 seconds for long press */

/* Boot brownout backoff */
#define BOOT_BROWNOUT_WINDOW_MINUTES  5u
#define BOOT_BROWNOUT_WINDOW_TICKS    (BOOT_BROWNOUT_WINDOW_MINUTES * 60u * TICKS_PER_1S)
#define BOOT_BACKOFF_INCREMENT_SEC    60u
#define BOOT_BACKOFF_MAX_SEC          3600u

/* Compile-time guards: TICK_PERIOD_MS must divide 100/500/1000 cleanly for derived tick constants */
STATIC_ASSERT((100 % TICK_PERIOD_MS) == 0, "TICK_PERIOD_MS must divide 100");
STATIC_ASSERT((500 % TICK_PERIOD_MS) == 0, "TICK_PERIOD_MS must divide 500");
STATIC_ASSERT((1000 % TICK_PERIOD_MS) == 0, "TICK_PERIOD_MS must divide 1000");

/* Snapshot and Scheduling */
/* Note: Snapshot updates use tick_counter deltas (canonical timing), not convenience counters */

/* Flash Persistence */
/* Magic number: "UPSP" - endian-safe constant construction */
/* Note: Flash storage is little-endian on STM32; this constant ensures correct
 * byte order when comparing against flash-stored values. */
#define FLASH_MAGIC_NUMBER            ((uint32_t)('U') | ((uint32_t)('P')<<8) | ((uint32_t)('S')<<16) | ((uint32_t)('P')<<24))
#define FLASH_STRUCTURE_VERSION       1           /* Increment when structure changes */
#define FLASH_WRITE_RATE_LIMIT_SEC    5           /* Minimum seconds between flash writes */
#define FLASH_DIRTY_MAX_INTERVAL_SEC  60          /* Max seconds before forcing a dirty save */
#define FLASH_RETRY_BACKOFF_SEC       2           /* Retry backoff after failed save */
#define FLASH_RETRY_BACKOFF_SEC_BYPASS 1          /* Backoff for bypass saves (protection/OTA) */

/* Flash Page Size - STM32F030F4Px uses 1KB pages */
#define FLASH_PAGE_SIZE               1024

/* True-VBAT Staleness (timing uses canonical 10ms tick - see TICK_PERIOD_MS block) */
#define TRUE_VBAT_MAX_AGE_SEC         600     /* 10 minutes max staleness */
/* Explicit uint32_t for overflow safety; use UINT32_C for literal if needed in expressions */
#define TRUE_VBAT_MAX_AGE_TICKS       ((uint32_t)((uint32_t)(TRUE_VBAT_MAX_AGE_SEC) * (uint32_t)(TICKS_PER_1S)))

/*===========================================================================*/
/*                         STATE MACHINE ENUMERATIONS                         */
/*===========================================================================*/

/**
 * @brief Power State - Controls RPi power (MT_EN pin)
 * 
 * Transitions:
 * - RPI_OFF -> LOAD_ON_DELAY: Battery % > Low Battery % AND AutoPowerOn enabled
 * - LOAD_ON_DELAY -> RPI_ON: Delay elapsed AND conditions still met
 * - LOAD_ON_DELAY -> RPI_OFF: Conditions no longer met
 * - RPI_ON -> PROTECTION_LATCHED: Battery voltage <= Protection Voltage (N samples)
 * - RPI_ON -> RPI_OFF: Button short press (toggle)
 * - PROTECTION_LATCHED -> LOAD_ON_DELAY: Charger connected AND battery % > threshold
 * - PROTECTION_LATCHED -> RPI_OFF: Charger disconnected or conditions not met
 */
typedef enum {
    POWER_STATE_RPI_OFF = 0,           /* RPi power disabled (MT_EN = LOW) */
    POWER_STATE_RPI_ON,                /* RPi power enabled (MT_EN = HIGH) */
    POWER_STATE_PROTECTION_LATCHED,    /* Protection voltage reached, power latched off */
    POWER_STATE_LOAD_ON_DELAY          /* Waiting for delay before enabling power */
} power_state_t;

/**
 * @brief Charger State - Controls charger input (IP_EN pin)
 * 
 * Transitions:
 * - ABSENT -> PRESENT: Charger voltage > present_on threshold (with stability)
 * - PRESENT -> FORCED_OFF_WINDOW: Sample period elapsed
 * - FORCED_OFF_WINDOW -> PRESENT: 1.5s window elapsed
 * - PRESENT -> ABSENT: Charger voltage < present_off threshold (with stability)
 * 
 * Note: FORCED_OFF_WINDOW is the only state where charger may be physically
 * connected (VBUS high) while IP_EN is LOW, enabling true VBAT measurement.
 */
typedef enum {
    CHARGER_STATE_ABSENT = 0,          /* Charger not connected (IP_EN = LOW) */
    CHARGER_STATE_PRESENT,             /* Charger connected and enabled (IP_EN = HIGH) */
    CHARGER_STATE_FORCED_OFF_WINDOW    /* Measurement window active (IP_EN = LOW for 1.5s) */
} charger_state_t;

/**
 * @brief Calibration Window Flag (legacy name: learning_mode_t)
 * 
 * Transitions:
 * - INACTIVE -> ACTIVE: Charger state enters FORCED_OFF_WINDOW
 * - ACTIVE -> INACTIVE: Charger state exits FORCED_OFF_WINDOW
 */
typedef enum {
    LEARNING_INACTIVE = 0,             /* Calibration window inactive */
    LEARNING_ACTIVE                    /* Calibration window active (charger forced off) */
} learning_mode_t;

/**
 * @brief Button State Machine states
 * 
 * Note: Released states (BUTTON_RELEASED_SHORT/LONG) are for debugging only.
 * Action dispatch uses pending_click field in button_handler_t.
 */
typedef enum {
    BUTTON_IDLE = 0,                   /* Button not pressed */
    BUTTON_PRESSED,                    /* Button pressed, debouncing */
    BUTTON_HELD,                       /* Button held after debounce */
    BUTTON_RELEASED_SHORT,             /* Released after short press (debug only) */
    BUTTON_RELEASED_LONG               /* Released after long press (debug only) */
} button_state_t;

/**
 * @brief Button click type for action dispatch
 */
typedef enum {
    BUTTON_CLICK_NONE = 0,             /* No click detected */
    BUTTON_CLICK_SHORT,                /* Short press (< 10 seconds) */
    BUTTON_CLICK_LONG                  /* Long press (>= 10 seconds) */
} button_click_t;

/*===========================================================================*/
/*                         SYSTEM STATE STRUCTURES                            */
/*===========================================================================*/

/**
 * @brief Charger Physical Detection State
 * 
 * Tracks physical charger presence (VBUS/USBIN voltage) separately from
 * charger_state_t (which includes path control). This enables "unplug mid-window
 * completes window" behavior - physical presence is tracked independently.
 * 
 * Detection Algorithm (must be implemented per these rules):
 * 1. Use MAX(usbc_voltage_mv, microusb_voltage_mv) as charger voltage
 * 2. Stability counting MUST gate on adc_sample_seq: only increment charger_stability_count
 *    when adc_sample_seq != charger_last_seen_seq, then update charger_last_seen_seq
 * 3. Apply hysteresis: present_on (4200mV) for detection, present_off (3800mV) for removal
 * 4. Require CHARGER_STABILITY_SAMPLES consecutive qualified samples before state change
 * 5. Reset semantics: If sample does not qualify for the current transition direction,
 *    stability_count resets to 0 (e.g., voltage below threshold when detecting, above when removing)
 * 
 * This is the ONLY module that owns charger stability counting. The charger FSM
 * (charger_state_t) consumes this physical detection result, not the stability counters.
 * Future risk: Charger FSM must consume only charger_physically_present and window
 * manager due/active timing. Do not add another stability counter or raw VBUS/USBIN
 * threshold checks in the charger FSM.
 */
typedef struct {
    /* Diagnostic only: not used for FSM decisions (stability + current charger_mv at window end drive state). */
    uint8_t charger_physically_present;  /* 1 = VBUS/USBIN above threshold, 0 = absent */
    uint8_t charger_stability_count;     /* Consecutive ADC samples (gated by adc_sample_seq) */
    uint32_t charger_last_seen_seq;     /* Last adc_sample_seq for charger detection */
} charger_physical_state_t;

/**
 * @brief Measurement Window Manager State
 * 
 * Manages measurement window scheduling and timing, separate from charger_state_t
 * to maintain orthogonal state dimensions and prevent accidental coupling.
 * 
 * Atomic Window Contract:
 * - Windows are NEVER interruptible, NEVER nested, NEVER restarted early
 * - window_active explicitly indicates active window (avoids tick 0 ambiguity at boot)
 * - window_due MUST be cleared ONLY when transitioning into CHARGER_STATE_FORCED_OFF_WINDOW
 *   (prevents "window storm" - constantly reasserting "due" every 10ms)
 * - If charger unplugged mid-window, window completes full 1.5s timing
 * 
 * Invariant: window_active == 1 iff charger_state == CHARGER_STATE_FORCED_OFF_WINDOW
 * Ownership: Charger FSM owns setting both window_active and charger_state atomically
 * (single function must update both to maintain invariant).
 * Future risk: Enforce the invariant in exactly one place—the single function that
 * transitions into/out of FORCED_OFF_WINDOW. Do not set window_active or charger_state
 * separately elsewhere, or you will drift (e.g. window_active=1 but charger_state=PRESENT).
 */
typedef struct {
    uint8_t window_due;                 /* Set when sample period elapsed, cleared on window entry */
    uint8_t window_active;               /* 1 = window currently active, 0 = inactive (invariant: matches charger_state) */
    uint32_t last_window_end_ticks;    /* When last measurement window ended */
    uint32_t window_start_ticks;        /* When current window started (only valid if window_active == 1) */
} window_manager_state_t;

/**
 * @brief Combined System State Machine Structure
 * 
 * Contains the three orthogonal state dimensions (power, charger, calibration flag)
 * plus their entry timing. Window management and physical charger detection
 * are separate structures to maintain clear separation of concerns.
 * 
 * Calibration Window Flag Contract:
 * - learning_mode is DERIVED-ONLY and must be recomputed from charger_state during state update
 * - learning_mode must NEVER be directly set from I2C writes
 * - Canonical source: charger_state == CHARGER_STATE_FORCED_OFF_WINDOW
 * - learning_mode == LEARNING_ACTIVE iff charger_state == CHARGER_STATE_FORCED_OFF_WINDOW
 */
typedef struct {
    /* State dimensions */
    power_state_t power_state;
    charger_state_t charger_state;
    learning_mode_t learning_mode;     /* Derived from charger_state; never set directly */
    
    /* State timing (in TIM1 ticks). Invariant: set on every transition into that state (including restart start, charger PRESENT/ABSENT, power RPI_ON/RPI_OFF). */
    uint32_t power_state_entry_ticks;
    uint32_t charger_state_entry_ticks;
    
    /* Factory reset flag (temporary, not persistent) */
    uint8_t factory_reset_requested;

    /* Factory testing selector (0 = disabled, runtime-only, not persistent) */
    uint8_t factory_test_selector;
    
    /* Pending power cut (for protection shutdown ordering) */
    /* Deterministic Protection Cut Ordering Invariant:
     * When entering POWER_STATE_PROTECTION_LATCHED:
     * 1. Set pending_power_cut = 1
     * 2. Request flash commit (even if flash disabled, maintain sequencing)
     * 3. Attempt flash commit in main loop
     * 4. ONLY AFTER commit attempt (success or failure), cut MT_EN LOW
     * This prevents cutting power before state is persisted. */
    uint8_t pending_power_cut;
    uint32_t pending_power_cut_start_ticks;

    /* Boot brownout backoff (runtime-only, not persisted) */
    uint8_t boot_backoff_active;
    uint32_t boot_attempt_start_ticks;
} system_state_t;

/**
 * @brief Protection State tracking
 */
typedef struct {
    uint8_t below_threshold_count;     /* Counts ADC updates, not tick_counter */
    uint8_t protection_active;         /* Protection is latched active */
    uint16_t last_adc_battery_mv;      /* Last battery voltage from ADC */
    uint32_t last_seen_seq;            /* Last seen adc_sample_seq */
} protection_state_t;

/**
 * @brief Button Handler State
 */
typedef struct {
    button_state_t state;
    uint32_t press_start_tick;         /* TIM1 tick when press first detected */
    uint16_t hold_ticks;                /* Duration held (increments while button pressed).
                                         * Saturates at >BUTTON_LONG_PRESS_TICKS to avoid re-fire. */
    uint8_t long_press_fired;           /* Flag: 1 = long press action already triggered (one-shot guarantee) */
    uint8_t debounce_counter;          /* For debounce filtering */
    button_click_t pending_click;       /* Click type to process */
} button_handler_t;

/**
 * @brief GPIO State tracking
 */
typedef struct {
    uint8_t ip_en_state;               /* 0=LOW, 1=HIGH */
    uint8_t mt_en_state;               /* 0=LOW, 1=HIGH */
} gpio_state_t;

/*===========================================================================*/
/*                      AUTHORITATIVE STATE STRUCTURE                         */
/*===========================================================================*/

/**
 * @brief Authoritative State - Single Source of Truth
 * 
 * All system state is contained here. Only the main loop may modify this.
 * ISRs set flags; main loop processes flags and updates this structure.
 * Snapshot is derived from this structure for I2C reads.
 * Future risk: Do NOT add power_status (0x17) here. Register 0x17 is derived only
 * in snapshot mapping via GetPowerStatusRegisterValue(); storing it here would create
 * two truths and divergence.
 */
typedef struct {
    /* Voltage measurements (updated from ADC in main loop) */
    uint16_t mcu_voltage_mv;           /* 0x01-0x02: uAVDDVolt */
    uint16_t pogopin_voltage_mv;       /* 0x03-0x04: uPIVCCVolt */
    uint16_t battery_voltage_mv;       /* 0x05-0x06: uVBATVolt */
    uint16_t usbc_voltage_mv;          /* 0x07-0x08: uVBUSVolt */
    uint16_t microusb_voltage_mv;      /* 0x09-0x0A: uUSBINVolt */
    /* Temperature: always integer °C (__LL_ADC_CALC_TEMPERATURE).
     * Contract: value is always degrees Celsius (integer). Field name temperature_raw retained for now. */
    uint16_t temperature_raw;          /* 0x0B-0x0C: Temperature, always °C integer */
    
    /* Battery parameters (RW via I2C) */
    uint16_t full_voltage_mv;          /* 0x0D-0x0E: uVBATMax */
    uint16_t empty_voltage_mv;         /* 0x0F-0x10: uVBATMin */
    uint16_t protection_voltage_mv;    /* 0x11-0x12: uVBATProtect */
    
    /* Battery status */
    /* Semantic type: uint8_t (0-100). Register mapping expands to 16-bit (0x13=LSB, 0x14=MSB).
     * This maintains single source of truth for semantics while matching register width.
     * Writer/compute logic MUST enforce 0-100 range. Snapshot/register mapping emits [percent, 0x00]. */
    uint8_t battery_percent;           /* 0x13-0x14: Published percentage (0-100), expanded to 16-bit in register */
    /* Battery percent staleness: battery_percent is assumed fresh iff last_true_vbat_sample_tick is fresh.
     * Power-on gating uses last_true_vbat_sample_tick exclusively (see IsTrueVbatSampleFresh()). */
    
    /* Configuration (RW via I2C, persisted to flash) */
    uint16_t sample_period_minutes;    /* 0x15-0x16: Sample period (1-1440 min) */
    uint8_t shutdown_countdown_sec;    /* 0x18: Current countdown (0=inactive, 10-255) */
    uint8_t auto_power_on;             /* 0x19: Back-to-AC auto power up (0/1) */
    uint8_t restart_countdown_sec;     /* 0x1A: Current countdown (0=inactive, 10-255) */
    /* Battery Parameters Self-Programmed (0x2A): 0 => self-programming enabled, 1 => disabled */
    uint8_t battery_params_self_programmed; /* 0x2A: 0=enable self-programming, 1=disable (user override) */
    uint8_t low_battery_percent;       /* 0x2B: Low battery threshold (0-100%) */
    /* Load On Delay Register Readback Rule (0x2C-0x2D):
     * - When countdown inactive (power_state != LOAD_ON_DELAY): Returns load_on_delay_config_sec
     * - When countdown active (power_state == LOAD_ON_DELAY): Returns load_on_delay_remaining_sec */
    uint16_t load_on_delay_config_sec; /* 0x2C-0x2D: Configured delay (0-65535) */
    uint16_t load_on_delay_remaining_sec; /* Remaining seconds when countdown active (0 when inactive) */
    
    /* Runtime counters */
    uint32_t cumulative_runtime_sec;   /* 0x1C-0x1F: RuntimeAtAll */
    uint32_t charging_time_sec;        /* 0x20-0x23: RuntimePowerOn */
    uint32_t current_runtime_sec;      /* 0x24-0x27: RuntimeOnetime */
    
    /* Version */
    uint16_t version;                  /* 0x28-0x29: Firmware version */
    
    /* Snapshot timing */
    uint32_t snapshot_tick;            /* TIM1 tick when snapshot was taken */
    uint32_t last_true_vbat_sample_tick; /* Tick when last true-VBAT was captured.
                                           * Must remain uint32_t; do not reintroduce 8-bit tick fields for staleness. */
} authoritative_state_t;

/* Charger Voltage Helper - Locks MAX(VBUS, USBIN) rule in code.
 * s must be non-NULL. Static inline eliminates macro footgun. */
static inline uint16_t UpsChargerVoltageMv(const authoritative_state_t *s) {
    return (s->usbc_voltage_mv > s->microusb_voltage_mv) ? 
           s->usbc_voltage_mv : s->microusb_voltage_mv;
}

/*===========================================================================*/
/*                         SNAPSHOT BUFFER STRUCTURE                          */
/*===========================================================================*/

/**
 * @brief Double-buffered Snapshot for I2C Coherence
 * 
 * Provides lock-free, atomic reads for I2C register access.
 * Writer updates inactive buffer, then atomically swaps active buffer pointer.
 * Reader latches active_buffer on I2C transaction start, uses until STOP.
 */
typedef struct {
    authoritative_state_t buffer[2];   /* Double buffer */
    volatile uint8_t active_buffer;    /* 0 or 1 - which buffer I2C reads from */
    volatile uint32_t snapshot_version; /* Debug: monotonic counter (not for coherence) */
} snapshot_buffer_t;

/*===========================================================================*/
/*                       FLASH PERSISTENCE STRUCTURE                          */
/*===========================================================================*/

/**
 * @brief Flash Persistent Data Structure
 * 
 * Stored in flash using A/B slot mechanism for safe updates.
 * Only RW configuration and learned values are persisted.
 * Countdown values (0x18, 0x1A) are runtime-only, NOT persisted.
 * 
 * CRC Calculation Rule:
 * - CRC32 covers bytes from full_voltage_mv through end of struct (charging_time_sec)
 * - CRC32 field itself is EXCLUDED from CRC calculation
 * - Header fields (magic, version, sequence_number, reserved_header) are EXCLUDED
 * - Reserved/padding fields MUST be zeroed before CRC calculation for deterministic results
 * 
 * Implementation Requirement:
 * - CRC implementation MUST compute CRC over [FLASH_CRC_START_OFFSET, FLASH_CRC_END_OFFSET)
 * - Do NOT use bespoke byte ranges; use the offset macros for safety
 */
typedef struct {
    /* Header (excluded from CRC) */
    uint32_t magic;                    /* Magic number: FLASH_MAGIC_NUMBER ("UPSP") */
    uint32_t version;                  /* Structure version */
    uint16_t sequence_number;          /* Monotonic sequence (wraps at 65535) */
    uint16_t reserved_header;          /* Padding for alignment - MUST be zeroed */
    
    /* CRC (excluded from CRC calculation) */
    uint32_t crc32;                    /* CRC32 of data section (full_voltage_mv through charging_time_sec) */
    
    /* Persistent configuration (included in CRC) */
    uint16_t full_voltage_mv;
    uint16_t empty_voltage_mv;
    uint16_t protection_voltage_mv;
    uint16_t sample_period_minutes;
    uint8_t auto_power_on;
    uint8_t battery_params_self_programmed;
    uint8_t low_battery_percent;
    uint8_t reserved_padding0;         /* Alignment + deterministic CRC - MUST be zeroed */
    uint16_t load_on_delay_config_sec;
    uint16_t reserved_padding1;        /* Alignment + deterministic CRC - MUST be zeroed */
    /* Note: Provide FlashPersistentData_InitZero(&rec) helper to ensure
     * all reserved/padding fields are zeroed before CRC calculation. */
    
    /* Runtime state (included in CRC, for recovery after unexpected power loss) */
    /* Note: power_status (register 0x17) is NOT persisted; it is derived from state machine on boot */
    /* Runtime counters persistence: Always persisted when flash is enabled (on every flash write).
     * These are "optional for recovery" per plan - they aid recovery but are not critical. */
    uint8_t reserved_padding2[4];       /* Alignment + deterministic CRC - MUST be zeroed */
    uint32_t cumulative_runtime_sec;
    uint32_t charging_time_sec;
} flash_persistent_data_t;

/* CRC calculation span - compile-time offsets for safety */
#define FLASH_CRC_START_OFFSET  offsetof(flash_persistent_data_t, full_voltage_mv)
#define FLASH_CRC_END_OFFSET    (sizeof(flash_persistent_data_t))
#define FLASH_CRC_SIZE          (FLASH_CRC_END_OFFSET - FLASH_CRC_START_OFFSET)

/* Compile-time assertion: flash structure must fit in slot size */
STATIC_ASSERT(sizeof(flash_persistent_data_t) <= FLASH_PAGE_SIZE,
               "flash_persistent_data_t exceeds FLASH_PAGE_SIZE");

/*===========================================================================*/
/*                         SCHEDULER STRUCTURES                               */
/*===========================================================================*/

/**
 * @brief Scheduler Flags - Set by TIM1 ISR, cleared by main loop (canonical scheduler)
 *
 * TIM1 ISR sets tick_10ms every 10ms, tick_100ms every 100ms, tick_500ms every 500ms.
 * Main loop derives tick_1s from tick_100ms (10 pulses = 1 second) and clears all flags after processing.
 * tick_counter is monotonic (never cleared); use it for precise timing deltas when needed.
 */
typedef struct {
    volatile uint8_t tick_10ms;    /* Set every 10ms, cleared by main loop */
    volatile uint8_t tick_100ms;   /* Set every 100ms, cleared by main loop */
    volatile uint8_t tick_500ms;    /* Set every 500ms, cleared by main loop */
    volatile uint8_t tick_1s;      /* Derived by main from tick_100ms (10 pulses = 1s), cleared by main loop */
    volatile uint32_t tick_counter; /* Free-running 10ms counter (monotonic, never cleared) */
} scheduler_flags_t;

/*===========================================================================*/
/*                         I2C PENDING WRITE STRUCTURE                        */
/*===========================================================================*/

/* I2C Write Limits: largest current RW register is 2 bytes; 4 is for future-proofing */
#define I2C_PENDING_WRITE_MAX_LEN      4       /* Maximum bytes for any register */

/**
 * @brief I2C Pending Write Buffer
 * 
 * I2C ISR stores write data here; main loop processes and applies to
 * authoritative state. This enforces the rule that only main loop
 * modifies authoritative state.
 */
typedef struct {
    volatile uint8_t reg_addr;         /* Starting register address (ISR writes, main reads) */
    volatile uint8_t data[I2C_PENDING_WRITE_MAX_LEN];  /* Write data buffer */
    volatile uint8_t length;           /* Number of data bytes (ISR writes, main reads) */
    volatile uint8_t pending;          /* Flag: 1 = pending write to process */
} i2c_pending_write_t;

/**
 * @brief I2C Transaction State (for snapshot latching)
 * 
 * Snapshot Latch Timing Rules (per plan):
 * - latched_snapshot_idx: Set on ADDR match (preferred) or first register-index byte (fallback)
 * - latched_snapshot_idx: NOT re-latched on repeated-start (must see STOP first)
 * - reg_ptr: Persists across repeated-start (for "write register then repeated-start read")
 * - in_transaction: Set on ADDR, cleared on STOPF
 * - All bytes in transaction use same latched snapshot buffer until STOPF
 */
typedef struct {
    uint8_t latched_snapshot_idx;      /* Buffer index latched at transaction start (ADDR match) */
    uint8_t in_transaction;            /* Flag: active transaction (set on ADDR, cleared on STOPF) */
    uint8_t reg_ptr;                   /* Current register index (persists across repeated-start) */
    uint8_t addr_phase_seen;           /* Flag: ADDR event detected */
} i2c_transaction_state_t;

/*===========================================================================*/
/*                              ADC FLAGS                                     */
/*===========================================================================*/

/**
 * @brief ADC Processing Flags
 * 
 * DMA ISR sets adc_ready; main loop processes and increments adc_sample_seq.
 * All sample-qualified logic uses adc_sample_seq with per-consumer tracking.
 * 
 * Critical Architectural Invariant - adc_sample_seq Increment Point:
 * adc_sample_seq++ MUST occur ONLY after:
 * 1. Raw ADC DMA data converted to scaled voltages (using current VREF calibration)
 * 2. Authoritative state updated with new voltage measurements
 * 3. Snapshot updated
 * 
 * This ensures all consumers see a consistent "new sample available" signal.
 * Any logic that depends on "new ADC data" MUST key exclusively off adc_sample_seq
 * with per-consumer last_seen_seq tracking.
 * Future risk: Do not use adc_ready for logic ("if adc_ready do X")—that can count
 * multiple times per sample or on non-sample cycles. Every sample-qualified consumer
 * keeps its own last_seen_seq and gates on (adc_sample_seq != last_seen_seq).
 */
/* ADC buffer size - must match ADC_CONVERTED_DATA_BUFFER_SIZE in main.c */
/* Use guard to prevent redefinition if already defined elsewhere (e.g., CubeMX) */
/* ADC Channel Ordering (verified from MX_ADC_Init sequence):
 * aADCxConvertedData[0] = Channel 0 (PIVCC)
 * aADCxConvertedData[1] = Channel 1 (VBAT)
 * aADCxConvertedData[2] = Channel 2 (VBUS)
 * aADCxConvertedData[3] = Channel 3 (USBIN)
 * aADCxConvertedData[4] = TEMPSENSOR (Temperature)
 * aADCxConvertedData[5] = VREFINT (for VDD calibration)
 * Channel ordering must match existing MX_ADC_Init sequence; do not reorder. */
#ifndef ADC_CONVERTED_DATA_BUFFER_SIZE
#define ADC_CONVERTED_DATA_BUFFER_SIZE 6
#endif

/* ADC flags and data - declared extern here, defined in main.c */
extern volatile uint8_t adc_ready;                    /* Set by DMA ISR, cleared by main loop */
extern volatile uint32_t adc_sample_seq;             /* Monotonic sequence, incremented AFTER processing */
extern volatile uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* Raw ADC DMA buffer */

/*===========================================================================*/
/*                         REGISTER MAP DEFINITIONS                           */
/*===========================================================================*/

/* Register addresses (for reference) */
#define REG_MCU_VOLTAGE_L       0x01
#define REG_MCU_VOLTAGE_H       0x02
#define REG_POGOPIN_VOLTAGE_L   0x03
#define REG_POGOPIN_VOLTAGE_H   0x04
#define REG_BATTERY_VOLTAGE_L   0x05
#define REG_BATTERY_VOLTAGE_H   0x06
#define REG_USBC_VOLTAGE_L      0x07
#define REG_USBC_VOLTAGE_H      0x08
#define REG_MICROUSB_VOLTAGE_L  0x09
#define REG_MICROUSB_VOLTAGE_H  0x0A
#define REG_TEMPERATURE_L       0x0B  /* Returns temperature in integer degrees Celsius (°C) */
#define REG_TEMPERATURE_H       0x0C  /* Returns temperature in integer degrees Celsius (°C) */
#define REG_FULL_VOLTAGE_L      0x0D
#define REG_FULL_VOLTAGE_H      0x0E
#define REG_EMPTY_VOLTAGE_L     0x0F
#define REG_EMPTY_VOLTAGE_H     0x10
#define REG_PROTECT_VOLTAGE_L   0x11
#define REG_PROTECT_VOLTAGE_H   0x12
/* Battery Percent Register Mapping Contract (0x13-0x14):
 * - battery_percent is stored as uint8_t (0-100) in authoritative_state
 * - Register mapping MUST emit [percent, 0x00] (LSB=percent, MSB=0)
 * Characterization: reg 0x14 (MSB) always 0x00 in legacy (see block at top). */
#define REG_BATTERY_PERCENT_L   0x13  /* LSB: battery_percent (0-100) */
#define REG_BATTERY_PERCENT_H   0x14  /* MSB: always 0x00 (verified) */
#define REG_SAMPLE_PERIOD_L     0x15
#define REG_SAMPLE_PERIOD_H     0x16
#define REG_POWER_STATUS        0x17
/* Countdown Registers (0x18, 0x1A, 0x2C-0x2D): Read has no side effects; decrement occurs only on tick_1s in main loop */
#define REG_SHUTDOWN_COUNTDOWN  0x18  /* Current countdown value (read from snapshot, no side effects) */
#define REG_AUTO_POWER_ON       0x19
#define REG_RESTART_COUNTDOWN   0x1A  /* Current countdown value (read from snapshot, no side effects) */
#define REG_FACTORY_RESET       0x1B
#define REG_RUNTIME_ALL_0       0x1C
#define REG_RUNTIME_ALL_1       0x1D
#define REG_RUNTIME_ALL_2       0x1E
#define REG_RUNTIME_ALL_3       0x1F
#define REG_RUNTIME_CHARGING_0  0x20
#define REG_RUNTIME_CHARGING_1  0x21
#define REG_RUNTIME_CHARGING_2  0x22
#define REG_RUNTIME_CHARGING_3  0x23
#define REG_RUNTIME_CURRENT_0   0x24
#define REG_RUNTIME_CURRENT_1   0x25
#define REG_RUNTIME_CURRENT_2   0x26
#define REG_RUNTIME_CURRENT_3   0x27
#define REG_VERSION_L           0x28
#define REG_VERSION_H           0x29
#define REG_BATTERY_SELF_PROG   0x2A
#define REG_LOW_BATTERY_PERCENT 0x2B
/* Load On Delay (0x2C-0x2D): Read has no side effects; decrement occurs only on tick_1s in main loop */
#define REG_LOAD_ON_DELAY_L     0x2C  /* Current countdown value (read from snapshot, no side effects) */
#define REG_LOAD_ON_DELAY_H     0x2D  /* Current countdown value (read from snapshot, no side effects) */

/* Reserved Regions - Behavior: ACK writes but discard; reads return 0x00.
 * Characterization: legacy returns 0x00 for reserved reads (see block at top).
 * Snapshot mapping MUST explicitly return 0x00 for any reg in [0x2E-0xEF] and [0xFC-0xFF];
 * do not fall through to memory.
 * Future risk: I2C write path must NOT store "last written value" for reserved
 * regs—writes are ACK'd but discarded; reserved reads always return 0x00. */
#define REG_RESERVED_START      0x2E  /* Start of reserved region */
#define REG_RESERVED_END        0xEF  /* End of reserved region */
#define REG_SERIAL_START        0xF0
#define REG_SERIAL_END          0xFB
#define REG_FACTORY_TEST_START  0xFC  /* Start of factory test region */
#define REG_FACTORY_TEST_END    0xFF  /* End of factory test region */

/* Reserved region helper macros for snapshot mapping */
#define IS_RESERVED_REG(addr)       ((addr) >= REG_RESERVED_START && (addr) <= REG_RESERVED_END)
#define IS_FACTORY_TEST_REG(addr)   ((addr) >= REG_FACTORY_TEST_START && (addr) <= REG_FACTORY_TEST_END)

/*===========================================================================*/
/*                         PIN BEHAVIOR TRUTH TABLE                           */
/*===========================================================================*/

/*
 * Pin Characterization Truth Table
 * ================================
 * 
 * IP_EN (PA5) - Input Power Enable (Charger Gate)
 * -----------------------------------------------
 * | Charger State          | IP_EN | Description                    |
 * |------------------------|-------|--------------------------------|
 * | CHARGER_STATE_ABSENT   | LOW   | No charger, path disabled      |
 * | CHARGER_STATE_PRESENT  | HIGH  | Charger connected and enabled  |
 * | CHARGER_STATE_FORCED_OFF_WINDOW | LOW | Measurement window active |
 * 
 * MT_EN (PA6) - RPi Power Enable
 * ------------------------------
 * | Power State                    | MT_EN | Description             |
 * |--------------------------------|-------|-------------------------|
 * | POWER_STATE_RPI_OFF            | LOW   | RPi power disabled      |
 * | POWER_STATE_RPI_ON             | HIGH  | RPi power enabled       |
 * | POWER_STATE_PROTECTION_LATCHED | LOW   | Protection shutdown     |
 * | POWER_STATE_LOAD_ON_DELAY      | LOW   | Waiting for delay       |
 * 
 * PWR_EN (PA7) - MCU Power Enable
 * -------------------------------
 * | State        | PWR_EN | Description                              |
 * |--------------|--------|------------------------------------------|
 * | Always       | HIGH   | MUST be HIGH or MCU loses power          |
 * 
 * Button (PB1) - Input with EXTI
 * ------------------------------
 * | Press Duration | Action                                          |
 * |----------------|------------------------------------------------|
 * | < 50ms         | Ignored (debounce)                              |
 * | 50ms - 9.99s   | Short press: Toggle RPi power                   |
 * | >= 10s         | Long press: Factory reset                       |
 */

/*===========================================================================*/
/*                         FUNCTION PROTOTYPES                                */
/*===========================================================================*/

/* These will be implemented in later phases */

/* State Machine */
void StateMachine_Init(void);
void StateMachine_Update(void);
/* Register 0x17 derivation: depends on learning_mode (from charger_state) and power_state.
 * Contract: Do not trust state->learning_mode unless it was recomputed during StateMachine_Update();
 * learning_mode is derived-only from charger_state and must be kept in sync when reading power status. */
uint8_t GetPowerStatusRegisterValue(const authoritative_state_t *auth_state, const system_state_t *state);

/* Snapshot Management */
/* Note: Register 0x17 (power_status) is computed during snapshot mapping via GetPowerStatusRegisterValue(),
 * not stored in authoritative_state_t. This prevents duplicated truth. */
void Snapshot_Init(void);
void Snapshot_Update(void);
uint8_t Snapshot_ReadRegister(uint8_t reg_addr);

/* Flash Persistence */
void Flash_Init(void);
void Flash_Load(void);
uint8_t Flash_Save(uint8_t bypass); /* Returns 1 on success or no-op, 0 on failure */
void Flash_FactoryReset(void);
uint8_t Flash_IsDirty(void);      /* Returns 1 if dirty, 0 if clean */
uint8_t Flash_CanWrite(void);    /* Returns 1 if rate limit allows, 0 if not */

/* ADC Processing */
void ADC_ProcessSample(void);

/* Scheduler */
void Scheduler_Init(void);
void Scheduler_ProcessTick(void);

/* Validation Functions */
uint8_t Validate_FullVoltage(uint16_t value);      /* Returns 1 if valid, 0 if invalid */
uint8_t Validate_EmptyVoltage(uint16_t value);     /* Returns 1 if valid, 0 if invalid */
uint8_t Validate_ProtectionVoltage(uint16_t value); /* Returns 1 if valid, 0 if invalid */
uint8_t Validate_SamplePeriod(uint16_t value);     /* Returns 1 if valid, 0 if invalid */
uint8_t Validate_Countdown(uint8_t value);          /* Returns 1 if valid, 0 if invalid */
uint8_t Validate_LowBatteryPercent(uint8_t value);  /* Returns 1 if valid, 0 if invalid */
uint8_t Validate_LoadOnDelay(uint16_t value);      /* Returns 1 if valid, 0 if invalid */

/* Utility */
/* Charger detection - distinguishes physical presence from path control */
/* Charger_UpdatePhysicalPresence: Updates charger_physical_state_t based on ADC readings.
 * This module owns stability counting and must update charger_stability_count and charger_last_seen_seq. */
void Charger_UpdatePhysicalPresence(const authoritative_state_t *state, 
                                     charger_physical_state_t *physical_state);
/* Updates physical_state->charger_physically_present, stability_count, last_seen_seq */

/* Charger_IsInfluencingVBAT: Canonical true-VBAT gate.
 * Returns 1 if charger_state == CHARGER_STATE_PRESENT (charger path enabled, influencing VBAT).
 * Returns 0 if charger_state != PRESENT (true-VBAT can be measured).
 * This is the canonical true-VBAT gate: true-VBAT iff returns 0. */
uint8_t Charger_IsInfluencingVBAT(const system_state_t *state);
/* Returns 1 if charger_state == CHARGER_STATE_PRESENT, 0 otherwise */

/* True-VBAT freshness check - uses TIM1 ticks (10ms resolution) */
/* Contract: Compares now_ticks against last_true_vbat_sample_tick and TRUE_VBAT_MAX_AGE_TICKS.
 * MUST NOT use SysTick milliseconds.
 * Wraparound: (now_ticks - last_true_vbat_sample_tick) MUST be computed as unsigned (uint32_t)
 * so that modulo-2^32 subtraction yields correct age when tick_counter has wrapped. */
uint8_t IsTrueVbatSampleFresh(uint32_t now_ticks, uint32_t last_true_vbat_sample_tick);
/* Returns 1 if fresh, 0 if stale */

#ifdef __cplusplus
}
#endif

#endif /* UPS_STATE_H */
