// Custom Analog Front End (AFE) - Task 2
//
// Responsibilities:
//  (i)   generate time-multiplexed LED drive phases: RED -> DARK -> IR -> DARK -> repeat
//  (ii)  subtract the ambient (dark) phase from each lit phase
//  (iii) "digitize" the resulting RED/IR channels at 16-bit resolution,
//        at >=100 Sa/s per channel
//  (iv)  expose both channels (plus a small config/status register set)
//        to the ESP32 over SPI, as a readable/writable register map
//
// Phase timing: 1 ms per phase, 4 phases per cycle (RED, DARK, IR, DARK)
// -> 4 ms per cycle -> 250 Hz update rate PER CHANNEL (>= the 100 Sa/s spec).
//
// SPI protocol (mode 0, slave, no dedicated CS support in the API -> CS is
// watched manually):
//   Transaction = CS low ... CS high
//   Byte 1: address byte.  bit7 = 0 -> read, bit7 = 1 -> write.
//           bits[6:0] = register address (see REG_* below)
//   Read : AFE clocks back 1 or 2 data bytes (register-dependent width)
//   Write: master clocks 1 data byte, which is stored into the register
//   If CS is still low after the data phase, the AFE goes back to
//   expecting a new address byte (burst / multi-register access).
//
// Register map:
//   0x00 / 0x01  RED_MSB / RED_LSB   (RO) dark-subtracted RED, 16-bit
//   0x02 / 0x03  IR_MSB  / IR_LSB    (RO) dark-subtracted IR, 16-bit
//   0x04         STATUS              (RO) bit0 = DRDY (cleared on read)
//   0x05         LED_CURRENT         (RW) 0-255 (informational in this sim)
//   0x06         GAIN                (RW) 0-255 (informational in this sim)
//   0x07         SAMPLE_RATE         (RW) 0-255 (informational in this sim)
//
// See https://docs.wokwi.com/chips-api/spi for the SPI slave API this is
// built on.

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PHASE_US 1000  // 1 ms per phase -> 250 Hz per channel

#define REG_RED_MSB     0x00
#define REG_RED_LSB     0x01
#define REG_IR_MSB      0x02
#define REG_IR_LSB      0x03
#define REG_STATUS      0x04
#define REG_LED_CURRENT 0x05
#define REG_GAIN        0x06
#define REG_SAMPLE_RATE 0x07

typedef enum { PH_RED = 0, PH_DARK1, PH_IR, PH_DARK2 } phase_t;

typedef struct {
  // pins
  pin_t pd_in;
  pin_t red_en;
  pin_t ir_en;
  pin_t cs;

  // SPI
  spi_dev_t spi;
  int spi_stage;          // 0 = waiting for address byte, 1 = waiting/sending data
  uint8_t addr_buf[1];
  uint8_t data_buf[2];
  uint8_t current_addr;
  int     current_write;  // 1 if this transaction is a write

  // LED phase sequencer
  timer_t phase_timer;
  phase_t phase;
  float   red_lit;
  float   ir_lit;

  // register map
  uint16_t red_code;
  uint16_t ir_code;
  uint8_t  status;
  uint8_t  led_current;
  uint8_t  gain;
  uint8_t  sample_rate;
} chip_state_t;

static uint16_t volts_to_code16(float v) {
  if (v < 0) v = 0;
  if (v > 3.3f) v = 3.3f;
  return (uint16_t)((v / 3.3f) * 65535.0f);
}

static void start_addr_stage(chip_state_t *chip) {
  chip->spi_stage = 0;
  spi_start(chip->spi, chip->addr_buf, 1);
}

// Fill data_buf with the correct number of bytes for `addr`, returns width.
static int prepare_read_data(chip_state_t *chip, uint8_t addr) {
  switch (addr) {
    case REG_RED_MSB:
      chip->data_buf[0] = (uint8_t)(chip->red_code >> 8);
      chip->data_buf[1] = (uint8_t)(chip->red_code & 0xFF);
      return 2;
    case REG_IR_MSB:
      chip->data_buf[0] = (uint8_t)(chip->ir_code >> 8);
      chip->data_buf[1] = (uint8_t)(chip->ir_code & 0xFF);
      return 2;
    case REG_STATUS:
      chip->data_buf[0] = chip->status;
      chip->status &= ~0x01; // DRDY clears on read
      return 1;
    case REG_LED_CURRENT:
      chip->data_buf[0] = chip->led_current;
      return 1;
    case REG_GAIN:
      chip->data_buf[0] = chip->gain;
      return 1;
    case REG_SAMPLE_RATE:
      chip->data_buf[0] = chip->sample_rate;
      return 1;
    default:
      chip->data_buf[0] = 0xFF;
      return 1;
  }
}

static void apply_write(chip_state_t *chip, uint8_t addr, uint8_t value) {
  switch (addr) {
    case REG_LED_CURRENT: chip->led_current = value; break;
    case REG_GAIN:         chip->gain = value; break;
    case REG_SAMPLE_RATE:  chip->sample_rate = value; break;
    default: break; // writes to read-only regs are ignored
  }
}

static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (count == 0) {
    // Transaction was aborted (CS went high mid-byte) - just resync.
    return;
  }

  if (chip->spi_stage == 0) {
    // We just received the address byte.
    uint8_t addr_byte = buffer[0];
    chip->current_write = (addr_byte & 0x80) ? 1 : 0;
    chip->current_addr  = addr_byte & 0x7F;

    int width;
    if (chip->current_write) {
      width = 1; // always clock in exactly 1 byte to write
      chip->data_buf[0] = 0x00;
    } else {
      width = prepare_read_data(chip, chip->current_addr);
    }

    if (pin_read(chip->cs) == LOW) {
      chip->spi_stage = 1;
      spi_start(chip->spi, chip->data_buf, width);
    }
  } else {
    // We just finished the data phase (either a read-back or a write).
    if (chip->current_write) {
      apply_write(chip, chip->current_addr, buffer[0]);
    }
    if (pin_read(chip->cs) == LOW) {
      // Burst mode: go back to expecting the next address byte.
      start_addr_stage(chip);
    }
  }
}

static void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (pin == chip->cs) {
    if (value == LOW) {
      start_addr_stage(chip);
    } else {
      spi_stop(chip->spi);
    }
  }
}

static void set_leds(chip_state_t *chip, int red, int ir) {
  pin_write(chip->red_en, red ? HIGH : LOW);
  pin_write(chip->ir_en, ir ? HIGH : LOW);
}

static void phase_timer_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  float sample = pin_adc_read(chip->pd_in);

  switch (chip->phase) {
    case PH_RED:
      chip->red_lit = sample;   // sample RED-lit value at end of this phase
      chip->phase = PH_DARK1;
      set_leds(chip, 0, 0);
      break;

    case PH_DARK1: {
      float dark = sample;      // ambient right after RED
      float red_val = chip->red_lit - dark;
      if (red_val < 0) red_val = 0;
      chip->red_code = volts_to_code16(red_val);
      chip->phase = PH_IR;
      set_leds(chip, 0, 1);
      break;
    }

    case PH_IR:
      chip->ir_lit = sample;    // sample IR-lit value at end of this phase
      chip->phase = PH_DARK2;
      set_leds(chip, 0, 0);
      break;

    case PH_DARK2: {
      float dark = sample;      // ambient right after IR
      float ir_val = chip->ir_lit - dark;
      if (ir_val < 0) ir_val = 0;
      chip->ir_code = volts_to_code16(ir_val);
      chip->status |= 0x01;     // new RED+IR pair ready -> DRDY
      chip->phase = PH_RED;
      set_leds(chip, 1, 0);
      break;
    }
  }
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->pd_in  = pin_init("PD_IN", ANALOG);
  chip->red_en = pin_init("RED_EN", OUTPUT);
  chip->ir_en  = pin_init("IR_EN", OUTPUT);
  chip->cs     = pin_init("CS", INPUT_PULLUP);

  const spi_config_t spi_config = {
    .sck = pin_init("SCK", INPUT),
    .miso = pin_init("MISO", OUTPUT),
    .mosi = pin_init("MOSI", INPUT),
    .mode = 0,
    .done = chip_spi_done,
    .user_data = chip,
  };
  chip->spi = spi_init(&spi_config);

  const pin_watch_config_t cs_watch = {
    .edge = BOTH,
    .pin_change = chip_pin_change,
    .user_data = chip,
  };
  pin_watch(chip->cs, &cs_watch);

  chip->led_current = 50;
  chip->gain = 4;
  chip->sample_rate = 100;
  chip->phase = PH_RED;
  set_leds(chip, 1, 0);

  const timer_config_t phase_cfg = {
    .callback = phase_timer_tick,
    .user_data = chip,
  };
  chip->phase_timer = timer_init(&phase_cfg);
  timer_start(chip->phase_timer, PHASE_US, true);

  printf("AFE chip initialized\n");
}
