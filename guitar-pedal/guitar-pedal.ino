#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <SPI.h>
#include <cmath>

// Unused GPIOs
#define GPIO3 15
#define GPIO4 4

// GPIO9 through GPIO18 is relevant
#define GPIO9 17
#define GPIO10 18
#define GPIO11 19
#define GPIO12 20
#define GPIO13 21
#define GPIO14 22
#define GPIO15 8
#define GPIO16 9
#define GPIO17 10
#define GPIO18 11

#define GPIO0 27

#define CODEC_DATA_OUT_NUM GPIO_NUM_9
#define CODEC_BITCLK_NUM GPIO_NUM_10
#define CODEC_LRCLK_NUM GPIO_NUM_11
#define CODEC_SYSCLK_NUM GPIO_NUM_12
#define CODEC_ML_NUM GPIO_NUM_13
#define CODEC_RST_NUM GPIO_NUM_14
#define CODEC_MC_NUM GPIO_NUM_15
#define CODEC_MD_NUM GPIO_NUM_16
#define CODEC_ZFLG_NUM GPIO_NUM_17
#define CODEC_DATA_IN_NUM GPIO_NUM_18

// #define CODEC_DATA_OUT GPIO9
// #define CODEC_BITCLK GPIO10
// #define CODEC_LRCLK GPIO11
// #define CODEC_SYSCLK GPIO12
// #define CODEC_ML GPIO13
// #define CODEC_RST GPIO14
// #define CODEC_MC GPIO15
// #define CODEC_MD GPIO16
// #define CODEC_ZFLG GPIO17
// #define CODEC_DATA_IN GPIO18

SPIClass* vspi = nullptr;

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

static const size_t bufferSize = 120;
void setup() {
  Serial.begin(115200);
  Serial.println("hello from esp32");

  // Set modes for pins connected to the codec
  // Only the DOUT, VOUTL, VOUTR, and ZFLG pins on the codec are inputs
  pinMode(CODEC_DATA_OUT_NUM, INPUT);
  pinMode(CODEC_BITCLK_NUM, OUTPUT);
  pinMode(CODEC_LRCLK_NUM, OUTPUT);
  pinMode(CODEC_SYSCLK_NUM, OUTPUT);
  pinMode(CODEC_ML_NUM, OUTPUT);
  pinMode(CODEC_RST_NUM, OUTPUT);
  pinMode(CODEC_MC_NUM, OUTPUT);
  pinMode(CODEC_MD_NUM, OUTPUT);
  pinMode(CODEC_ZFLG_NUM, INPUT);
  pinMode(CODEC_DATA_IN_NUM, OUTPUT);

  // Using boot button also as a custom button
  pinMode(GPIO_NUM_0, INPUT);

  // Use SPI to configure the codec
  vspi = new SPIClass(3); // "VSPI" gave error
  // TODO: begin() signature is different from Arduino docs
  vspi->begin(
    CODEC_MC_NUM, // CLK
    GPIO_NUM_3, // Unused CIPO
    CODEC_MD_NUM, // COPI
    CODEC_ML_NUM // CS  // TODO: ML is CS?
  );
  // The max SPI frequency should be 10MHz for the PCM3002 codec and 80MHz for the ESP32. We're setting it to 5MHz to be safe, but if all works well we can increase it to 10MHz.
  // TODO: change frequency
  vspi->beginTransaction(SPISettings(10'000'000 /* freq */, MSBFIRST, SPI_MODE0));
  // Reg values: 5 reserved bits (all 0), 2 bits for register address, 9 bits for register value
  // Reg0 msb->lsb: 0b00000'00'0'00000000
  // bit[8] means we disable attenuation, and bits[7:0] are ignored (attenuation level config)
  // Reg1 msb->lsb: 0b00000'01'0'00000000
  // bit[8] means we disable attenuation, and bits[7:0] are ignored (attenuation level config)
  // Reg2 msb->lsb: 0b00000'01'0'0'0'1'0'0'01'0
  // bit[7] = 0 enables high-pass filter for ADC
  // bit[5] = 1 makes DAC attenuation the same for both channels (both controlled by reg0)
  // bit[4] enables infinite zero detection. Set to 0 for normal operation
  // bit[3] disables DAC output. Set to 0 for normal operation
  // bit[2:1] de-emphasis control. Set to 0b01 for normal operation
  // bit[0] is soft mute control. Set to 0 to disable soft mute
  // Reg3 msb->lsb: 0b00000'11'000'0'0'11'0'0
  // bit[5] is loopback. Set to 0 to disable loopback
  // bit[3:2] is data format. Select Format 3 for I2S by setting to 0b11
  // bit[1] is a polarity bit, which doesn't apply to Format 3

  // Reg0 rewritten: 0b0000000000000000
  // Reg1 rewritten: 0b0000001000000000
  // Reg2 rewritten: 0b0000001000100010
  // Reg3 rewritten: 0b0000011000001100
  // Reg0 separated bytes: 0b00000000 0b00000000
  // Reg1 separated bytes: 0b00000010 0b00000000
  // Reg2 separated bytes: 0b00000010 0b00100010
  // Reg3 separated bytes: 0b00000110 0b00001100
  // Send control register 0
  digitalWrite(vspi->pinSS(), LOW);
  vspi->transfer(0b00000000);
  vspi->transfer(0b00000000);
  digitalWrite(vspi->pinSS(), HIGH);
  delay(1);
  // Send control register 1
  digitalWrite(vspi->pinSS(), LOW);
  vspi->transfer(0b00000010);
  vspi->transfer(0b00000000);
  digitalWrite(vspi->pinSS(), HIGH);
  delay(1);
  // Send control register 2
  digitalWrite(vspi->pinSS(), LOW);
  vspi->transfer(0b00000010);
  vspi->transfer(0b00100010);
  digitalWrite(vspi->pinSS(), HIGH);
  delay(1);
  // Send control register 3
  digitalWrite(vspi->pinSS(), LOW);
  vspi->transfer(0b00000110);
  vspi->transfer(0b00001100);
  digitalWrite(vspi->pinSS(), HIGH);
  delay(1);

  // Notes on using driver/i2s_std.h
  // - The esp32s2 only supports the Standard mode in the i2s driver library
  // - The Standard Mode has 3 formats. We want the Philips format, because
  // it matches the format that the pcm3002 (our codec) uses (Format 3 in the
  // pcm3002 docs)

  // Example full-duplex (ie tx & rx) STD mode
  // #include <driver/i2s_std.h>
  // #include "driver/gpio.h"


  /// We want to use Philips Format, with 20 bits per slot
  /// LRCIN - a "clock" that chooses left (LOW) or right (HIGH) audio channel
  /// BCKIN - clock for bit data. Up-edge is when the data is valid, as specified by I2S

  /* Allocate a pair of I2S channel */
  /* Get the default channel configuration by the helper macro.
   * This helper macro is defined in `i2s_common.h` and shared by all the I2S communication modes.
   * It can help to specify the I2S role and port ID */
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  /* Allocate for TX and RX channel at the same time, then they will work in full-duplex mode */
  i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);

  /* Set the configurations for BOTH TWO channels, since TX and RX channel have to be same in full-duplex mode */
  /* Setting the configurations, the slot configuration and clock configuration can be generated by the macros.
   * These two helper macros are defined in `i2s_std.h` which can only be used in STD mode.
   * They can help to specify the slot and clock configurations for initialization or updating */
  /*
  TODO: figure out the correct combination of bitwidth & ws_width s.t. i2s configuration is correct,
  including "captures" (eg. input byte array size, etc.)
  */
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
    // .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) 20, I2S_SLOT_MODE_MONO),
    .slot_cfg = {
      .data_bit_width = (i2s_data_bit_width_t) 20,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
      .slot_mode = I2S_SLOT_MODE_MONO,
      .slot_mask = I2S_STD_SLOT_LEFT,
      .ws_width = 32,
      .ws_pol = false,
      .bit_shift = true,
      .msb_right = true,

      // .ws_pol = false,
      // .left_align = true,
      // .big_endian = false,
      // .bit_order_lsb = false,
      // .bit_shift = true
    },
    .gpio_cfg = {
      // from https://arc.net/l/quote/wehtbdsa
      .mclk = CODEC_SYSCLK_NUM, // Only appplicable for peripheral mode (but we are using the esp32 in controller mode)
      .bclk = CODEC_BITCLK_NUM,
      .ws = CODEC_LRCLK_NUM, // TODO: does this make sense with MONO?
      .dout = CODEC_DATA_IN_NUM,
      .din = CODEC_DATA_OUT_NUM,
      .invert_flags = { // TODO: Idk what these do
        .mclk_inv = false,
        .bclk_inv = true,
        .ws_inv = false,
      },
    },
  };
  /* Initialize the channel */
  i2s_channel_init_std_mode(tx_handle, &std_cfg);
  i2s_channel_init_std_mode(rx_handle, &std_cfg);

  i2s_channel_enable(tx_handle);
  i2s_channel_enable(rx_handle);


  /* If the configurations of slot or clock need to be updated,
   * stop the channel first and then update it */
  // i2s_channel_disable(tx_handle);
  // std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO; // Default is stereo
  // i2s_channel_reconfig_std_slot(tx_handle, &std_cfg.slot_cfg);
  // std_cfg.clk_cfg.sample_rate_hz = 96000;
  // i2s_channel_reconfig_std_clock(tx_handle, &std_cfg.clk_cfg);
}

esp_err_t res;
void loop() {
  Serial.println("poop");
  static uint8_t rBuffer[bufferSize];
  size_t resBytesIn = 0;
  res = i2s_channel_read(rx_handle, &rBuffer, bufferSize, &resBytesIn, portMAX_DELAY);
  if (res != ESP_OK) {
    Serial.println("Error reading from I2S");
    // return;
  } else {
    Serial.printf("zSuccess reading %ld bytes from I2S: ", resBytesIn);
    for (size_t i = 0; i < resBytesIn; i++) {
      Serial.printf("%ld-", rBuffer[i]);
    }
    Serial.println();
  }

  // int32_t samples_read = resBytesIn / 8; // TODO: the example code was 8; why 8?
  static uint8_t wBuffer[bufferSize];
  size_t sum = 0;
  if (resBytesIn > 0) {
    for (size_t i = 0; i < resBytesIn; ++i) {
      wBuffer[i] = rBuffer[i];
      sum += rBuffer[i];
    }
  }

  // vTaskDelay(pdMS_TO_TICKS(1000));
}

// Goat example:
// https://dronebotworkshop.com/esp32-i2s/
