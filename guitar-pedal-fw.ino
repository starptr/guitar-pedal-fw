#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <SPI.h>

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

#define CODEC_DATA_OUT GPIO9
#define CODEC_BITCLK GPIO10
#define CODEC_LRCLK GPIO11
#define CODEC_SYSCLK GPIO12
#define CODEC_ML GPIO13
#define CODEC_RST GPIO14
#define CODEC_MC GPIO15
#define CODEC_MD GPIO16
#define CODEC_ZFLG GPIO17
#define CODEC_DATA_IN GPIO18

//// COPYPASTA
//const size_t bufferLen = 64;
//int16_t sBuffer[bufferLen];
//
//void i2s_install() {
//  // Set up I2S Processor configuration
//  const i2s_config_t i2s_config = {
//    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
//    .sample_rate = 44100,
//    .bits_per_sample = i2s_bits_per_sample_t(16),
//    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
//    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
//    .intr_alloc_flags = 0,
//    .dma_buf_count = 8,
//    .dma_buf_len = bufferLen,
//    .use_apll = false
//  };
//
//  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
//}
//
//void i2s_setpin() {
//  // Set I2S pin configuration
//  const i2s_pin_config_t pin_config = {
//    .bck_io_num = I2S_SCK,
//    .ws_io_num = I2S_WS,
//    .data_out_num = -1,
//    .data_in_num = I2S_SD
//  };
//
//  i2s_set_pin(I2S_PORT, &pin_config);
//}
//
//// END COPYPASTA

SPIClass* vspi = nullptr;

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("hello from esp32");

  // Set modes for pins connected to the codec
  // Only the DOUT, VOUTL, VOUTR, and ZFLG pins on the codec are inputs
  pinMode(CODEC_DATA_OUT, INPUT);
  pinMode(CODEC_BITCLK, OUTPUT);
  pinMode(CODEC_LRCLK, OUTPUT);
  pinMode(CODEC_SYSCLK, OUTPUT);
  pinMode(CODEC_ML, OUTPUT);
  pinMode(CODEC_RST, OUTPUT);
  pinMode(CODEC_MC, OUTPUT);
  pinMode(CODEC_MD, OUTPUT);
  pinMode(CODEC_ZFLG, INPUT);
  pinMode(CODEC_DATA_IN, OUTPUT);


  // Use SPI to configure the codec
  vspi = new SPIClass(VSPI);
  vspi->begin(
    CODEC_MC, // CLK
    GPIO3, // Unused CIPO
    CODEC_MD, // COPI
    CODEC_ML // CS (TODO: ML is CS?)
  );
  // The max SPI frequency should be 10MHz for the PCM3002 codec and 80MHz for the ESP32. We're setting it to 5MHz to be safe, but if all works well we can increase it to 10MHz.
  vspi->beginTransaction(SPISettings(5'000'000 /* freq */, MSBFIRST, SPI_MODE0));
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
  /* Setting the configurations, the slot configuration and clock configuration can be generated by the macros
   * These two helper macros are defined in `i2s_std.h` which can only be used in STD mode.
   * They can help to specify the slot and clock configurations for initialization or updating */
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_20BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = CODEC_BITCLK,
      .ws = CODEC_LRCLK,
      .dout = CODEC_DATA_IN,
      .din = CODEC_DATA_OUT,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
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

#define bufferLen 64;
void loop() {
  static int32_t sBuffer[bufferLen + 1 /* Add 1 just for the two_mean example */];
  size_t resBytesIn = 0;
  esp_err_t res = i2s_channel_read(rx_handle, &sBuffer, bufferLen, &resBytesIn, portMAX_DELAY);
  if (res != ESP_OK) {
    Serial.println("Error reading from I2S");
    return;
  }

  int32_t samples_read = resBytesIn / 2; // TODO: why 8?
  static int32_t wBuffer[bufferLen];
  if (samples_read > 0) {
    for (int32_t i = 0; i < samples_read; ++i) {
      float two_mean = 0;
      two_mean = (sBuffer[i] + sBuffer[i+1])/2;
      wBuffer[i] = two_mean;
    }
  }

  size_t resBytesOut = 0;
  esp_err_t res = i2s_channel_write(tx_handle, &wBuffer, resBytesIn, &resBytesOut, portMAX_DELAY);
}
