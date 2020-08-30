/*--------------------------------------------------------------------
Copyright 2020 fukuen
fft demo is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This software is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with This software.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include "i2s.h"
#include "fpioa.h"
#include "gpio.h"
#include "fft.h"

#ifdef M5STICKV
#define AXP192_ADDR 0x34
#define PIN_SDA 29
#define PIN_SCL 28
#endif

// common
#define WIDTH 280
#define HEIGHT 240
#define FFT_N              512U
#define FRAME_LEN          FFT_N
#define SAMPLING_FREQUENCY 16000

int i, j;
uint16_t rx_buf[FRAME_LEN * 2];
uint32_t g_rx_dma_buf[FRAME_LEN * 2 * 2];
uint16_t g_lcd_gram[WIDTH * HEIGHT] __attribute__((aligned(64)));

volatile uint32_t g_index;
volatile uint8_t i2s_rec_flag;
volatile uint8_t i2s_start_flag = 0;

// fft
#define FFT_FORWARD_SHIFT   0x0U
#define FFT_BACKWARD_SHIFT  0x1ffU

typedef enum _complex_mode
{
    FFT_HARD = 0,
    FFT_SOFT = 1,
    FFT_COMPLEX_MAX,
} complex_mode_t;

fft_data_t fft_in_data[FFT_N];
fft_data_t fft_out_data[FFT_N];
complex_hard_t data_hard[FFT_N] = {0};
float hard_power[FFT_N];

TFT_eSPI lcd;

int i2s_dma_irq(void *ctx) {
    uint32_t i;
    if (i2s_start_flag) {
        int16_t s_tmp;
        if (g_index) {
            i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[g_index], FRAME_LEN * 2, DMAC_CHANNEL0);
            g_index = 0;
            for (i = 0; i < FRAME_LEN; i++) {
                s_tmp = (int16_t)(g_rx_dma_buf[2 * i] & 0xffff); //g_rx_dma_buf[2 * i + 1] Low left
                rx_buf[i] = s_tmp + 32768;
            }
            i2s_rec_flag = 1;
        } else {
            i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);
            g_index = FRAME_LEN * 2;
            for (i = FRAME_LEN; i < FRAME_LEN * 2; i++) {
                s_tmp = (int16_t)(g_rx_dma_buf[2 * i] & 0xffff);//g_rx_dma_buf[2 * i + 1] Low left
                rx_buf[i] = s_tmp + 32768;
            }
            i2s_rec_flag = 2;
        }
    } else {
        i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);
        g_index = FRAME_LEN * 2;
    }
    return 0;
}

bool axp192_init() {
    Serial.printf("AXP192 init.\n");
    sysctl_set_power_mode(SYSCTL_POWER_BANK3,SYSCTL_POWER_V33);

    Wire.begin((uint8_t) PIN_SDA, (uint8_t) PIN_SCL, 400000);
    Wire.beginTransmission(AXP192_ADDR);
    int err = Wire.endTransmission();
    if (err) {
        Serial.printf("Power management ic not found.\n");
        return false;
    }
    Serial.printf("AXP192 found.\n");

    // Clear the interrupts
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x46);
    Wire.write(0xFF);
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x23);
    Wire.write(0x08); //K210_VCore(DCDC2) set to 0.9V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x33);
    Wire.write(0xC1); //190mA Charging Current
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x36);
    Wire.write(0x6C); //4s shutdown
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x91);
    Wire.write(0xF0); //LCD Backlight: GPIO0 3.3V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x90);
    Wire.write(0x02); //GPIO LDO mode
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x28);
    Wire.write(0xF0); //VDD2.8V net: LDO2 3.3V,  VDD 1.5V net: LDO3 1.8V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x27);
    Wire.write(0x2C); //VDD1.8V net:  DC-DC3 1.8V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x12);
    Wire.write(0xFF); //open all power and EXTEN
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x23);
    Wire.write(0x08); //VDD 0.9v net: DC-DC2 0.9V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x31);
    Wire.write(0x03); //Cutoff voltage 3.2V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x39);
    Wire.write(0xFC); //Turnoff Temp Protect (Sensor not exist!)
    Wire.endTransmission();

    fpioa_set_function(23, (fpioa_function_t)(FUNC_GPIOHS0 + 26));
    gpiohs_set_drive_mode(26, GPIO_DM_OUTPUT);
    gpiohs_set_pin(26, GPIO_PV_HIGH); //Disable VBUS As Input, BAT->5V Boost->VBUS->Charing Cycle

    msleep(20);
    return true;
}

int pio_init() {
    fpioa_set_function(12, FUNC_I2S0_IN_D0);
    fpioa_set_function(10, FUNC_I2S0_WS);
    fpioa_set_function(13, FUNC_I2S0_SCLK);

    //i2s init
    i2s_init(I2S_DEVICE_0, I2S_RECEIVER, 0x03);

    i2s_rx_channel_config(I2S_DEVICE_0, I2S_CHANNEL_0,
            RESOLUTION_16_BIT, SCLK_CYCLES_32,
            TRIGGER_LEVEL_4, STANDARD_MODE);

    uint32_t ret = i2s_set_sample_rate(I2S_DEVICE_0, SAMPLING_FREQUENCY);
    printf("actual rate %ul\n", ret);

//    plic_init();
    dmac_init();
    dmac_set_irq(DMAC_CHANNEL0, i2s_dma_irq, NULL, 1);
    i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);

    /* Enable the machine interrupt */
    sysctl_enable_irq();
    return 0;
}

void FFT(int offset) {
    for (i = 0; i < FFT_N / 2; i++) {
        fft_in_data[i].I1 = 0;
        fft_in_data[i].R1 = rx_buf[2 * i + offset] - 32768;
        fft_in_data[i].I2 = 0;
        fft_in_data[i].R2 = rx_buf[2 + i + 1 + offset] - 32768;
    }
    fft_complex_uint16_dma(DMAC_CHANNEL1, DMAC_CHANNEL2, FFT_FORWARD_SHIFT, FFT_DIR_FORWARD, (uint64_t *)fft_in_data, FFT_N, (uint64_t *)fft_out_data);
    for (i = 0; i < FFT_N / 2; i++) {
        data_hard[2 * i].imag = fft_out_data[i].I1;
        data_hard[2 * i].real = fft_out_data[i].R1;
        data_hard[2 * i + 1].imag = fft_out_data[i].I2;
        data_hard[2 * i + 1].real = fft_out_data[i].R2;
    }
}

#define SWAP_16(x) ((x >> 8 & 0xff) | (x << 8))

void update_image_fft(float* hard_power, float pw_max, uint32_t* pImage, uint32_t color, uint32_t bkg_color) {
    uint32_t bcolor= SWAP_16((bkg_color << 16)) | SWAP_16(bkg_color);
    uint32_t fcolor= SWAP_16((color << 16)) | SWAP_16(color);

    int  h[80];

    int x = 0;

    for (i = 0; i < 80; i++) {
        h[i]=120*(hard_power[i])/pw_max;

        if (h[i]>120)
            h[i] = 120;
        if (h[i]<0)
            h[i] = 0;
    }

    for (i = 0; i < 80; i++) {  // 53* 38640/512 => ~4000Hz
        x=i*2;
        for( int y=0; y<120; y++) {
            if( y<(120 - h[i+2]) ) {
                pImage[x+y*2*140]=bcolor;
                pImage[x+1+y*2*140]=bcolor;
                pImage[x+(y*2+1)*140]=bcolor;
                pImage[x+1+(y*2+1)*140]=bcolor;
            } else {
                pImage[x+y*2*140]=fcolor;
                pImage[x+1+y*2*140]=fcolor;
                pImage[x+(y*2+1)*140]=bcolor;
                pImage[x+1+(y*2+1)*140]=bcolor;
            }
        }
    }
}

void drawFft(int offset) {
    FFT(offset);

    float pmax=10;
    for (i = 0; i < FFT_N / 2; i++) {
        hard_power[i] = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);

        //Convert to dBFS
        hard_power[i] = 20*log(2*hard_power[i]/FFT_N);

        if( hard_power[i]>pmax)
            pmax = hard_power[i];
    }
    update_image_fft(hard_power, 140 /*MAX range dBFS*/, (uint32_t *)g_lcd_gram, TFT_BLUE, TFT_BLACK);
    lcd.pushImage(0, 0, WIDTH, HEIGHT, (uint16_t*)g_lcd_gram);
}

void setup() {
    Serial.begin(115200);
    axp192_init();

    /* LCD init */
    lcd.begin();
    lcd.setRotation(1);

    pio_init();

    g_index = 0;
    i2s_rec_flag = 0;
    i2s_start_flag = 1;
}

void loop() {
    if (i2s_rec_flag == 0) {
        //continue
    } else if (i2s_rec_flag == 1) {
        drawFft(0);
    } else {
        drawFft(FRAME_LEN);
    }
}