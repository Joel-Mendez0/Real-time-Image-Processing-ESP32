#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include "driver/spi_common.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "jpeg_decoder.h"
#include "esp_err.h"

// Button to swap output
bool color_mode = false;
#define SWITCH 32
uint8_t switch_pin = SWITCH;
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg){
	int gpio = (int)arg;
	xQueueSendFromISR(gpio_evt_queue,&gpio,NULL);
}
void init_gpio(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Trigger on falling edge (button press)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    io_conf.pin_bit_mask = 0;
    io_conf.pin_bit_mask |= (1ULL << switch_pin);
    gpio_config(&io_conf);
    
    gpio_evt_queue = xQueueCreate(10, sizeof(int));
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(switch_pin, gpio_isr_handler, (void*)(intptr_t)switch_pin);	
}
void handle_input(){
	int gpio;
	static uint32_t last_press_time = 0;
	uint32_t current_time = xTaskGetTickCount();
	
	    if (xQueueReceive(gpio_evt_queue, &gpio, pdMS_TO_TICKS(10))) {
        	if (current_time - last_press_time > pdMS_TO_TICKS(300)) {
            	printf("Button GPIO %d pressed\n", gpio);
            	last_press_time = current_time;
            
            	switch (gpio) {
                	case SWITCH:
                		color_mode = !color_mode;
                		 
                		 if(color_mode == false){
							 printf("RGB Capture Mode\n");
						 }
						 else{
							 printf("Grayscale/Threshold Capture Mode\n");
						 }
                	default:
                    	break;
            }
        }
    }
}

//WROVER-KIT PIN Map
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    21
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      19
#define CAM_PIN_D2      18
#define CAM_PIN_D1       5
#define CAM_PIN_D0       4
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// Screen
#define PARALLEL_LINES 16
#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 240
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8
#define BACKGROUND 0x0000;

#define LCD_SPI_HOST    SPI3_HOST
#define LCD_PIN_NUM_MOSI 13
#define LCD_PIN_NUM_MISO 12
#define LCD_PIN_NUM_CLK  14
#define LCD_PIN_NUM_CS   15
#define LCD_PIN_NUM_DC    2

#define MAX_IMAGE_SIZE (EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 2) // Assuming RGB565 format
uint8_t out_img_buf[MAX_IMAGE_SIZE];

esp_lcd_panel_handle_t panel_handle;

void fill_screen_color(esp_lcd_panel_handle_t panel_handle, uint16_t color)
{
    uint16_t *line = heap_caps_malloc(EXAMPLE_LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    assert(line != NULL);
    for (int i = 0; i < EXAMPLE_LCD_H_RES; i++) {
        line[i] = color;
    }
    for (int y = 0; y < EXAMPLE_LCD_V_RES; y++) {

        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, EXAMPLE_LCD_H_RES, y + 1, line);
    }

    heap_caps_free(line);
}

static const char *TAG = "esp32-camera";

static spi_bus_config_t lcd_buscfg = {
    .sclk_io_num = LCD_PIN_NUM_CLK,
    .mosi_io_num = LCD_PIN_NUM_MOSI,
    .miso_io_num = LCD_PIN_NUM_MISO,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = PARALLEL_LINES * EXAMPLE_LCD_H_RES * 2 + 8
};
    
esp_lcd_panel_io_handle_t io_handle = NULL;

static esp_lcd_panel_io_spi_config_t lcd_io_config = {
    .dc_gpio_num = LCD_PIN_NUM_DC,
    .cs_gpio_num = LCD_PIN_NUM_CS,
    .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
    .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
    .spi_mode = 0,
    .trans_queue_depth = 10,
};

static esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

esp_err_t camera_init(){
    if(CAM_PIN_PWDN != -1){
        gpio_set_direction(CAM_PIN_PWDN, GPIO_MODE_OUTPUT);
        gpio_set_level(CAM_PIN_PWDN, 0);
    }
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}
esp_err_t draw_captured_image(esp_lcd_panel_handle_t panel_handle, int width, int height, pixformat_t format, uint8_t* buf, size_t len) {
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = buf,
        .indata_size = len,
        .outbuf = out_img_buf,
        .outbuf_size = sizeof(out_img_buf),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 1,
        }
    };
    esp_jpeg_image_output_t outimg;
 
    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode failed");
        return ret;
    }
    
    if(color_mode == true){
    // Grayscale Conversion & Threshold
    for (int i = 0; i < outimg.width * outimg.height; i++) {
        // Extract RGB565 components
        uint16_t pixel = ((uint16_t*)out_img_buf)[i];
        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5) & 0x3F;
        uint8_t b = pixel & 0x1F;

        // Convert to 8-bit per channel
        r = (r << 3) | (r >> 2);
        g = (g << 2) | (g >> 4);
        b = (b << 3) | (b >> 2);

        // Compute grayscale value
        uint8_t gray = (r * 30 + g * 59 + b * 11) / 100;

        // Apply threshold
        if (gray >= 240) {
            gray = 255;
        } else {
            gray = 0;
        }

        // Store back as RGB565
        r = gray >> 3;
        g = gray >> 2;
        b = gray >> 3;

        ((uint16_t*)out_img_buf)[i] = (r << 11) | (g << 5) | b;
    }
    }

    ESP_LOGI(TAG, "Decoded image size: %dx%d", outimg.width, outimg.height);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, outimg.width, outimg.height, out_img_buf));

    return ESP_OK;
}


esp_err_t process_image(int width, int height, pixformat_t format, uint8_t* buf, size_t len) {
    ESP_LOGI("process_image", "Processing image with length = %d", len);

    return draw_captured_image(panel_handle, width, height, format, buf, len);
}

esp_err_t camera_capture() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }

    esp_err_t err = process_image(fb->width, fb->height, fb->format, fb->buf, fb->len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to process image");
    }
    esp_camera_fb_return(fb);
    return err;
}

void app_main(void)
{	
	
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &lcd_buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &lcd_io_config, &io_handle));
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
	
	init_gpio();
	camera_init();
	fill_screen_color(panel_handle, 0xFFFF);
    
    while (true) {
        camera_capture();
        handle_input();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}