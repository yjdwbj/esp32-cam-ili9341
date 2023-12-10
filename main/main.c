/**
 * This example takes a picture every 5s and print its size on serial monitor.
 */

// =============================== SETUP ======================================

// 1. Board setup (Uncomment):
// #define BOARD_WROVER_KIT
#define BOARD_ESP32CAM_AITHINKER

/**
 * 2. Kconfig setup
 *
 * If you have a Kconfig file, copy the content from
 *  https://github.com/espressif/esp32-camera/blob/master/Kconfig into it.
 * In case you haven't, copy and paste this Kconfig file inside the src directory.
 * This Kconfig file has definitions that allows more control over the camera and
 * how it will be initialized.
 */

/**
 * 3. Enable PSRAM on sdkconfig:
 *
 * CONFIG_ESP32_SPIRAM_SUPPORT=y
 *
 * More info on
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-esp32-spiram-support
 */

// ================================ CODE ======================================

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <string.h>
#include <sys/param.h>

#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define STR(x) #x
#define STRINGIFY(x) STR(x)

#if defined(_WIFI_SSID) && defined(_WIFI_PWD)
#define USE_NET 1
#endif

#if USE_NET
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"
#include <esp_http_server.h>

typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#endif

static const char *TAG = "esp32_cam:http_jpg";
#define LCD_H_RES (320)
#define LCD_V_RES (240)
#define LCD_BIT_PER_PIXEL (16)
#define ESP_CAMERA_SUPPORTED 1

uint8_t *_jpg_buf = NULL;
size_t _jpg_buf_len = 0;
volatile uint32_t _jpg_frame_number = 0;

#ifdef USE_ILI9341_DRIVER
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "lcd_ili9341.h"
#include "decode_image.h"
#endif



// ESP32Cam (AiThinker) PIN Map

#ifdef BOARD_ESP32CAM_AITHINKER

#define LCD_HOST SPI2_HOST

#define TFT_LCD_CS      (12)
#define TFT_LCD_SCK     (13)
#define TFT_LCD_MOSI    (15)
#define TFT_LCD_DC      (14)
#define TFT_LCD_RST     (2)
#define TFT_LCD_BL      (-1) // High level is valid connect to VCC

#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1 // software reset will be performed
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#endif

// #define BOARD_ESP32_S2_OV2640 1
#define LCD_HOST SPI2_HOST
#ifdef BOARD_ESP32_S2_OV2640

#define TFT_LCD_CS (14)
#define TFT_LCD_SCK (12)
#define TFT_LCD_MOSI (11)
#define TFT_LCD_DC (10)
#define TFT_LCD_RST (13)
#define TFT_LCD_BL (9)

#define CAM_PIN_PWDN 3
#define CAM_PIN_RESET 34 // software reset will be performed
#define CAM_PIN_XCLK -1
#define CAM_PIN_SIOD 18
#define CAM_PIN_SIOC 16

#define CAM_PIN_D7 1
#define CAM_PIN_D6 39
#define CAM_PIN_D5 40
#define CAM_PIN_D4 37
#define CAM_PIN_D3 38
#define CAM_PIN_D2 35
#define CAM_PIN_D1 36
#define CAM_PIN_D0 33
#define CAM_PIN_VSYNC 17
#define CAM_PIN_HREF 21
#define CAM_PIN_PCLK 2
#endif

#ifdef USE_ILI9341_DRIVER
static esp_lcd_panel_handle_t panel_handle = NULL;

static void inital_lcd() {

#if TFT_LCD_BL >= 0
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(TFT_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(TFT_LCD_BL, 1);
#endif
    const spi_bus_config_t buscfg = {
        .sclk_io_num = TFT_LCD_SCK,
        .mosi_io_num = TFT_LCD_MOSI,
        .miso_io_num = -1,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        // The max_transfer_sz value will occur the below error when it over 8192.
        // E (14337) lcd_panel.io.spi: panel_io_spi_tx_color(387): spi transmit (queue) color failed
        .max_transfer_sz = 8192,
    };

    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH1);

    // ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = {                                                               \
        .cs_gpio_num = TFT_LCD_CS,
        .dc_gpio_num = TFT_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };

    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TFT_LCD_RST, // Shared with Touch reset
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
#else
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
#endif
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };

    esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_off(panel_handle, false);
#else
    esp_lcd_panel_disp_on_off(panel_handle, true);
#endif
}

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
esp_lcd_touch_handle_t tp = NULL;
#endif

#define PARALLEL_LINES  (16)
#define ROTATE_FRAME  (30)

static uint16_t *s_lines[2];

uint16_t *pixels;

// Grab a rgb16 pixel from the esp32_tiles image
static inline uint16_t get_bgnd_pixel(int x, int y) {
    // Get color of the pixel on x,y coords
    return (uint16_t) * (pixels + (y * IMAGE_W) + x);
}

// Instead of calculating the offsets for each pixel we grab, we pre-calculate the valueswhenever a frame changes, then re-use
// these as we go through all the pixels in the frame. This is much, much faster.
static int8_t xofs[320], yofs[240];
static int8_t xcomp[320], ycomp[240];

// Calculate the pixel data for a set of lines (with implied line size of 320). Pixels go in dest, line is the Y-coordinate of the
// first line to be calculated, linect is the amount of lines to calculate. Frame increases by one every time the entire image
// is displayed; this is used to go to the next frame of animation.
void pretty_effect_calc_lines(uint16_t *dest, int line, int linect) {
    for (int y = line; y < line + linect; y++) {
        for (int x = 0; x < 320; x++) {
            *dest++ = get_bgnd_pixel(x + yofs[y] + xcomp[x], y + xofs[x] + ycomp[y]);
        }
    }
}

static void display_pretty_colors(esp_lcd_panel_handle_t panel_handle) {
    // Indexes of the line currently being sent to the LCD and the line we're calculating
    int sending_line = 0;
    int calc_line = 0;

    // After ROTATE_FRAME frames, the image will be rotated
    for (int y = 0; y < LCD_V_RES; y += PARALLEL_LINES) {
        // Calculate a line
        pretty_effect_calc_lines(s_lines[calc_line], y, PARALLEL_LINES);
        sending_line = calc_line;
        calc_line = !calc_line;
        // Send the calculated data
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 0 + LCD_H_RES, y + PARALLEL_LINES, s_lines[sending_line]);
    }
}

static void tftlcd_task(void)
{
    static camera_fb_t *pic = NULL;
    while (1) {
        pic = esp_camera_fb_get();
        _jpg_buf = pic->buf;
        _jpg_buf_len = pic->len;
        _jpg_frame_number++;
        decode_image(&pixels, pic);
        display_pretty_colors(panel_handle);
        esp_camera_fb_return(pic);
        free(pixels);
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}
#endif

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn   = CAM_PIN_PWDN,
    .pin_reset  = CAM_PIN_RESET,
    .pin_xclk   = CAM_PIN_XCLK,
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

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 24000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,     // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 16, // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,      // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera(void) {
    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}
#endif

#if USE_NET

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t http_server = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char *part_buf[64];

    static int64_t last_frame = 0;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }

    while (true) {
        if (last_frame == _jpg_frame_number)
            continue;
        last_frame = _jpg_frame_number;
        if (res == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
    }
    return res;
}

static esp_err_t jpg_httpd_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    res = httpd_resp_set_type(req, "image/jpeg");
    if (res == ESP_OK) {
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }
    if (res == ESP_OK) {
        res = httpd_resp_send(req, (const char *)_jpg_buf,_jpg_buf_len);
    }
    return res;
}

httpd_uri_t uri_handler_jpg = {
    .uri = "/jpg",
    .method = HTTP_GET,
    .handler = jpg_httpd_handler};

httpd_uri_t uri_handler_stream = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler};

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&http_server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(http_server, &uri_handler_jpg);
        httpd_register_uri_handler(http_server, &uri_handler_stream);
        return;
    }

    ESP_LOGI(TAG, "Error starting server!");
}

void stop_webserver() {
    // Stop the httpd server
    httpd_stop(http_server);
}

#define CONFIG_EXAMPLE_ESP_MAXIMUM_RETRY 5
#define EXAMPLE_ESP_MAXIMUM_RETRY CONFIG_EXAMPLE_ESP_MAXIMUM_RETRY

static uint8_t own_addr_type;
static int s_retry_num = 0;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            ESP_ERROR_CHECK(esp_wifi_connect());
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            if (http_server != NULL)
                stop_webserver();
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        if(http_server == NULL)
            start_webserver();
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void initialise_wifi(void) {
    s_wifi_event_group = xEventGroupCreate();
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = STRINGIFY(_WIFI_SSID),
            .password = STRINGIFY(_WIFI_PWD),
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 STRINGIFY(_WIFI_SSID), STRINGIFY(_WIFI_PWD));
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 STRINGIFY(_WIFI_SSID), STRINGIFY(_WIFI_PWD));
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}
#endif


void app_main(void) {
#if ESP_CAMERA_SUPPORTED
    if (ESP_OK != init_camera()) {
        return;
    }
#ifdef USE_ILI9341_DRIVER
    inital_lcd();
#endif
#if USE_NET
    // ESP_ERROR_CHECK(nvs_flash_erase());
    esp_err_t ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);
    initialise_wifi();
#endif

#ifdef USE_ILI9341_DRIVER

    for (int i = 0; i < 2; i++) {
        s_lines[i] = heap_caps_malloc(LCD_H_RES * PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA);
        assert(s_lines[i] != NULL);
    }
    TaskHandle_t myTaskHandle = NULL;
    xTaskCreate(tftlcd_task, "Tftlcd_task", 4096, NULL, 1, &myTaskHandle);
#endif
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
#endif
}
