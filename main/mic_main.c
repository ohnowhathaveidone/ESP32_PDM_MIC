/*  testing i2s microphone on esp32
    based on:
    article: https://reversatronics.blogspot.com/2020/06/i2s-microphones-on-esp32-how-high-can-i.html?m=0 
    code: https://github.com/GrahamM/ESP32_I2S_Microphone 

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include <driver/i2s.h>
#include <soc/i2s_reg.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
//#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "freertos/event_groups.h"

//adjust to suit your wifi settings
#define SSID "iamnotgoodwithcomputer"
#define PASS "ohnowhathaveidone"
#define MAXRETRY 10

static const char *TAG = "micStreamer";

// takes care of wifi connection
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static bool hasWifi = false;
static bool UDPruns = false;
static bool bufReady = false;
static uint16_t bufOffset = 0;

static int s_retry_num = 0; //ok...

//buffer for mic data
//uint8_t buffer[2048];
//uint32_t buffer[512]; 
uint32_t buffer[1024];

//volatile uint16_t rPtr = 0; // Yes, I should be using a pointer.
//size_t readOut;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXRETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID: %s password: %s",
                 SSID, PASS);
        hasWifi = true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s, password: %s",
                 SSID, PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

// UDP Destination
//IPAddress udpAddress(192, 168, 1, 101);
// Connection state
//bool connected = false;

//this is for the udp server

static struct sockaddr_in6 dest_addr;
int sock = -1;
int len = -1;
struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
socklen_t socklen = sizeof(source_addr);

const int udpPort = 3333;
static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    while (1) {

        if (sock<0) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(udpPort);
            ip_protocol = IPPROTO_IP;

            sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
            //sock = socket(addr_family, SOCK_STREAM, ip_protocol);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Socket created");

            int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            }
            ESP_LOGI(TAG, "Socket bound, port %d", udpPort);
            UDPruns = true;
        }
        
        //int len = -1;
        //struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        //socklen_t socklen = sizeof(source_addr);

        while (1) {
            if (UDPruns && len<0) {
                ESP_LOGI(TAG, "Waiting for data");
                len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
                // Get the sender's ip address as string
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                // Error occurred during receiving
                if (len < 0) {
                    ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                    break;
                }
            } else if (bufReady && len>0) {
                //printf("sending data w/ offset %d\n", bufOffset); //too slow?..
                
                //TODO: replace 1024 w/ variable!!!!!
                bufReady = false;
                //printf("%d\n", bufOffset);
                int err = sendto(sock, (uint8_t*)buffer + bufOffset, 2048, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            } else {
                //vTaskDelay(xDelay);
                continue;
            }
        }

        /*
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }*/
    }
    vTaskDelete(NULL);
}

// microphone stuff
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 128;

void I2SSetup() {
    esp_err_t err;

    // The I2S config as per the example
    static const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM, // We drive clk, only receive. NOTE: in PDM mode, clk is on WS line
        .sample_rate = 200000, //record with sample_rate/2 
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, //I2S_BITS_PER_SAMPLE_32BIT/I2S_BITS_PER_SAMPLE_24BIT/I2S_BITS_PER_SAMPLE_16BIT, // WS signal must be BCLK/64 - this is how we manage it
        .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,  // Left by default 
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,// (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
        .dma_buf_count = 4,                           // number of buffers
        .dma_buf_len = BLOCK_SIZE,                    // samples per buffer
        .use_apll = 1,
        .fixed_mclk = 124000000, //115200000, // 38400000, //124000000, ->38400000! //
    };

    // The pin config as per the setup
    //update: great - the mic is being clocked via the WS pin... 
    const i2s_pin_config_t pin_config = {
        .bck_io_num = -1, //14,   // Bit Clk
        .ws_io_num = 14, //14, //-1, //12,    // LR Clk
        .data_out_num = -1, // Data out
        .data_in_num = 32   // Data in
    };


    // Configuring the I2S driver and pins.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        printf(" ! Failed installing driver: %d\n", err);
        while (true);
    }

    // Alterations for SPH0645 to ensure we receive MSB correctly.
    //REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   // I2S_RX_SD_IN_DELAY
    //REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  // Phillips I2S - WS changes a cycle earlier

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        printf(" ! Failed setting pin: %d\n", err);
        while (true);
    }
    printf(" + I2S driver installed.\n");
}

//int32_t buffer[512];    // Effectively two 1024 byte buffers

//static void udp_server_task(void *pvParameters)
static void mic_record_task(void *pvParameters)
{
    esp_err_t readErr;
    uint8_t state = 0; 
    volatile uint16_t rPtr = 0; // Yes, I should be using a pointer. 
    size_t readOut;

    while(1){
        readErr = i2s_read(I2S_PORT,
            (char*)buffer + rPtr,
            BLOCK_SIZE,     // Number of bytes, 
            &readOut,       // stores bytes that were actually read
            portMAX_DELAY); // No timeout

        rPtr = rPtr + readOut;

        switch (state) {
            case 0: // wait for index to pass halfway
                if (rPtr > 2047) {
                    state = 1;
                }
                break;
            case 1: // send the first half of the buffer
                //printf("state %d\n", state);
                state = 2;
                bufReady = true;
                bufOffset = 0;

                //printf("tik\n");
                break;
            case 2: // wait for index to wrap
                //printf("state %d\n", state);
                if (rPtr < 2047) {
                    state = 3;
                }
                break;
            case 3: // send second half of the buffer
                //printf("state %d\n", state);
                state = 0;
                bufReady = true;
                bufOffset = 2048;
                //printf("tok\n");
                break;
            }

        // Wrap this when we get to the end of the buffer
        if (rPtr > 4091) rPtr = 0; //wait, why 2043? 4 bytes/32 bit away from end? 4096
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    //int i = 0;
    I2SSetup();
    //static uint8_t state = 0;
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (hasWifi) {
        //xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
        xTaskCreatePinnedToCore(udp_server_task, "udp_server", 8192, (void*)NULL, 2, NULL, 1);
    }
    
    // this seems to be necessary to finalize the setup. without the delay, the udp server gets stuck somewhere. 
    vTaskDelay(xDelay);

    if (UDPruns) {
        xTaskCreatePinnedToCore(mic_record_task, "mic_record", 8192, (void*) NULL, 5, NULL, 1);
    }

    //seems tpo work without this too... 
    //i had the feeling that without anything in the main loop, the two other tasks starve 
    //the watchdog. but this delay doesn't seem to have changed anything. 
    //
    //while(1) {
    //    vTaskDelay(xDelay);
    //}
}
