/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "driver/ledc.h"
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>

#include <app_wifi.h>

#include "app_priv.h"

static const char *TAG = "app_main";


#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define LEDC_LS_CH0_GPIO       (1)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (2)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)
/* Callback to handle commands received from the RainMaker cloud */
int ch=0;
void led_setup()
{
	     /*
	      * Prepare and set configuration of timers
	      * that will be used by LED Controller
	      */
	     ledc_timer_config_t ledc_timer = {
	         .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
	         .freq_hz = 5000,                      // frequency of PWM signal
	         .speed_mode = LEDC_LS_MODE,           // timer mode
	         .timer_num = LEDC_LS_TIMER,            // timer index
	         .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
	     };
	     // Set configuration of timer0 for high speed channels
	     ledc_timer_config(&ledc_timer);

	     ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {

	             {
	                 .channel    = LEDC_LS_CH0_CHANNEL,
	                 .duty       = 0,
	                 .gpio_num   = LEDC_LS_CH0_GPIO,
	                 .speed_mode = LEDC_LS_MODE,
	                 .hpoint     = 0,
	                 .timer_sel  = LEDC_LS_TIMER
	             },
	             {
	                 .channel    = LEDC_LS_CH1_CHANNEL,
	                 .duty       = 0,
	                 .gpio_num   = LEDC_LS_CH1_GPIO,
	                 .speed_mode = LEDC_LS_MODE,
	                 .hpoint     = 0,
	                 .timer_sel  = LEDC_LS_TIMER
	             },

	             {
	                 .channel    = LEDC_LS_CH2_CHANNEL,
	                 .duty       = 0,
	                 .gpio_num   = LEDC_LS_CH2_GPIO,
	                 .speed_mode = LEDC_LS_MODE,
	                 .hpoint     = 0,
	                 .timer_sel  = LEDC_LS_TIMER
	             },
	             {
	                 .channel    = LEDC_LS_CH3_CHANNEL,
	                 .duty       = 0,
	                 .gpio_num   = LEDC_LS_CH3_GPIO,
	                 .speed_mode = LEDC_LS_MODE,
	                 .hpoint     = 0,
	                 .timer_sel  = LEDC_LS_TIMER
	             },
	         };

	     for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
	            ledc_channel_config(&ledc_channel[ch]);
	        }

	        // Initialize fade service.
	        ledc_fade_func_install(0);
/*
	        ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, bright*100);
	       	        ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	       	        vTaskDelay(1000 / portTICK_PERIOD_MS);
*/
}


static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
/*
    if (app_driver_set_gpio(esp_rmaker_param_get_name(param), val.val.i) == ESP_OK) {
        esp_rmaker_param_update_and_report(param, val);
    }
*/

    const char *device_name = esp_rmaker_device_get_name(device);
      const char *param_name = esp_rmaker_param_get_name(param);
      if (strcmp(param_name, "brightness2") == 0) {
    	  ESP_LOGI(TAG, "Received value = %d for %s - %s",
    	                    val.val.i, device_name, param_name);
    	  int bright2=val.val.i*100<8000?val.val.i*100:8000;
    	            ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, bright2);
    	            	       	        ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);
    	            	       	        vTaskDelay(1000 / portTICK_PERIOD_MS);         // app_driver_set_gpio(param_name, val.val.b);
      }  if (strcmp(param_name, "brightness1") == 0) {
          ESP_LOGI(TAG, "Received value = %d for %s - %s",
                  val.val.i, device_name, param_name);
          int bright1=val.val.i*100<8000?val.val.i*100:8000;
          ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, bright1);
          	       	        ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);
          	       	        vTaskDelay(1000 / portTICK_PERIOD_MS);

      }
          else {
              /* Silently ignoring invalid params */
              return ESP_OK;
          }
          esp_rmaker_param_update_and_report(param, val);
          return ESP_OK;
}


void app_main()
{
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_init()
     */
    app_wifi_init();
    
   led_setup();


    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_wifi_init() but before app_wifi_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "ESP32-S2-Saola-1");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create a device and add the relevant parameters to it */
    esp_rmaker_device_t *gpio_device = esp_rmaker_device_create("ESP32-S2-Saola-1", NULL, NULL);
    esp_rmaker_device_add_cb(gpio_device, write_cb, NULL);

    /*esp_rmaker_param_t *red_param = esp_rmaker_param_create("Red", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(red_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_device_add_param(gpio_device, red_param);
	*/

    esp_rmaker_param_t *red_param = esp_rmaker_param_create("brightness1", NULL, esp_rmaker_int(100), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(red_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(red_param, esp_rmaker_int(0), esp_rmaker_int(100), esp_rmaker_int(1));
    esp_rmaker_device_add_param(gpio_device, red_param);

    esp_rmaker_param_t *green_param = esp_rmaker_param_create("brightness2", NULL, esp_rmaker_int(100), PROP_FLAG_READ | PROP_FLAG_WRITE);
      esp_rmaker_param_add_ui_type(green_param, ESP_RMAKER_UI_SLIDER);
      esp_rmaker_param_add_bounds(green_param, esp_rmaker_int(0), esp_rmaker_int(100), esp_rmaker_int(1));
      esp_rmaker_device_add_param(gpio_device, green_param);

    /*esp_rmaker_param_t *green_param = esp_rmaker_param_create("Green", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(green_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(gpio_device, green_param);

    esp_rmaker_param_t *blue_param = esp_rmaker_param_create("Blue", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(blue_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(gpio_device, blue_param);
*/
    esp_rmaker_node_add_device(node, gpio_device);

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    app_wifi_start();
}
