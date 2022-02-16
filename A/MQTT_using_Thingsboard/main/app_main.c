#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "esp_adc_cal.h"
#include "math.h"

#include "freertos/timers.h"

/////////////////////////////////////////////////////////////
#include "driver/i2c.h"
#include "driver/adc.h"


static const char *TAG_I2C = "i2c-test";

#define DATA_LENGTH 32              /*!< Data buffer length of test buffer */

#define I2C_MASTER_SCL_IO 25        //34 - mtero express    9 - feather esp32-s2
#define I2C_MASTER_SDA_IO 26       //33 - metro express   8 - feather esp32-s2
#define I2C_MASTER_NUM 1            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 5
#define SLAVE_ADDR 0x04 /*!< Slave address*/
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
//////////////////////////////////////////////////////////////adc
TimerHandle_t adc_timer;
uint16_t average=2;
uint32_t adc_val = 0;
uint16_t trigger=10;
uint16_t count_average=0;
//////////////////////////////////////////////////////////////
#define topics_sub "v1/devices/me/rpc/request/+"
#define topics_pub "v1/devices/me/telemetry"
#define broker "mqtt://uphXuis4LCohvfC2bcmo:NULL@demo.thingsboard.io:1883"

char Datapub[64];
char Datapub_[64];

char DataSub[128];

uint8_t data[32] = {0};



static const char *TAG = "MQTT_Thingsboard";

esp_mqtt_client_config_t mqtt_cfg = {
			        .uri = broker,
			    };
static esp_mqtt_client_handle_t client;
uint8_t Lamp1 =-1;
uint16_t value_p1;
uint16_t last_value_p1;
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, topics_sub, 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:

        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        strncat(DataSub,(event->data),event->data_len);
        printf("DataSub= %s\n",DataSub);
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_Subscribe(void){
	client = esp_mqtt_client_init(&mqtt_cfg);
	 esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);
}
void read_string(char *s){

	////////////////

	if(strstr(s,"button1")!=NULL){
		if(strstr(s,"true")!=NULL){
			printf("button1 ON\n");
			Lamp1 = 1;
		}else if(strstr(s,"false")!=NULL){
			printf("button1 OFF\n");
			Lamp1 = 0;
		}
	}
	////////////////

        if (strstr(s,"potentiometer1")!=NULL) {
            char* arg;
            char* arg_;
            char* cmd = strtok(s, ":");////////// {"method":"potentiometer1","params":52}

        	if (cmd != NULL) {
            arg = strtok(NULL, ":"); // first after command led
            arg_ = strtok(NULL, "}"); // second after command led
             value_p1 = atoi(arg_);
			printf("value %d \n",value_p1);

        }
    }

}
void Thingsboard(void *pvParameter){

	while(1){
		read_string(DataSub);
		memset(DataSub, 0, sizeof Datapub);
		if (Lamp1 == 1){
			ESP_LOGI(TAG, "lamp1 ON");
			Lamp1 = -1;

		      float T_ = 1;
		      char buf[10];
		      sprintf(buf,"%f",T_);
		      strcpy(Datapub_,"{\"lamp1\":");
		      strcat(Datapub_,buf);
		      strcat(Datapub_,"}");
		      esp_mqtt_client_publish(client,topics_pub,Datapub_, 0, 1, 0);
		      data[0] = 1;


		}else if(Lamp1 == 0){
			ESP_LOGI(TAG, "lamp1 OFF");
			Lamp1 = -1;
		      float T_ = 0;
		      char buf[10];
		      sprintf(buf,"%f",T_);
		      strcpy(Datapub_,"{\"lamp1\":");
		      strcat(Datapub_,buf);
		      strcat(Datapub_,"}");
		      esp_mqtt_client_publish(client,topics_pub,Datapub_, 0, 1, 0);
		      data[0] = 0;

		}
		if(value_p1 != last_value_p1){
					ESP_LOGI(TAG, "motor1");
					last_value_p1 = value_p1;
					float T_ = value_p1;
				      char buf[10];
				      sprintf(buf,"%f",T_);
				      strcpy(Datapub_,"{\"motor1\":");
				      strcat(Datapub_,buf);
				      strcat(Datapub_,"}");
				      esp_mqtt_client_publish(client,topics_pub,Datapub_, 0, 1, 0);
				  	data[1] = value_p1;

				}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
//////////////////////////////////////////////i2c
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t write_I2C(i2c_port_t i2c_num, uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    err = i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    err = i2c_master_write(cmd, data, size, ACK_CHECK_EN);
    err = i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}
static void i2c_master_task(void *arg)
{
    while (1)
    {
        write_I2C(I2C_MASTER_NUM, data, DATA_LENGTH);
        vTaskDelay(50 / portTICK_RATE_MS);
    }
    free(data);
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG_I2C, "I2C unitialized successfully");
}

///////////////////////////////////////////////////////adc
void adc_callback(TimerHandle_t xTimer){
	xTimerChangePeriod(xTimer, pdMS_TO_TICKS(trigger), 0);
	if(count_average < average){
		int val = adc1_get_raw(ADC1_CHANNEL_7);
//		ESP_LOGI("adc_value ","%d :: count_number = %d\n", val,count_average+1);

		count_average++;
		adc_val = adc_val+val;
		if(count_average == average){
//			ESP_LOGI("total of adc values", "%d\n", adc_val);
//			ESP_LOGI("average_number", "%d\n", count_average);
			ESP_LOGI("adc_average", "%d\n", adc_val/average);
			data[2]=(adc_val*255)/(average*4095);
			count_average=0;
			adc_val=0;

		}
	}
}
//////////////////////////////////////////////////
void app_main(void)
{

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG_I2C, "I2C initialized successfully");
    /* Create adc */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);
	//esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    mqtt_Subscribe();
    xTaskCreatePinnedToCore(&Thingsboard,"Control The LED", 2048, NULL, 1,NULL, 1);

    xTaskCreate(&i2c_master_task, "i2c_master_task", 2048, NULL, 2, NULL);

    adc_timer = xTimerCreate("adc Timer",
            pdMS_TO_TICKS(100),
            pdTRUE,
            NULL,
            adc_callback);
    /* Start timer */
    xTimerStart(adc_timer, 0);
}


