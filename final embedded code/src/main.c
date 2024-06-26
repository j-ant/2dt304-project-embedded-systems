#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lwip/apps/mqtt.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "lwip/etharp.h"
#include "lwip/apps/mqtt_priv.h"
#include "bme680.h"
#include "bme680_defs.h"

#define I2C_PORT i2c0
#define I2C_SDA  0
#define I2C_SCL  1
#define BROKER_ADDRESS "192.168.0.208"
#define BROKER_PORT     1883
#define MQTT_TOPIC      "sensor/data"
#define SENSOR_DATA_FORMAT "{\"Temperature\":%.2f,\"Pressure\":%.2f,\"Humidity\":%.2f,\"Gas\":%lu}"

static bool mqttConnected = false;


typedef struct {
    mqtt_client_t *client;
    ip_addr_t mqttBrokerIPAddr;
} mqtt_client_info_t;

static mqtt_client_info_t mqttClientInfo;

typedef struct MQTT_CLIENT_T_ {
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

static struct netif netif;
mqtt_client_t* mqtt_client;
ip_addr_t mqtt_broker_ip_addr;
struct bme680_dev sensor;



void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connected\n");
        mqttConnected = true;
    } else {
        printf("MQTT connection failed\n");
        mqttConnected = false;
    }
}

void connect_to_mqtt_broker(void) {
    printf("Connecting to MQTT broker at %s...\n", BROKER_ADDRESS);
    mqttClientInfo.client = mqtt_client_new();
    struct mqtt_connect_client_info_t ci = {
        .client_id = "pico_w",
        .keep_alive = 60,
    };

    ip_addr_t mqttBrokerIP;
    IP_ADDR4(&mqttBrokerIP, 172,20,10,4);

    mqtt_client_connect(mqttClientInfo.client, &mqttBrokerIP, BROKER_PORT, mqtt_connection_cb, NULL, &ci);
}


int getSensorDataAsJson(char* buffer, size_t bufferSize) {
    struct bme680_field_data data;
    if (bme680_set_sensor_mode(&sensor) == BME680_OK) {
        
        sensor.delay_ms(70);
        if (bme680_get_sensor_data(&data, &sensor) == BME680_OK) {
            return snprintf(buffer, bufferSize, SENSOR_DATA_FORMAT,
                            data.temperature / 100.0f, data.pressure / 100.0f,
                            data.humidity / 1000.0f, data.gas_resistance);
        }
    }
    return 0;
}

void publish_sensor_data(void) {
    char messageBuffer[256];
    int messageLength = getSensorDataAsJson(messageBuffer, sizeof(messageBuffer));
    if (messageLength > 0) {

        mqtt_publish(mqttClientInfo.client, MQTT_TOPIC, messageBuffer, messageLength, 0, 0, NULL, NULL);
        printf("Published: %s\n", messageBuffer);
    } else {
        printf("Failed to get sensor data.\n");
    }
}

void mqtt_publish_task(void * pvParameters) {
    char messageBuffer[256];
    while (1) {
        publish_sensor_data();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


void init_bme(void) {
    sensor.tph_sett.os_hum = BME680_OS_2X;
    sensor.tph_sett.os_pres = BME680_OS_4X;
    sensor.tph_sett.os_temp = BME680_OS_8X;
    sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
    sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    sensor.gas_sett.heatr_temp = 320;
    sensor.gas_sett.heatr_dur = 150;
    sensor.power_mode = BME680_FORCED_MODE; 

    uint8_t set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;
    bme680_set_sensor_settings(set_required_settings, &sensor);
}

void user_delay_ms(uint32_t period) {
    sleep_ms(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    i2c_write_blocking(I2C_PORT, dev_id, &reg_addr, 1, true);
    if (i2c_read_blocking(I2C_PORT, dev_id, data, len, false) != len) return BME680_E_COM_FAIL;
    return BME680_OK;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    for (uint16_t i = 0; i < len; i++) buf[i + 1] = data[i];
    if (i2c_write_blocking(I2C_PORT, dev_id, buf, len + 1, false) != len + 1) return BME680_E_COM_FAIL;
    return BME680_OK;
}


void setup() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
    }

    cyw43_arch_enable_sta_mode();

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
    sensor.intf = BME680_I2C_INTF;
    sensor.read = user_i2c_read;
    sensor.write = user_i2c_write;
    sensor.delay_ms = user_delay_ms;
    sensor.amb_temp = 25;

    if (bme680_init(&sensor) == BME680_OK) {
        init_bme();
    } else {
        printf("Sensor initialization failed\n");
    }
}


void bme680_task() {
    struct bme680_field_data data;
    while (1) {
        if (bme680_set_sensor_mode(&sensor) == BME680_OK) {
            sensor.delay_ms(70);
            if (bme680_get_sensor_data(&data, &sensor) == BME680_OK) {
                printf("Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f %%, Gas: %d ohms\n",
                    data.temperature / 100.0f, data.pressure / 100.0f, data.humidity / 1000.0f, data.gas_resistance);
            } else {
                printf("Failed to get sensor data\n");
            }
        } else {
            printf("Failed to set sensor mode\n");
        }
        vTaskDelay(2000);
    }
}

// A task to test LED blink, uncomment the task creation on main() to use
void GreenLEDTask(void *param) {
  while (1)
  {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    vTaskDelay(500);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    vTaskDelay(500);
    printf("HELLO WORLD\n");
  }
}

void connect_wifi_task() {
    while (1)
    {
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
            printf("Connection Failed\n");
        } else {
            printf("Successfully Connected\n");
            if (!mqttConnected) {
                connect_to_mqtt_broker();
            }
        }
        vTaskDelay(2000);
    }
}

int main() {
    setup();

    TaskHandle_t readData = NULL;
    xTaskCreate(bme680_task, "BME680 Task", 1024, NULL, 2, &readData);

    TaskHandle_t publishData = NULL;
    xTaskCreate(mqtt_publish_task, "Publish Task", 2048, NULL, 1, &publishData);

    TaskHandle_t wifiTask = NULL;
    xTaskCreate(connect_wifi_task, "WiFi Task", 1024, NULL, 3, &wifiTask);


    // Uncomment to test pico LED blink
    // TaskHandle_t LEDTask = NULL;
    // xTaskCreate(GreenLEDTask, "Led Task", 1024, NULL, 4, &LEDTask);

    vTaskStartScheduler();
    return 0;
}
