#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include <string.h>
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "APP";

#define RX_BAC355 47
#define TX_BAC355 48

#define RX_BMS 20
#define TX_BMS 21

#define UART_BAC UART_NUM_1
#define UART_BMS UART_NUM_2

#define SLAVE_ID  0x01
#define REG_COUNT 4              //0x0004
#define START_ADDR 260           //0x0104
// MODBUS FUNKCIJA 

uint16_t modbusCRC(const uint8_t* data, size_t len){ //data je kazalec za posamezni Byte frame-a
    uint16_t crc = 0xFFFF; // Začetna vrednost za protokola 0xFFFF, polinom pri protokolu je 0xA001

    for (size_t i = 0; i < len; i++){
        crc ^= data[i]; // Prvo XOR z prvim byte-om  
        for (int b = 0; b < 8; b++){
        if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001; //če LSB pri koncu prejšne operacije 1 ---> Shift right za 1 bit, pa XOR z polinomom
        else crc >>= 1;   //če je LSB pri koncu prejšne operacije 0 ---> Shift right za 1 bit
        }
    }

    return crc;
}

//Inicijalizacija UART
static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // -------- BAC355 UART --------
    uart_driver_install(UART_BAC, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_BAC, &cfg);
    uart_set_pin(UART_BAC, TX_BAC355, RX_BAC355, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_flush_input(UART_BAC);
    // -------- BMS UART --------
    uart_driver_install(UART_BMS, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_BMS, &cfg);
    uart_set_pin(UART_BMS, TX_BMS, RX_BMS, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// MODBUS request master to slave
void sendreq(void){
    uint8_t req[8];

    req[0] = SLAVE_ID;                  //SLAVE ID
    req[1] = 0x03;                      //READ registers funkcija
    req[2] = (START_ADDR >> 8) & 0xFF;  //Start Address 260 HIGH Byte, 0xFF se uporabi da je to 1 Byte (varno)
    req[3] = START_ADDR & 0xFF;         //Start Address 260 LOW Byte
    req[4] = 0;                         //stevilo registri HIGH Byte
    req[5] = REG_COUNT;                 //stevilo registri LOW Byte

    uint16_t crc = modbusCRC(req, 6);
    req[6] = crc & 0xFF;                //CRC Low Byte
    req[7] = (crc >> 8) & 0xFF;         //CRC High Byte


    uart_write_bytes(UART_BAC, (const char*)req, sizeof(req));

}

// READ response
int read_response(uint8_t *buf, int max_len)
{
    return uart_read_bytes(UART_BAC, buf, max_len, 300 / portTICK_PERIOD_MS);

}

// Razčlenitev
void parse_response(uint8_t *rx, int len)
{
    if (len < (3 + REG_COUNT * 2 + 2)) {
        ESP_LOGW(TAG, "Frame prekrajši");
        return;
    }

    if (rx[0] != SLAVE_ID) {
        ESP_LOGW(TAG, "Narobe slave ID");
        return;
    }

    if (rx[1] == 0x83) {
        ESP_LOGW(TAG, "Modbus exception");
        return;
    }

    if (rx[1] != 0x03) {
        ESP_LOGW(TAG, "Wrong function code");
        return;
    }

    if (rx[2] != REG_COUNT * 2) {
        ESP_LOGW(TAG, "Byte count mismatch");
        return;
    }

    uint16_t crcCalc = modbusCRC(rx, len - 2);        // CRC izračunan
    uint16_t crcRecv = rx[len - 2] | (rx[len - 1] << 8);  // CRC sprejet
    if (crcCalc != crcRecv) {
        ESP_LOGW(TAG, "Narobe CRC");
        return;
    }
  

    // raw registri
    uint16_t speed_raw = (rx[3] << 8) | rx[4];     //0x[high byte][lowbyte] , register 260
    uint16_t temp      = (rx[5] << 8) | rx[6];     //                         register 261
    uint16_t reg262    = (rx[7] << 8) | rx[8];     //                         register 262
    uint16_t rpm       = (rx[9] << 8) | rx[10];    //                         register 263

    float speed = speed_raw / 256.0f;

    ESP_LOGI(TAG,
             "Speed: %.2f | Temp: %d | RPM: %d",
             speed, temp, rpm);
}


//---- BAC355 task
void bac_task(void *arg)
{
    uint8_t rx[128];

    while (1) {

        // small Modbus silence gap
        vTaskDelay(pdMS_TO_TICKS(5));

        sendreq();

        int len = read_response(rx, sizeof(rx));

        if (len > 0) {
            parse_response(rx, len);
        } else {
            ESP_LOGW(TAG, "No response");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    uart_init();

    ESP_LOGI(TAG, "BAC355 Modbus start");

    xTaskCreate(bac_task,
                "bac_task",
                4096,
                NULL,
                5,
                NULL);
}

