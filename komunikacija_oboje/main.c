#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"

#define TAG "APP"

#define UART_BMS UART_NUM_2
#define UART_BAC UART_NUM_1

#define RX_BAC355 16
#define TX_BAC355 17

#define TX_BMS 9
#define RX_BMS 10

#define SLAVE_ID  0x01
#define REG_COUNT 5              //0x0004
#define START_ADDR 259           //0x0104

#define JK_CELL_COUNT 16
#define JK_REG_CHARGING     0xAB
#define JK_REG_DISCHARGING  0xAC

typedef struct {
    float total_voltage;
    float current;
    uint8_t soc;
    float temperature1;
    float temperature2;

    float cell_voltage[JK_CELL_COUNT];

    bool charging_enabled;
    bool discharging_enabled;
} jk_bms_data_t;

static jk_bms_data_t jk_data;

static const uint8_t jk_read_all_cmd[] = {
    0x4E, 0x57, 0x00, 0x13,
    0x00, 0x00, 0x00, 0x00,
    0x06, 0x03,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x68,
    0x00, 0x00,
    0x01, 0x29
};

static uint16_t jk_sum(const uint8_t *data, int len)
{
    uint16_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

static uint16_t u16_be(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static int jk_find_header(const uint8_t *buf, int len)
{
    for (int i = 0; i < len - 1; i++) {
        if (buf[i] == 0x4E && buf[i + 1] == 0x57) {
            return i;
        }
    }
    return -1;
}

static bool jk_validate_frame(const uint8_t *f, int len)
{
    if (len < 20) {
        ESP_LOGW(TAG, "Frame too short");
        return false;
    }

    if (f[0] != 0x4E || f[1] != 0x57) {
        ESP_LOGW(TAG, "Wrong header");
        return false;
    }

    uint16_t recv_sum = u16_be(&f[len - 2]);
    uint16_t calc_sum = jk_sum(f, len - 2);

    if (recv_sum != calc_sum) {
        ESP_LOGW(TAG,
                 "Checksum mismatch recv=0x%04X calc=0x%04X len=%d",
                 recv_sum,
                 calc_sum,
                 len);
        return false;
    }

    return true;
}

static void jk_parse_frame(const uint8_t *f, int len, jk_bms_data_t *out)
{
    if (!jk_validate_frame(f, len)) {
        return;
    }

    memset(out, 0, sizeof(*out));

    int i = 11;

    while (i < len - 4) {
        uint8_t tag = f[i];

        switch (tag) {
            case 0x79: {
                uint8_t data_len = f[i + 1];
                uint8_t cells = data_len / 3;

                if (cells > JK_CELL_COUNT) {
                    cells = JK_CELL_COUNT;
                }

                for (int c = 0; c < cells; c++) {
                    uint8_t cell_num = f[i + 2 + c * 3];
                    uint16_t mv = u16_be(&f[i + 3 + c * 3]);

                    if (cell_num >= 1 && cell_num <= JK_CELL_COUNT) {
                        out->cell_voltage[cell_num - 1] = mv / 1000.0f;
                    }
                }

                i += 2 + data_len;
                break;
            }

            case 0x82:
                uint16_t temp = u16_be(&f[i + 1]);
                out->temperature1 = (temp <= 100) ? temp : -(temp - 100);
                i += 3;
                break;

            case 0x83:
                out->total_voltage = u16_be(&f[i + 1]) / 100.0f;
                i += 3;
                break;

            case 0x84: {
                uint16_t raw = u16_be(&f[i + 1]);

                bool charging = (raw & 0x8000) != 0;
                uint16_t value = raw & 0x7FFF;

                if (charging) {
                    out->current = value * 0.01f;     // charging = positive
                } else {
                    out->current = -(value * 0.01f);  // discharging = negative
                }

            ESP_LOGI(TAG,
             "Raw current=0x%04X value=%u direction=%s current=%.2f A",
             raw,
             value,
             charging ? "charging" : "discharging",
             out->current);

            i += 3;
            break;
}

            case 0x85:
                out->soc = f[i + 1];
                i += 2;
                break;

            case 0x8C:
                uint16_t status = u16_be(&f[i + 1]);

                out->charging_enabled    = (status & 0x0001) != 0;
                out->discharging_enabled = (status & 0x0002) != 0;

                i += 3;
                break;

            default:
                i++;
                break;
        }
    }

    ESP_LOGI(TAG,
             "Voltage=%.2f V | Current=%.2f A | SOC=%u%% | CHG=%s | DSG=%s | Temperature= %.0f",
             out->total_voltage,
             out->current,
             out->soc,
             out->charging_enabled ? "ON" : "OFF",
             out->discharging_enabled ? "ON" : "OFF",
             out->temperature1);

    for (int c = 0; c < JK_CELL_COUNT; c++) {
        ESP_LOGI(TAG, "Cell %02d = %.3f V", c + 1, out->cell_voltage[c]);
    }
}

static void jk_send_read_request(void)
{
    uart_write_bytes(UART_BMS,
                     (const char *)jk_read_all_cmd,
                     sizeof(jk_read_all_cmd));
}

static void jk_write_register(uint8_t reg, bool enable)
{
    uint8_t frame[20] = {
        0x4E, 0x57,
        0x00, 0x14,
        0x00, 0x00, 0x00, 0x00,
        0x02,
        reg,
        enable ? 0x01 : 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x68,
        0x00, 0x00,
        0x00, 0x00
    };

    uint16_t sum = jk_sum(frame, 18);

    frame[18] = (sum >> 8) & 0xFF;
    frame[19] = sum & 0xFF;

    uart_write_bytes(UART_BMS, (const char *)frame, sizeof(frame));
}

void jk_enable_charging(bool enable)
{
    jk_write_register(JK_REG_CHARGING, enable);
}

void jk_enable_discharging(bool enable)
{
    jk_write_register(JK_REG_DISCHARGING, enable);
}

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //UART BMS
    uart_driver_install(UART_BMS, 4096, 0, 0, NULL, 0);
    uart_param_config(UART_BMS, &cfg);
    uart_set_pin(UART_BMS,
                 TX_BMS,
                 RX_BMS,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    
    uart_flush_input(UART_BMS);

    //UART BAC355
    uart_driver_install(UART_BAC, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_BAC, &cfg);
    uart_set_pin(UART_BAC, TX_BAC355, RX_BAC355, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_flush_input(UART_BAC);
}

void jk_task(void *arg)
{
    uint8_t rx[512];

    while (1) {
        uart_flush_input(UART_BMS);

        jk_send_read_request();

        int len = uart_read_bytes(UART_BMS,
                                  rx,
                                  sizeof(rx),
                                  pdMS_TO_TICKS(300));

        if (len > 0) {
            int start = jk_find_header(rx, len);

            if (start >= 0) {
                jk_parse_frame(&rx[start], len - start, &jk_data);
            } else {
                ESP_LOGW(TAG, "No JK header found");
            }
        } else {
            ESP_LOGW(TAG, "No BMS response");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//BAC 355 FUNKCIJE:
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
    uint16_t controller_temp = (rx[3] << 8) | rx[4];      //                         register 259
    uint16_t speed_raw        = (rx[5] << 8) | rx[6];     //0x[high byte][lowbyte] , register 260
    uint16_t temp             = (rx[7] << 8) | rx[8];     //                         register 261
    uint16_t motor_current_raw    = (rx[9] << 8) | rx[10];     //                         register 262
    uint16_t rpm              = (rx[11] << 8) | rx[12];    //                         register 263

    float speed = speed_raw / 256.0f;
    float motor_current = motor_current_raw / 32.0f; 

    ESP_LOGI(TAG,
             "Controller Temperatura: %d | Speed: %.2f | RPM: %d | Tok motorja: %.2f ",
             controller_temp, speed, rpm, motor_current);
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
            ESP_LOGW(TAG, "No BAC355 response");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void)
{
    uart_init();

    ESP_LOGI(TAG, "APP started");

    xTaskCreate(jk_task,
                "jk_task",
                8192,
                NULL,
                5,
                NULL);

    xTaskCreate(bac_task, "bac_task", 4096, NULL, 4, NULL);
    /*
       Examples:

       Disable charging:
       jk_enable_charging(false);

       Enable charging:
       jk_enable_charging(true);

       Disable discharging:
       jk_enable_discharging(false);

       Enable discharging:
       jk_enable_discharging(true);
    */
}