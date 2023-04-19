/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "math.h"

/***************************** PIN Definitions  ******************************/

#define DEBOUNCE_DELAY_MS   (50)
#define READ_TEMP_DELAY_MS  (100)

#define UART_NUM            UART_NUM_0
#define UART_BUFF_SIZE      (1024)
#define UART_TX_PIN         (1)
#define UART_RX_PIN         (3)
#define UART_MAX_PACKET_LEN (50)

#define OUTPUT_CONTROL       (2)   // Output control

#define LED_R_PIN            (25)  // LED Red 
#define LED_B_PIN            (26)  // LED Blue  
#define LED_G_PIN            (27)  // LED Green 

#define RES_0                (1000) // Thermistor Value at 25°C
#define BETA                 (3300) // Thermistor BETA parameter
#define ADC_BITS             (4095) // 12-bit ADC resolution

#define THERM_ADC_CH         (ADC1_CHANNEL_0) // ADC1 channel 0 is GPIO0 */

#define BTN_ADD_PIN          (32)  // add button
#define BTN_SUB_PIN          (33)  // subtract button

#define MIN_TEMP    0
#define MAX_TEMP    99

#define STEP_UP 5
#define STEP_DOWN 5

#define NUMBER_SAMPLES 10

/********************* Global variable declaration  ******************************/

bool btn_sum_pressed = false;
bool btn_down_pressed = false;

// copy these commands to docklight exclude ("") e.g (#GET_RANGE_TEMP)
const char *cmd_set_temp  = "#SET_TEMP";

const char *cmd_set_hist  = "#SET_HIST";

int glob_desired_temp = 10;
int glob_cautin_temp;
uint8_t glob_histeresis = 10;

// Function definitions

void uart_task(void *args);
void temp_task (void *args);
void cautin_task(void *args);

// Callbacks

void button_add_isr_handler(void *args)
{
    static int last_time = 0;
    int current_time = xTaskGetTickCountFromISR();

    // Check if the ISR is triggered within the debouncing delay
    if ((current_time - last_time) < pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
        // Ignore the ISR if it's triggered within the debouncing delay
        return;
    }

    // Process button press event here
    if(glob_desired_temp < (MAX_TEMP - STEP_UP)) glob_desired_temp += STEP_UP;

    // Update the last ISR time
    last_time = current_time;
}

void button_sub_isr_handler(void *args)
{
    static int last_time = 0;
    int current_time = xTaskGetTickCountFromISR();

    // Check if the ISR is triggered within the debouncing delay
    if ((current_time - last_time) < pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
        // Ignore the ISR if it's triggered within the debouncing delay
        return;
    }

    // Process button press event here
    if(glob_desired_temp > (MIN_TEMP + STEP_DOWN)) glob_desired_temp -= STEP_DOWN;

    // Update the last ISR time
    last_time = current_time;
}


// Init peripherals
static QueueHandle_t uart_queue = NULL; 

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUFF_SIZE, UART_BUFF_SIZE, 100, &uart_queue, 0);
}

void init_ADC(void)
{    
    adc1_config_channel_atten(THERM_ADC_CH, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
}

void init_button(void)
{
    const gpio_config_t btn_gpio_config =
        {
            .pin_bit_mask = (1ULL << BTN_ADD_PIN) | (1ULL << BTN_SUB_PIN),
            .mode = GPIO_MODE_DEF_INPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .intr_type = GPIO_INTR_NEGEDGE};

    gpio_config(&btn_gpio_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_ADD_PIN, button_add_isr_handler, NULL);
    gpio_isr_handler_add(BTN_SUB_PIN, button_sub_isr_handler, NULL);
}

void led_init(void)            
{
    gpio_reset_pin(OUTPUT_CONTROL);
    gpio_set_direction(OUTPUT_CONTROL, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_R_PIN);
    gpio_set_direction(LED_R_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_G_PIN);
    gpio_set_direction(LED_G_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_B_PIN);
    gpio_set_direction(LED_B_PIN, GPIO_MODE_OUTPUT);
}

///////////////////

void process_packet(char *data, int data_len )
{
    printf("bytes received  = [%d]\r\n", data_len);
    printf("data = %.*s\r\n", data_len, data);

    if(!memcmp(data, cmd_set_temp, strlen(cmd_set_temp)))
    {
        printf("cmd detected -> [%s]\r\n", cmd_set_temp);
        // print temperature range
                // get value in packet
        int value = 0;
        if (sscanf(data, "#SET_TEMP$%d$", &value) == 1){ // #SET_TEMP$40$
            printf("The integer value is: %d\n", value);

            if(value >= MIN_TEMP && value <= MAX_TEMP )
            {
                glob_desired_temp = value;
                printf("Temperature desired set to: %d", glob_desired_temp);
            }

        }
        else {
            printf("No integer value found in the string.\n");
        }
    }
    else if (!memcmp(data, cmd_set_hist, strlen(cmd_set_hist)))
    {
        printf("cmd detected -> [%s]\r\n", cmd_set_hist);
        // print temperature threshold
                // get value in packet
        int value = 0;
        if (sscanf(data, "#SET_HIST$%d$", &value) == 1){ // #SET_HIST$10$
            printf("The integer value is: %d\n", value);

            if(value > 0)
            {
                glob_histeresis = value;
                printf("Histeresis set to: %d\n", glob_histeresis);
            }
        }
        else {
            printf("No integer value found in the string.\n");
        }
    }


}


/*********************** LED  related Functions ******************************/


void led_on(gpio_num_t gpio)
{
    gpio_set_level(gpio, 1);
}

void led_off(gpio_num_t gpio)
{
    gpio_set_level(gpio, 0);
}

void rgb_led_on(gpio_num_t gpio)
{
    switch (gpio)
    {
    case LED_R_PIN: led_on(LED_R_PIN); led_off(LED_G_PIN); led_off(LED_B_PIN); break;
    case LED_G_PIN: led_on(LED_G_PIN); led_off(LED_R_PIN); led_off(LED_B_PIN); break;
    case LED_B_PIN: led_on(LED_B_PIN); led_off(LED_G_PIN); led_off(LED_R_PIN); break;
    
    default:
        break;
    }
}

void rgb_led_off(gpio_num_t gpio)
{
    led_off(gpio);
}

float get_thermistor_temperature()
{
    float temp = 0.0;
    int raw = adc1_get_raw(THERM_ADC_CH);
    float adc_val = raw / ADC_BITS;
    float RES_1 = adc_val * RES_0 / (1 - adc_val);
    temp = 1.0 / (1.0 / 298.15 + 1.0 / BETA * (log(RES_1 / RES_0))) - 273.15;
    return temp;
}

///////

void print_startup_info(void)
{
    printf("Project Name : Examen\r\n");
    printf("Student : Juan Sebastián Bedoya\r\n");
    printf("Student : Ángel Ocampo García\r\n");
}

void app_main(void)
{
    print_startup_info();

    // Task #1 (UART)
    uart_init();
    xTaskCreate(uart_task,"uart_task", 1024*2, NULL, 5, NULL);

    // Task #2 (ADC)
    init_ADC();
    xTaskCreate(temp_task,"temp_task", 1024*2, NULL, 6, NULL);

    // Task #3 (BUTTONS & LEDS)
    printf("LEDs & BUTTONs INIT\r\n");
    init_button();
    led_init();

    // Task #4 (CAUTIN)
    xTaskCreate(cautin_task,"cautin_task", 1024*2, NULL, 6, NULL);

}

/********************* uart Task ******************************/

void uart_task(void *args)
{

    printf("UART INIT\r\n");

    // init peripherals
    char uart_data[UART_MAX_PACKET_LEN];
    uart_event_t uart_event;

    while (1)
    {
        // uart data events 
        if (xQueueReceive(uart_queue, (void *)&uart_event, pdMS_TO_TICKS(200)))
        { 
            bzero(uart_data, UART_MAX_PACKET_LEN);
            
            switch (uart_event.type)
            { 
                case UART_DATA:
                {
                    uart_read_bytes(UART_NUM, uart_data, uart_event.size, pdMS_TO_TICKS(100));
                    process_packet(uart_data, uart_event.size);
                } break;

                default : {                 
                } break;
            }
        }
    }
}

/********************* Temp Task ******************************/
void temp_task (void *args)
{  

    printf("ADC INIT\r\n");

    while (1)
    { 
        glob_cautin_temp = 0;

        for (size_t count = 0; count < NUMBER_SAMPLES; count++)
        {
            glob_cautin_temp += get_thermistor_temperature();
            vTaskDelay(pdMS_TO_TICKS(READ_TEMP_DELAY_MS));
        }
        
        printf("temperature = %.3d\r\n", glob_cautin_temp/NUMBER_SAMPLES);
    }
    
}

/********************** Cautin Task *****************************/

void cautin_task(void *args)
{
    while(1)
    {
        if(glob_cautin_temp < glob_desired_temp - glob_histeresis) // Turn ON
        {
            rgb_led_on(LED_B_PIN);
            printf("Temperature Control ON - LED BLUE ON\r\n");
            gpio_set_level(OUTPUT_CONTROL, 1);

        }else if(glob_cautin_temp > glob_desired_temp + glob_histeresis) // Turn OFF
        {
            rgb_led_on(LED_R_PIN);
            printf("Temperature Control OFF - LED RED ON\r\n");
            gpio_set_level(OUTPUT_CONTROL, 0);
        }
        else // In Range
        {
            rgb_led_on(LED_G_PIN);
            printf("Temperature in Histeresis Range - LED GREEN ON\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
