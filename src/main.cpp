#include <stdio.h>
#include "freertos/FreeRTOS.h" //Siempre se pone la primera si no da error
#include "driver/gpio.h"
#include "esp_log.h"     // para sustituir
#include "driver/ledc.h" //Para PWM
// #include "driver/adc.h" //no lo utilizo
#include <esp_adc/adc_oneshot.h>
#include <esp_timer.h>
#include "pid.h"

// standby driver motores
constexpr gpio_num_t MOTOR_STANDBY = GPIO_NUM_19;
constexpr gpio_num_t BOTON_ENCENDIDO = GPIO_NUM_25;

bool seguilineas_encendido = false;

// motor A
constexpr gpio_num_t MOTOR_AIN1 = GPIO_NUM_0; // SENTIDO MOTOR A
constexpr gpio_num_t MOTOR_AIN2 = GPIO_NUM_2; // SENTIDO MOTOR A
constexpr gpio_num_t MOTOR_PWMA = GPIO_NUM_4; // PWM MOTOR A

// motor B
constexpr gpio_num_t MOTOR_BIN1 = GPIO_NUM_14; // SENTIDO MOTOR B
constexpr gpio_num_t MOTOR_BIN2 = GPIO_NUM_12; // SENTIDO MOTOR B
constexpr gpio_num_t MOTOR_PWMB = GPIO_NUM_32; // PWM MOTOR B

int duty_motor_A = 0; // Duty motor
int duty_motor_B = 0; // Duty motor

esp_err_t oneshot_init(void);                          // Declaro el ADC
esp_err_t set_pwm(void);                               // declaro la funcion para el PWM con LEDC
esp_err_t set_pwm_duty(void);                          // Declaro la funcion DUTY del PWM
esp_err_t set_pio(void);                               // Declaro la funcion PIO
esp_err_t set_motor_adelante(void);                    // Motor hacia adelante
esp_err_t set_motor_velocidad(void);                   // Meto la velocidad al motor
esp_err_t set_motor_on(bool driver_motor);             // enciendo el driver del motor
esp_err_t leer_nuevo_sensor_infrarojos(int *sensores); // Lee el sensor infrarojos con un nueva funcion
esp_err_t coche_on_off(void);                          // Mira el pulsador y enciende el coche o lo apaga
float crea_error_sensor_infrarojos(int *sensores);     // Crea el error

// Configuro el ADC lo necesitamos para adc_oneshot_new_unit es para cada unit (canal)
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;

// Tema 11: PID
PID pid(100.0f, 7.0f, 60.0f);

uint64_t last_call = esp_timer_get_time(); // Lee el tiempo
uint64_t curr_time = esp_timer_get_time();
float result = 0;
float lastLineValue;

int adc[7]; // Declaro un array para leer los 8 sensores

// Como usamos C++ con esta declaración conseguimos que el void app_main() no cambie de nombre
extern "C" void app_main();
void app_main()
{
    set_pwm();      // Inicializo PWM
    set_pio();      // Inializo GPIO
    oneshot_init(); // Configura y Inializo el ADC

    set_pwm_duty();       // Pone la velocidad (DUTY) de los motores
    set_motor_adelante(); // El motor va hacia adelante

    while (1)
    {
        // Funcion para encender o apagar motores
        coche_on_off();

        // lectura de la linea negra
        leer_nuevo_sensor_infrarojos(adc); // LEO LOS INFRAROJOS con la nueva funcion

        // Crear el error para PID
        lastLineValue = crea_error_sensor_infrarojos(adc); // Da el error segun este en la linea

        //printf("lastLineValue = %.2f \n", lastLineValue);

        curr_time = esp_timer_get_time(); // Leo el tiempo para el PID
        result = pid.update(lastLineValue, curr_time - last_call);

        result = result / 100.0f;

        if (result > 700)
        {
            result = 700;
        }

        //printf("result  %.2f \n", result);

        last_call = curr_time;

        set_motor_velocidad(); // mando la velocidad a los motores Duty

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t coche_on_off(void)
{
    if (seguilineas_encendido)
    {
        set_motor_on(1); // Activo el driver del motor
    }
    else
    {
        set_motor_on(0); // Desactivo el driver del motor
    }

    // Miro haber si he pulsado el boton de encendido
        if (!gpio_get_level(BOTON_ENCENDIDO))
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (seguilineas_encendido)
            {
                seguilineas_encendido = false;
            }
            else
            {
                seguilineas_encendido = true;
            }
        }

    return ESP_OK;
}

esp_err_t set_motor_on(bool driver_motor) // enciendo el driver del motor
{
    if (driver_motor)
    {
        gpio_set_level(MOTOR_STANDBY, 1); // Activo el driver del motor
    }
    else
    {
        gpio_set_level(MOTOR_STANDBY, 0); // Desactivo el driver del motor
    }

    return ESP_OK;
}

esp_err_t set_motor_velocidad(void) // Meto la velocidad al motor
{
    duty_motor_A = (400 + (int)result);
    duty_motor_B = (400 - (int)result);

    if (duty_motor_A > 1023)
    {
        duty_motor_A = 1023;
    }
    if (duty_motor_B > 1023)
    {
        duty_motor_B = 1023;
    }
    if (duty_motor_A < 0)
    {
        duty_motor_A = 0;
    }
    if (duty_motor_B < 0)
    {
        duty_motor_B = 0;
    }

    set_pwm_duty(); // Pone el DUTY de los motores
    //printf("Motor A:%i  Motor B:%i \n", duty_motor_A, duty_motor_B);

    return ESP_OK;
}

esp_err_t set_motor_adelante(void) // El motor va hacia adelante
{
    gpio_set_level(MOTOR_AIN1, 0);
    gpio_set_level(MOTOR_AIN2, 1);
    gpio_set_level(MOTOR_BIN1, 1);
    gpio_set_level(MOTOR_BIN2, 0);

    return ESP_OK; // Devuelvo
}

float crea_error_sensor_infrarojos(int *sensores) // Crea el error de la linea
{
    int32_t acum = 0;
    int32_t weightened = 0;
    float linea = 0;
    if (sensores[0] > 2048)
    {
        weightened += ((float)sensores[0]) * (0 - (3.5f)) * 100.0f;
        acum += sensores[0];
    }

    if (sensores[1] > 2048)
    {
        weightened += ((float)sensores[1]) * (1 - (3.5f)) * 100.0f;
        acum += sensores[1];
    }

    if (sensores[2] > 2048)
    {
        weightened += ((float)sensores[2]) * (2 - (3.5f)) * 100.0f;
        acum += sensores[2];
    }

    if (sensores[3] > 2048)
    {
        weightened += ((float)sensores[3]) * (3 - (3.5f)) * 100.0f;
        acum += sensores[3];
    }

    if (sensores[4] > 2048)
    {
        weightened += ((float)sensores[4]) * (4 - (3.5f)) * 100.0f;
        acum += sensores[4];
    }

    if (sensores[5] > 2048)
    {
        weightened += ((float)sensores[5]) * (5 - (3.5f)) * 100.0f;
        acum += sensores[5];
    }

    if (sensores[6] > 2048)
    {
        weightened += ((float)sensores[6]) * (6 - (3.5f)) * 100.0f;
        acum += sensores[6];
    }

    if (sensores[7] > 2048)
    {
        weightened += ((float)sensores[7]) * (7 - (3.5f)) * 100.0f;
        acum += sensores[7];
    }

    if (acum != 0)
    {
        linea = (float)weightened / acum;
    }

    //printf("weightened = %.2f\n", (float)weightened);
    //printf("acum = %.2f\n", (float)acum);
    //printf("linea = %.2f\n", linea);

    return linea;
}

esp_err_t leer_nuevo_sensor_infrarojos(int *sensores) // Lee el sensor infrarojos con un nueva funcion
{
    // Leo los sensores
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &sensores[0]);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &sensores[1]);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &sensores[2]);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &sensores[3]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_9, &sensores[4]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &sensores[5]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_4, &sensores[6]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_3, &sensores[7]);

    return ESP_OK;
}

esp_err_t oneshot_init(void) // Configuro el ADC
{
    // Configuro ADC1 UNIT1
    adc_oneshot_unit_init_cfg_t adc_oneshot_unit_init = {
        // Declaro la estructura del ADC1
        .unit_id = ADC_UNIT_1,           // Utilizo el ADC1 que es UNIT1
        .ulp_mode = ADC_ULP_MODE_DISABLE // No utilizo el Ultra Low Power (ULP) coprocessor
    };

    // Para añadir la estructura creada y añadir mas datos
    adc_oneshot_new_unit(&adc_oneshot_unit_init, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12}; // Atenuación y resolución

    // Ahora inicializamos adc_oneshot_chan_cfg_t
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config); // Configuro D1 de UNIT1 GPIO35
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config); // Configuro D2 de UNIT1 GPIO34
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config); // Configuro D3 de UNIT1 GPIO33
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config); // Configuro D4 de UNIT1 GPIO36 SP

    // Configuro ADC2 UNIT2
    adc_oneshot_unit_init_cfg_t adc_oneshot_unit_init_2 = {
        // Declaro la estructura del ADC2
        .unit_id = ADC_UNIT_2,           // Utilizo el ADC2 que es UNIT2
        .ulp_mode = ADC_ULP_MODE_DISABLE // No utilizo el Ultra Low Power (ULP) coprocessor
    };

    adc_oneshot_new_unit(&adc_oneshot_unit_init_2, &adc2_handle); // Para añadir la estructura creada y añadir mas datos

    // Ahora inicializamos adc_oneshot_chan_cfg_t
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_9, &config); // Configuro D5  de UNIT2
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config); // Configuro D5  de UNIT2
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_4, &config); // Configuro D5  de UNIT2
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_3, &config); // Configuro D5  de UNIT2

    return ESP_OK; // Devuelvo
}

esp_err_t set_pwm(void) // Inicializacion del PWM con LEDC para los motores

{
    // motor A
    ledc_channel_config_t channelConfigMa = {}; // Creo el objeto DEL MOTOR A
    channelConfigMa.gpio_num = MOTOR_PWMA;
    channelConfigMa.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigMa.channel = LEDC_CHANNEL_3;
    channelConfigMa.intr_type = LEDC_INTR_DISABLE;
    channelConfigMa.timer_sel = LEDC_TIMER_0;
    channelConfigMa.duty = 0;

    // motor B
    ledc_channel_config_t channelConfigMb = {}; // Creo el objeto DEL MOTOR B
    channelConfigMb.gpio_num = MOTOR_PWMB;
    channelConfigMb.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigMb.channel = LEDC_CHANNEL_4;
    channelConfigMb.intr_type = LEDC_INTR_DISABLE;
    channelConfigMb.timer_sel = LEDC_TIMER_0;
    channelConfigMb.duty = 0;

    // GRABO LA CONFIGURACION
    ledc_channel_config(&channelConfigMa); // motor A
    ledc_channel_config(&channelConfigMb); // motor B

    // Creo el objeto TIMER para todos los canales
    ledc_timer_config_t timerconfig = {};
    timerconfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    timerconfig.duty_resolution = LEDC_TIMER_10_BIT; // Utilizo 10Bit de resolucción 1024
    timerconfig.timer_num = LEDC_TIMER_0;
    timerconfig.freq_hz = 1000; // trabajo con 1Khz

    ledc_timer_config(&timerconfig);

    return ESP_OK; // Devuelvo
}

esp_err_t set_pwm_duty(void) // Pongo el DUTY para los motores
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, duty_motor_A); // Cambio el duty MOTOR A
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, duty_motor_B); // Cambio el duty MOTOR B

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3); // Actualizo el duty motor A
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4); // Actualizo el duty motor B

    return ESP_OK; // Devuelvo
}

esp_err_t set_pio(void) // INICIALIZO LOS GPIO SALIDA ENTRADA
{
    // Configuro los GPIO para el MOTOR
    gpio_config_t config;
    config.mode = GPIO_MODE_OUTPUT;
    config.pin_bit_mask =
        (((uint64_t)1) << MOTOR_AIN1) |
        (((uint64_t)1) << MOTOR_AIN2) |
        (((uint64_t)1) << MOTOR_BIN1) |
        (((uint64_t)1) << MOTOR_BIN2) |
        (((uint64_t)1) << MOTOR_STANDBY);

    config.intr_type = GPIO_INTR_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&config);

    // Configuro el GPIO del boton de encendido
    config.intr_type = GPIO_INTR_DISABLE;
    config.mode = GPIO_MODE_INPUT;
    config.pin_bit_mask = (((uint64_t)1) << BOTON_ENCENDIDO);
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&config);

    return ESP_OK;
}
