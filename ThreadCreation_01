#include <stdio.h>
#include <stdlib.h>
#include <esp_mac.h>

#include "rand_num.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1_PIN GPIO_NUM_32
#define LED2_PIN GPIO_NUM_33

BaseType_t th_gen_num, th_read_num;
TaskHandle_t th_handle_1, th_handle_2;

void taskGenNum(void);
void taskReadNum(void);

void taskGenNum(void)
{
  int16_t ran_num;
  while (1)
  {
    ran_num = genRanNum(-350, 1260);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void taskReadNum(void)
{
}

void app_main(void)
{
  gpio_config_t led_conf;
  led_conf.intr_type = GPIO_INTR_DISABLE;                          // Habilita interrupção na queda
  led_conf.mode = GPIO_MODE_OUTPUT;                                // Configura o pino como saída
  led_conf.pin_bit_mask = (1ULL << LED1_PIN) | (1ULL << LED2_PIN); // Seleciona os pinos GPIO
  led_conf.pull_up_en = GPIO_PULLUP_DISABLE;                       // Desabilita pull up
  led_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;                    // Habilita pull down
  gpio_config(&led_conf);

  th_gen_num = xTaskCreate((TaskFunction_t)taskGenNum,
                           "thread 1",
                           2 * 1024,
                           NULL,
                           5,
                           &th_handle_1);

  th_read_num = xTaskCreate((TaskFunction_t)taskGenNum,
                            "thread 2",
                            2 * 1024,
                            NULL,
                            5,
                            &th_handle_2);

  if (th_gen_num != pdPASS)
  {
    printf("Erro ao criar thread 1");
    return;
  }
  else
  {
    printf("Thread 1 criada com sucesso");
    return;
  }

  if (th_read_num != pdPASS)
  {
    printf("Erro ao criar thread 2");
    return;
  }
  else
  {
    printf("Thread 2 criada com sucesso");
    return;
  }

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
