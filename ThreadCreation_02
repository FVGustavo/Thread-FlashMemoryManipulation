#include <stdio.h>
#include <stdlib.h>
#include <esp_mac.h>

#include "rand_num.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define LED1_PIN GPIO_NUM_32
#define LED2_PIN GPIO_NUM_33

#define QUEUE_SIZE 16

BaseType_t th_gen_num_1, th_gen_num_2, th_read_num_1, th_read_num_2;
TaskHandle_t th_handle_1, th_handle_2, th_handle_3, th_handle_4;
QueueHandle_t queue_1, queue_2;

void taskGenNum1(void);
void taskGenNum2(void);
void taskReadNum1(void);
void taskReadNum2(void);

void taskGenNum1(void)
{
  int16_t ran_num_1;
  while (1)
  {
    ran_num_1 = genRanNum(-350, 1260);
    xQueueSend(queue_1, &ran_num_1, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskGenNum2(void)
{
  int16_t ran_num_2;
  while (1)
  {
    ran_num_2 = genRanNum(-630, 890);
    xQueueSend(queue_2, &ran_num_2, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskReadNum1(void)
{
  int16_t received_num_1;
  while (1)
  {
    xQueueReceive(queue_1, &received_num_1, portMAX_DELAY);
    if (received_num_1 < -200 || received_num_1 > 1000)
    {
      gpio_set_level(LED1_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(1000));
      gpio_set_level(LED1_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    gpio_set_level(LED1_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskReadNum2(void)
{
  int16_t received_num_2;
  while (1)
  {
    xQueueReceive(queue_2, &received_num_2, portMAX_DELAY);
    if (received_num_2 < -400 || received_num_2 > 700)
    {
      gpio_set_level(LED2_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(1000));
      gpio_set_level(LED2_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    gpio_set_level(LED2_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
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

  queue_1 = xQueueCreate(QUEUE_SIZE, sizeof(int16_t));
  queue_2 = xQueueCreate(QUEUE_SIZE, sizeof(int16_t));

  if (queue_1 == NULL)
  {
    printf("Erro ao criar fila 1");
  }

  if (queue_2 == NULL)
  {
    printf("Erro ao criar fila 2");
  }

  th_gen_num_1 = xTaskCreate((TaskFunction_t)taskGenNum1, "thread 1", 2 * 1024, NULL, 5, &th_handle_1);

  th_gen_num_1 = xTaskCreate((TaskFunction_t)taskGenNum2, "thread 2", 2 * 1024, NULL, 5, &th_handle_2);

  th_read_num_1 = xTaskCreate((TaskFunction_t)taskReadNum1, "thread 3", 2 * 1024, NULL, 5, &th_handle_3);

  th_read_num_2 = xTaskCreate((TaskFunction_t)taskReadNum2, "thread 4", 2 * 1024, NULL, 5, &th_handle_4);

  if (th_gen_num_1 != pdPASS)
  {
    printf("Erro ao criar thread 1");
    return;
  }

  if (th_gen_num_2 != pdPASS)
  {
    printf("Erro ao criar thread 2");
    return;
  }
  if (th_read_num_1 != pdPASS)
  {
    printf("Erro ao criar thread 3");
    return;
  }
  if (th_read_num_2 != pdPASS)
  {
    printf("Erro ao criar thread 4");
    return;
  }

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
