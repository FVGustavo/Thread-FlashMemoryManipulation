#include <stdio.h>
#include <stdlib.h>
#include <esp_mac.h>

#include "rand_num.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "nvs_flash.h"

#define LED1_PIN GPIO_NUM_32
#define LED2_PIN GPIO_NUM_33
#define BTN1_PIN GPIO_NUM_14

#define QUEUE_SIZE 16

#define MIN_1 -400
#define MIN_2 -350
#define MAX_1 1200
#define MAX_2 960

BaseType_t th_gen_num_1, th_gen_num_2, th_read_num_1, th_read_num_2, th_grava_flash;
TaskHandle_t th_handle_1, th_handle_2, th_handle_3, th_handle_4, th_handle_5;
QueueHandle_t queue_handle_1, queue_handle_2, queue_handle_3;

void taskGenNum1(void);
void taskGenNum2(void);
void taskReadNum1(void);
void taskReadNum2(void);
void taskGravaFlash(void);
void taskReadFlash(void);

void taskGenNum1(void)
{
  int16_t ran_num_1;
  while (1)
  {
    ran_num_1 = genRanNum(-600, 1400);
    xQueueSend(queue_handle_1, &ran_num_1, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskGenNum2(void)
{
  int16_t ran_num_2;
  while (1)
  {
    ran_num_2 = genRanNum(-580, 1320);
    xQueueSend(queue_handle_2, &ran_num_2, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskReadNum1(void)
{
  int16_t received_num_1;

  while (1)
  {
    xQueueReceive(queue_handle_1, &received_num_1, portMAX_DELAY);

    if (received_num_1 < MIN_1 || received_num_1 > MAX_1)
    {
      gpio_set_level(LED1_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(1000));

      xQueueSend(queue_handle_3, &received_num_1, portMAX_DELAY);

      gpio_set_level(LED1_PIN, 1);
    }
  }
}

void taskReadNum2(void)
{
  int16_t received_num_2;

  while (1)
  {
    xQueueReceive(queue_handle_2, &received_num_2, portMAX_DELAY);

    if (received_num_2 < MIN_2 || received_num_2 > MAX_2)
    {
      gpio_set_level(LED2_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(1000));

      xQueueSend(queue_handle_3, &received_num_2, portMAX_DELAY);

      gpio_set_level(LED2_PIN, 1);
    }
  }
}

void taskGravaFlash(void)
{
  int16_t received_num;
  esp_err_t err;
  nvs_handle_t nvs_handle_1;
  int16_t index = 0;

  while (1)
  {
    xQueueReceive(queue_handle_3, &received_num, portMAX_DELAY);

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle_1);
    if (err != ESP_OK)
    {
      printf("Erro ao abrir flash \n");
      continue;
    }

    err = nvs_set_i16(nvs_handle_1, "num_array_index", index);
    if (err != ESP_OK)
    {
      printf("Erro ao gravar índice na flash \n");
    }

    char key[15];
    snprintf(key, sizeof(key), "num_%d", index);

    err = nvs_set_i16(nvs_handle_1, key, received_num);
    if (err != ESP_OK)
    {
      printf("Erro ao gravar na flash \n");
    }

    err = nvs_commit(nvs_handle_1);
    if (err != ESP_OK)
    {
      printf("Erro ao commitar na flash \n");
    }

    nvs_close(nvs_handle_1);

    index = (index + 1) % QUEUE_SIZE;
  }
}

void taskReadFlash(void)
{
  esp_err_t err;
  nvs_handle_t handle;
  int16_t stored_value;
  int16_t index = 0;

  err = nvs_open("storage", NVS_READWRITE, &handle);
  if (err != ESP_OK)
  {
    printf("Erro ao abrir flash \n");
    return;
  }

  err = nvs_get_i16(handle, "num_array_index", &index);
  if (err != ESP_OK)
  {
    printf("Erro ao ler índice da flash \n");
  }

  for (int i = 0; i < QUEUE_SIZE; i++)
  {
    char key[15];
    snprintf(key, sizeof(key), "num_%d", i);
    err = nvs_get_i16(handle, key, &stored_value);
    if (err == ESP_OK)
    {
      printf("Valores da flash [%d]: %hd \n", i, stored_value);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
      printf("Valor não encontrado para índice %d\n", i);
    }
    else
    {
      printf("Erro na leitura da flash para índice %d\n", i);
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }

  nvs_close(handle);
}

void app_main(void)
{
  esp_err_t ret = nvs_flash_init();
  if (ret != ESP_OK)
  {
    printf("Erro ao inicializar flash \n");
    return;
  }

  gpio_config_t led_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << LED1_PIN) | (1ULL << LED2_PIN),
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE};
  gpio_config(&led_conf);

  gpio_config_t btn_conf = {
      .intr_type = GPIO_INTR_NEGEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << BTN1_PIN),
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE};
  gpio_config(&btn_conf);

  queue_handle_1 = xQueueCreate(QUEUE_SIZE, sizeof(int16_t));
  if (queue_handle_1 == NULL)
  {
    printf("Erro ao criar fila 1 \n");
    return;
  }

  queue_handle_2 = xQueueCreate(QUEUE_SIZE, sizeof(int16_t));
  if (queue_handle_2 == NULL)
  {
    printf("Erro ao criar fila 2 \n");
    return;
  }

  queue_handle_3 = xQueueCreate(QUEUE_SIZE, sizeof(int16_t));
  if (queue_handle_3 == NULL)
  {
    printf("Erro ao criar fila 3 \n");
    return;
  }

  th_gen_num_1 = xTaskCreate(taskGenNum1, "thread_1", 2 * 1024, NULL, 5, &th_handle_1);
  if (th_gen_num_1 != pdPASS)
  {
    printf("Erro ao criar task 1 \n");
    return;
  }

  th_gen_num_2 = xTaskCreate(taskGenNum2, "thread_2", 2 * 1024, NULL, 5, &th_handle_2);
  if (th_gen_num_2 != pdPASS)
  {
    printf("Erro ao criar task 2 \n");
    return;
  }

  th_read_num_1 = xTaskCreate(taskReadNum1, "thread_3", 2 * 1024, NULL, 5, &th_handle_3);
  if (th_read_num_1 != pdPASS)
  {
    printf("Erro ao criar task 3 \n");
    return;
  }

  th_read_num_2 = xTaskCreate(taskReadNum2, "thread_4", 2 * 1024, NULL, 5, &th_handle_4);
  if (th_read_num_2 != pdPASS)
  {
    printf("Erro ao criar task 4 \n");
    return;
  }

  th_grava_flash = xTaskCreate(taskGravaFlash, "thread_5", 2 * 1024, NULL, 5, &th_handle_5);
  if (th_grava_flash != pdPASS)
  {
    printf("Erro ao criar task 5 \n");
    return;
  }

  while (1)
  {
    if (gpio_get_level(BTN1_PIN) == 0)
    {
      taskReadFlash();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
