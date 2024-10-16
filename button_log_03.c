#include <stdio.h>
#include <stdlib.h>
#include <esp_mac.h>

#include "rand_num.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "esp_err.h"

#include "nvs_flash.h"

#define LED1_PIN GPIO_NUM_32
#define LED2_PIN GPIO_NUM_33
#define BTN1_PIN GPIO_NUM_14

#define QUEUE_SIZE 16

BaseType_t th_gen_num_1, th_gen_num_2, th_read_num_1, th_read_num_2, th_read_nvs;
TaskHandle_t th_handle_1, th_handle_2, th_handle_3, th_handle_4;
QueueHandle_t queue_1, queue_2;
SemaphoreHandle_t mutex;

void taskReadNVS(void);

void taskGenNum1(void);
void taskGenNum2(void);
void taskReadNum1(void *);
void taskReadNum2(void *);

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

void taskReadNum1(void *pvParameter)
{
  int16_t received_num_1;
  esp_err_t err;
  nvs_handle_t nvs_handle_1;

  while (1)
  {
    xQueueReceive(queue_1, &received_num_1, portMAX_DELAY);

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle_1);

    if (err != ESP_OK)
    {
      printf("Erro ao tentar abrir flash \n");
      return;
    }
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      if (received_num_1 < -200 || received_num_1 > 1000)
      {
        gpio_set_level(LED1_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        err = nvs_set_i32(nvs_handle_1, "th_read_num_1", received_num_1);

        if (err != ESP_OK)
        {
          printf("Erro ao gravar \n");
        }

        err = nvs_commit(nvs_handle_1);

        if (err != ESP_OK)
        {
          printf("Erro ao dar commit \n");
        }

        gpio_set_level(LED1_PIN, 1);
      }
      xSemaphoreGive(mutex);
    }
    else
    {
      printf("Erro ao pegar mutex \n");
    }
    nvs_close(nvs_handle_1);
  }
}

void taskReadNum2(void *pvParameter)
{
  int16_t received_num_2;
  esp_err_t err;
  nvs_handle_t nvs_handle_2;

  while (1)
  {
    xQueueReceive(queue_2, &received_num_2, portMAX_DELAY);

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle_2);

    if (err != ESP_OK)
    {
      printf("Erro ao tentar abrir flash \n");
      return;
    }

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      if (received_num_2 < -400 || received_num_2 > 700)
      {
        gpio_set_level(LED2_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        err = nvs_set_i32(nvs_handle_2, "th_read_num_2", received_num_2);

        if (err != ESP_OK)
        {
          printf("Erro ao gravar \n");
        }

        err = nvs_commit(nvs_handle_2);

        if (err != ESP_OK)
        {
          printf("Erro ao dar commit \n");
        }

        gpio_set_level(LED2_PIN, 1);
      }
      xSemaphoreGive(mutex);
    }
    else
    {
      printf("Erro ao pegar mutex \n");
    }
    nvs_close(nvs_handle_2);
  }
}

void taskReadNVS(void)
{
  esp_err_t err;
  nvs_handle_t handle;
  int32_t stored_value_1 = 0;
  int32_t stored_value_2 = 0;

  err = nvs_open("storage", NVS_READWRITE, &handle);
  if (err != ESP_OK)
  {
    printf("Erro ao abrir a flash \n");
    return;
  }

  err = nvs_get_i32(handle, "th_read_num_1", &stored_value_1);
  if (err == ESP_ERR_NVS_NOT_FOUND)
  {
    printf("Valor th_read_num_1 ainda não foi inicializado \n");
  }
  else if (err == ESP_OK)
  {
    printf("Valor gravado pela task 1: %ld \n", stored_value_1);
  }
  else
  {
    printf("Erro na leitura de th_read_num_1 \n");
  }

  err = nvs_get_i32(handle, "th_read_num_2", &stored_value_2);
  if (err == ESP_ERR_NVS_NOT_FOUND)
  {
    printf("Valor th_read_num_2 ainda não foi inicializado \n");
  }
  else if (err == ESP_OK)
  {
    printf("Valor gravado pela task 2: %ld \n", stored_value_2);
  }
  else
  {
    printf("Erro na leitura de th_read_num_2 \n");
  }

  nvs_close(handle);
}

void app_main(void)
{
  esp_err_t ret = nvs_flash_init();

  gpio_config_t led_conf;
  led_conf.intr_type = GPIO_INTR_DISABLE;
  led_conf.mode = GPIO_MODE_OUTPUT;
  led_conf.pin_bit_mask = (1ULL << LED1_PIN) | (1ULL << LED2_PIN);
  led_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  led_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpio_config(&led_conf);

  gpio_config_t btn_conf;
  btn_conf.intr_type = GPIO_INTR_NEGEDGE;
  btn_conf.mode = GPIO_MODE_INPUT;
  btn_conf.pin_bit_mask = (1ULL << BTN1_PIN);
  btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&btn_conf);

  mutex = xSemaphoreCreateMutex();

  if (mutex == NULL)
  {
    printf("Erro ao criar mutex \n");
    return;
  }

  queue_1 = xQueueCreate(QUEUE_SIZE, sizeof(int16_t));
  if (queue_1 == NULL)
  {
    printf("Erro ao criar fila 1 \n");
    return;
  }

  queue_2 = xQueueCreate(QUEUE_SIZE, sizeof(int16_t));
  if (queue_2 == NULL)
  {
    printf("Erro ao criar fila 2 \n");
    return;
  }

  th_gen_num_1 = xTaskCreate((TaskFunction_t)taskGenNum1, "thread 1", 2 * 1024, NULL, 5, &th_handle_1);
  if (th_gen_num_1 != pdPASS)
  {
    printf("Erro ao criar thread 1 \n");
    return;
  }

  th_gen_num_2 = xTaskCreate((TaskFunction_t)taskGenNum2, "thread 2", 2 * 1024, NULL, 5, &th_handle_2);
  if (th_gen_num_2 != pdPASS)
  {
    printf("Erro ao criar thread 2 \n");
    return;
  }

  th_read_num_1 = xTaskCreate((TaskFunction_t)taskReadNum1, "thread 3", 2 * 1024, NULL, 5, &th_handle_3);
  if (th_read_num_1 != pdPASS)
  {
    printf("Erro ao criar thread 3 \n");
    return;
  }

  th_read_num_2 = xTaskCreate((TaskFunction_t)taskReadNum2, "thread 4", 2 * 1024, NULL, 5, &th_handle_4);
  if (th_read_num_2 != pdPASS)
  {
    printf("Erro ao criar thread 4 \n");
    return;
  }

  while (1)
  {
    if (gpio_get_level(BTN1_PIN) == 0)
    {
      taskReadNVS();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
