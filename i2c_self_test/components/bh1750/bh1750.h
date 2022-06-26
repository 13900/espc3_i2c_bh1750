
#ifndef BH1750_H
#define BH1750_H
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_5           /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_4           /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0                       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define BH1750_SLAVE_ADDR   0x23 // 从机地址
#define BH1750_PWR_DOWN     0x00 // 关闭模块
#define BH1750_PWR_ON       0x01 // 打开模块等待测量指令
#define BH1750_RST          0x07 // 重置数据寄存器值在PowerOn模式下有效
#define BH1750_CON_H        0x10 // 连续高分辨率模式，1lx，120ms
#define BH1750_CON_H2       0x11 // 连续高分辨率模式，0.5lx，120ms
#define BH1750_CON_L        0x13 // 连续低分辨率模式，4lx，16ms
#define BH1750_ONE_H        0x20 // 一次高分辨率模式，1lx，120ms，测量后模块转到PowerDown模式
#define BH1750_ONE_H2       0x21 // 一次高分辨率模式，0.5lx，120ms，测量后模块转到PowerDown模式
#define BH1750_ONE_L        0x23 // 一次低分辨率模式，4lx，16ms，测量后模块转到PowerDown模式
#define print_mux xSemaphoreCreateMutex()
#define xSemaphoreGive(xSemaphore) xQueueGenericSend( ( QueueHandle_t ) ( xSemaphore ), NULL, semGIVE_BLOCK_TIME, queueSEND_TO_BACK )
TaskHandle_t BH1750_TASK_HANDEL;
int I2C_Init(void);
int I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);
int I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);
void BH1750_Init(void);
float BH1750_ReadLightIntensity(void);
void i2c_test_task(void *arg);
#endif
