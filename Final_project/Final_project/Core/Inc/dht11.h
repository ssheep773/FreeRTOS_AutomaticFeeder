/*
 * dht11.h
 *
 *  Created on: Jun 20, 2024
 *      Author: ssheep
 */

#ifndef DHT_H_
#define DHT_H_

#include "main.h"

typedef struct {
float hum;
float temp;
} DHT_data;

typedef enum {
DHT11
} DHT_type;

#define DHT_Port GPIOB
#define DHT_Pin GPIO_PIN_0
#define DHT_PullUp 0
#define DHT_timeout 10000

DHT_data DHT_getData(DHT_type t);

#endif
