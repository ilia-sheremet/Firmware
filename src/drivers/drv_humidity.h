/*
 * drv_humidity.h
 *
 *  Created on: Jul 9, 2015
 *      Author: postal
 */

#ifndef DRV_HUMIDITY_H_
#define DRV_HUMIDITY_H_

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define HTU21D_DEVICE_PATH	"/dev/htu21d"

struct hum_report {
	uint64_t timestamp;
	uint64_t error_count;
	float				_humidity;
};


#endif /* DRV_HUMIDITY_H_ */
