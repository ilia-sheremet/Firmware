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

#define HTU21D_CONVERSION_INTERVAL 	100000 /* 60ms for one sonar */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	100000 /* 30ms between each sonar measurement (watch out for interference!) */


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


struct hum_report {
	uint64_t timestamp;
	uint64_t error_count;
	float				_humidity;
};


#endif /* DRV_HUMIDITY_H_ */
