/*
 * htdu21d.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: postal
 */

// write a publisher in collect function (maybe join hum and temp meaurement)?

//TODO check all the headers
#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/humidity.h>

#include <board_config.h>
#include <drivers/drv_humidity.h>

#define HTU21D_I2CADDR       0x40 // 0x40
#define HTU21D_READTEMP      0xE3 //e3
#define HTU21D_READHUM       0xE5 //e5
#define HTU21D_WRITEREG      0xE6
#define HTU21D_READREG       0xE7
#define HTU21D_RESET         0xFE

#define HTU21D_BUS 		PX4_I2C_BUS_EXPANSION
//#define HTU21D_CONVERSION_INTERVAL	(1000000 / 10)	/* microseconds */

//TODO make crc_handler

class HTU21D : public device::I2C
{
public:
	HTU21D(int bus = HTU21D_BUS, int address = HTU21D_I2CADDR);
	virtual ~HTU21D();

	virtual int 		init();
	int                 reset();
	int                 get_humidity();
	virtual int         get_temperature();  // TODO check virtual - _retries

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);
	/**
		* Static trampoline from the workq context; because we don't have a
		* generic workq wrapper yet.
		*
		* @param arg		Instance pointer for the driver that is polling.
		*/
	static void			cycle_trampoline(void *arg);


	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

//protected:
//	virtual int			probe() = 0;

private:
	float				    _humidity;
	float                   _temperature;
	work_s				    _work;
	bool				    _sensor_ok;
	int					    _measure_ticks;
	bool				    _collect_phase;
	int				        _class_instance;
	int				        _orb_class_instance;
	ringbuffer::RingBuffer	*_reports;

	struct humidity_s report;
	orb_advert_t _humidity_pub;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int					_cycling_rate;	/* */
	uint8_t				_index_counter;	/* temporary sonar i2c address */
	std::vector<uint8_t>	addr_ind; 	/* temp sonar i2c address vector */
	std::vector<float>	_latest_sonar_measurements; /* vector to store latest sonar measurements in before writing to report */


	int					collect();
	int                 measure();
	void                cycle();

	void                start();
	void                stop();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int htu21d_main(int argc, char *argv[]);

HTU21D::HTU21D(int bus, int address) :
	I2C("HTU21D", HTU21D_DEVICE_PATH, bus, address, 100000),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "mb12xx_read")),
	_comms_errors(perf_alloc(PC_COUNT, "mb12xx_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "mb12xx_buffer_overflows")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0) 	/* initialising temp sonar i2c address to zero */

{
	printf("\n\nHTU21D constructor starts ...\n\n"); //DELETE

	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

HTU21D::~HTU21D()
{
	/* make sure we are truly inactive */
	//stop();              // TODO check the func()

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
HTU21D::init()
{
	printf("\nHTU21D Initialization starts ...\n"); //DELETE

	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		errx(1, "init::I2C::init() != OK");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(humidity_s));
	if (_reports == nullptr)
	{
		ret = ERROR;
		return ret;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(HTU21D_DEVICE_PATH);

	/* publication init */
	if (_class_instance == CLASS_DEVICE_PRIMARY) {

		/* advertise sensor topic, measure manually to initialize valid report */
		struct humidity_s arp;
		measure();
		_reports->get(&arp);

		/* measurement will have generated a report, publish */
		_humidity_pub = orb_advertise(ORB_ID(humidity), &arp);

		if (_humidity_pub == nullptr)
			warnx("uORB started?");
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

int
HTU21D::reset()
{
	printf("\nReseting HTU21D ...\n"); //DELETE

	int ret;
	uint8_t test = 0;
	//printf("\nTest value before= %hhu\n", &test); //DELETE
	warnx("Test value before = %u", test); //DELETE

	/*
	 * Send the command to begin a measurement.
	 */

	uint8_t cmd = HTU21D_READREG;

	ret = transfer(&cmd, 1, nullptr, 0);
	if (OK != ret) {
		errx(1, "measure::OK != ret");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

	usleep(25000);

	ret = transfer(nullptr, 0, &test, 1);

	debug("My ransfer returned %d", ret);
	if (OK != ret) {
		errx(1, "measure::OK != ret on return");
		perf_count(_comms_errors);
		//debug("i2c::transfer returned %u", ret);
		return ret;
	}

	//printf("\nTest value = %hhu\n", &test); //DELETE
	warnx("Test value after = %u", test); //DELETE

	ret = OK;

	return ret;
}

int
HTU21D::get_humidity()
{
	int	ret = -EIO;
	uint8_t cmd = HTU21D_READHUM, crc = 0, val[3] = {0, 0, 0};
	uint16_t hum = 0;

	ret = transfer(&cmd, 1, nullptr, 0);
	if (OK != ret) {
		errx(1, "get_humidity::OK != transfer");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

	usleep(5000); //TODO check how usleep func work

	ret = transfer(nullptr, 0, &val[0], 3);
	if (OK != ret) {
		errx(1, "get_humidity::OK != receiving");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

    hum = (val[0] << 8) + val[1];	//TODO check the expression
    crc = val[2];
    debug("i2c::transfer returned %d", crc); //DELETE

    // Following constants are taken from the htu21d datasheet
    _humidity = (hum * 125) / 65536 - 6;
    warnx("Humidity = %f", _humidity); //DELETE

	return ret;
}

int
HTU21D::get_temperature()
{
	int	ret = -EIO;
	uint8_t cmd = HTU21D_READTEMP, crc = 0, val[3] = {0, 0, 0};
	uint16_t tem = 0;

	//_retries = 10;

	ret = transfer(&cmd, 1, nullptr, 0);
	if (OK != ret) {
		errx(1, "get_temperature::OK != send");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

	usleep(60000); //TODO check how usleep func work

	ret = transfer(nullptr, 0, &val[0], 3);
	if (OK != ret) {
		errx(1, "get_temperature::OK != receive");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

    tem = (val[0] << 8) + val[1];	//TODO check the expression
    crc = val[2];
    debug("i2c::transfer returned %d", crc); //DELETE

    // Following constants are taken from the htu21d datasheet
    _temperature = (tem * 175.72) / 65536 - 46.85;
    warnx("Temperature = %f", _temperature); //DELETE

	return ret;
}

int HTU21D::measure()     //TODO check if the code (::cycle) works with this function
{
	int ret;

	uint8_t cmd = HTU21D_READREG;

	ret = transfer(&cmd, 1, nullptr, 0);
	if (OK != ret) {
		errx(1, "measure::OK != ret");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}
	ret = OK;

	return ret;
}

int
HTU21D::collect()
{
	int ret = ERROR;
	printf("\nHTU21D collection starts ...\n"); //DELETE

	if (get_humidity() != OK) {
			errx(1, "get_humidity() != OK");
			return ret;
		}

	if (get_temperature() != OK) {
				errx(1, "get_temperature() != OK");
				return ret;
			}

	report.timestamp = hrt_absolute_time();
	report.humidity_percent = _humidity;
	report.hum_temperature_celsius = _temperature;

	if (_humidity_pub != nullptr && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(humidity), _humidity_pub, &report);

	}

	//TODO make a proper publishing continuance, check the examples

	printf("\nHTU21D collection ends ...\n"); //DELETE
	ret = OK;
	return ret;
}

void
HTU21D::print_info()
{
    perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
}

void HTU21D::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			debug("collection error");
			/* restart the measurement state machine */
			start(); //TODO find out how _reports work and implement it everywhere
			_sensor_ok = false;
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		if (_measure_ticks > USEC2TICK(HTU21D_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&HTU21D::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(HTU21D_CONVERSION_INTERVAL));

			return;
		}
	}


	/* measurement phase */
	if (OK != measure())
		debug("measure error");

	//_sensor_ok = (ret == OK); TODO fix

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
			&_work,
			(worker_t)&HTU21D::cycle_trampoline,
			this,
			USEC2TICK(HTU21D_CONVERSION_INTERVAL));

}

void
HTU21D::cycle_trampoline(void *arg)
{
	HTU21D *dev = (HTU21D *)arg;

	dev->cycle();
}

void
HTU21D::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HTU21D::cycle_trampoline, this, 1);
}

void
HTU21D::stop()
{
	work_cancel(HPWORK, &_work);
}

/**
 * Local functions in support of the shell command.
 */
namespace htu21d
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

HTU21D	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new HTU21D(HTU21D_BUS);

	if (g_dev == nullptr) {
		errx(1, "g_dev == nullptr");
		goto fail;
	}

	if (OK != g_dev->init()) {
		errx(1, "start::OK != g_dev->init()");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(HTU21D_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "fd < 0");
		goto fail;
	}



//	usleep(50000); //TODO check how usleep func work
//	for (int i = 0; i < 100; i++){
//		usleep(10000); //TODO check how usleep func work
//		g_dev->get_temperature();
//		usleep(10000); //TODO check how usleep func work
//		g_dev->get_humidity();
//	}
//
//
//	g_dev->get_temperature();
	//g_dev->cycle();

	//TODO: check, do I need it or not
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		errx(1, "SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT");
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct hum_report report;                              //to-do check it later
	ssize_t sz;
	int ret;

	int fd = open(HTU21D_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'htu21d start' if the driver is not running", HTU21D_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("time:        %llu", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("time:        %llu", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(HTU21D_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */

int htu21d_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		htu21d::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		htu21d::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		htu21d::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		htu21d::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		htu21d::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}

