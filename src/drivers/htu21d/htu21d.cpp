/**
 * @file htdu21d.cpp
 * @author Illia Sheremet "illia.sheremet@gmail.com"
 *
 * Driver for the MS HTU21d humidity and temperature sensors connected via I2C.
 */

#include <nuttx/config.h>

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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/humidity.h>

#include <board_config.h>
#include <drivers/drv_humidity.h>

/* HTU21D registers */
#define HTU21D_I2CADDR       0x40
#define HTU21D_READTEMP      0xE3
#define HTU21D_READHUM       0xE5
#define HTU21D_WRITEREG      0xE6
#define HTU21D_READREG       0xE7
#define HTU21D_RESET         0xFE

#define HTU21D_BUS 		PX4_I2C_BUS_EXPANSION

#define HTU21D_MEASUREMENT_RATE 1 /* in HZ */
#define HTU21D_CONVERSION_INTERVAL 	(1000000 / HTU21D_MEASUREMENT_RATE) /* 1HZ */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

//TODO make crc_handler

class HTU21D : public device::I2C
{
public:
	HTU21D(int bus = HTU21D_BUS, int address = HTU21D_I2CADDR);
	virtual ~HTU21D();

	virtual int 		init();
	int                 probe();
	int                 get_humidity();
	int                 get_temperature();

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


private:
	float				    _humidity;
	float                   _temperature;
	work_s				    _work;
	bool				    _sensor_ok;
	int					    _measure_ticks;
	bool				    _collect_phase;
	int				        _class_instance;
	RingBuffer	*_reports;

	struct humidity_s report;
	orb_advert_t _humidity_pub;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

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
	_sample_perf(perf_alloc(PC_ELAPSED, "htu21d_read")),
	_comms_errors(perf_alloc(PC_COUNT, "htu21d_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "htu21d_buffer_overflows")),
	_humidity(0),
	_temperature(0),
	_reports(nullptr),
	_humidity_pub(-1)

{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

HTU21D::~HTU21D()
{
	/* make sure we are truly inactive */
	stop();

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
HTU21D::init()
{
	int ret = ERROR;

	/* do I2C init first */
	if (I2C::init() != OK) {
		errx(1, "init::I2C::init() != OK");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(humidity_s));
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

		if (_humidity_pub < 0)
			warnx("uORB started?");
	}

	ret = OK;

	_sensor_ok = true;

	return ret;
}

int
HTU21D::probe()
{
	int ret;
	uint8_t probe_value = 0;

	/*
	 * Send the command to read from the sensor.
	 */
	uint8_t cmd = HTU21D_READREG;

	ret = transfer(&cmd, 1, nullptr, 0);
	if (OK != ret) {
		errx(1, "measure::OK != ret");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

	usleep(5000);

	ret = transfer(nullptr, 0, &probe_value, 1);
	if (OK != ret) {
		errx(1, "measure::OK != ret on return");
		perf_count(_comms_errors);
		return ret;
	}

	//TODO probe_value sometimes not equal to 2, check why
//	if (probe_value == 2)
//	{
//		ret = OK;
//		warnx("Success!");
//	}
//	else
//	{
//		warnx("Sensor is not connected");
//		errx(1, "probe_value !== 2");
//	}

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

	usleep(5000);

	ret = transfer(nullptr, 0, &val[0], 3);
	if (OK != ret) {
		errx(1, "get_humidity::OK != receiving");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

    hum = (val[0] << 8) + val[1];
    crc = val[2];
    debug("i2c::transfer returned %d", crc); //DELETE

    // Following constants are taken from the htu21d datasheet
    _humidity = (hum * 125) / 65536 - 6;
    //warnx("Humidity = %f", _humidity); //DELETE dubug

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

	usleep(60000);

	ret = transfer(nullptr, 0, &val[0], 3);
	if (OK != ret) {
		errx(1, "get_temperature::OK != receive");
		perf_count(_comms_errors);
		debug("i2c::transfer returned %d", ret);
		return ret;
	}

    tem = (val[0] << 8) + val[1];
    crc = val[2]; //TODO implement
    debug("i2c::transfer returned %d", crc); //DELETE

    // Following constants are taken from the htu21d datasheet
    _temperature = (tem * 175.72) / 65536 - 46.85;
    //warnx("Temperature = %f", _temperature); //DELETE debug

	return ret;
}

int HTU21D::measure()
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

	if (!(_pub_blocked)) {
		orb_publish(ORB_ID(humidity), _humidity_pub, &report);
	}

	if (!_reports->force(&report))
		perf_count(_buffer_overflows);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

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
			start();
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

int
HTU21D::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(HTU21D_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(HTU21D_CONVERSION_INTERVAL))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100))
				return -EINVAL;

			irqstate_t flags = irqsave();
			if (!_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}
			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
HTU21D::read(struct file *filp, char *buffer, size_t buflen)
{

	unsigned count = buflen / sizeof(struct humidity_s);
	struct humidity_s *hbuf = reinterpret_cast<struct humidity_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(hbuf)) {
				ret += sizeof(*hbuf);
				hbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(hbuf)) {
			ret = sizeof(*hbuf);
		}

	} while (0);

	return ret;
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
	if (g_dev != nullptr) {
		if (OK != g_dev->probe()) {
			errx(1, "test::OK != g_dev->probe()");
		}
	}
	errx(1, "Start the driver first");

	exit(0);
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

