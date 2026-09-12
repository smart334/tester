/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  Continuous logging of raw IMU samples to a CSV file on the SD card.

  The file is written to the board storage directory as imu_log_001.csv,
  imu_log_002.csv and so on; the lowest number not already present is
  used, so an existing log is never overwritten.

  Format:

    # ArduPilot raw IMU sample log
    # imu=0 accel_rate_hz=1000 gyro_rate_hz=8000 buffer_samples=1560
    # accel in m/s/s, gyro in rad/s, sensor sample before any filtering
    # comment lines start with '#'
    # '# dropped,N' means N samples were lost to an SD card stall here
    type,sample_us,x,y,z
    A,153870451,0.123456,-0.234567,-9.812345
    G,153870451,0.001221,-0.002441,0.000610
    ...

  type is A for accel and G for gyro, and the rows appear in the order
  the driver produced them, so the accel/gyro interleave in the file is
  the interleave of the sensor.  sample_us is the timestamp the driver
  gave the sample, in the same time base as AP_HAL::micros64().

  The comment lines are understood directly by the common analysis
  tools, for example:

    import pandas as pd
    d = pd.read_csv('imu_log_001.csv', comment='#')
    gyro = d[d.type == 'G']
 */

#include "AP_InertialSensor_RawSampleLogger.h"

#if AP_INERTIALSENSOR_RAW_SAMPLE_LOGGER_ENABLED

#include "AP_InertialSensor.h"

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

// where the CSV files are written.  On ChibiOS this is /APM on the SD
// card, on SITL and Linux it is the working directory.
#ifndef AP_INERTIALSENSOR_RAWLOG_DIRECTORY
#define AP_INERTIALSENSOR_RAWLOG_DIRECTORY HAL_BOARD_STORAGE_DIRECTORY
#endif

// highest file number we will look for before giving up
#define RAWLOG_MAX_FILE_NUM 999

// Longest line we can produce.  "%.6f" of a float is at most 47
// characters, so a row is at most 1 + 1 + 20 + 3*(1+47) + 3 + 1 = 170
// even for a sensor reporting nonsense.  Rounded up so a row can never
// be truncated part way through.
#define RAWLOG_MAX_LINE_LEN 176

// how much text we hand to the filesystem in one write().  SD cards
// are much happier with a few large writes than with many small ones.
#define RAWLOG_WRITE_CHUNK 4096

// push a partial block out at least this often, so an abrupt power loss
// costs at most this much data
#define RAWLOG_FLUSH_INTERVAL_MS 1000

// give the card at most this long per pass before going back round the
// writer loop, so the thread stays responsive to a stop request
#define RAWLOG_MAX_PASS_MS 50

#define RAWLOG_THREAD_STACK_SIZE 2048

const AP_Param::GroupInfo AP_InertialSensor_RawSampleLogger::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Raw IMU sample logging to SD card
    // @Description: Enables continuous logging of raw IMU samples to a CSV file on the SD card. A new file imu_log_NNN.csv is started each time this is set to 1, using the lowest number not already on the card. Set to 0 to close the file. This is a high bandwidth log and is intended for bench analysis, not for normal flight logging.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_InertialSensor_RawSampleLogger, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: IMU
    // @DisplayName: IMU instance to log
    // @Description: Which IMU instance to capture raw samples from.
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("IMU", 2, AP_InertialSensor_RawSampleLogger, _imu_instance, 0),

    // @Param: SENS
    // @DisplayName: Sensors to log
    // @Description: Bitmask of which sensors of the selected IMU to log.
    // @Bitmask: 0:Accel,1:Gyro
    // @User: Advanced
    AP_GROUPINFO("SENS", 3, AP_InertialSensor_RawSampleLogger, _sensor_mask, 3),

    // @Param: BUFKB
    // @DisplayName: Raw IMU log buffer size
    // @Description: Size of the buffer that absorbs the difference between the IMU sample rate and the speed of the SD card. Larger values ride out longer card stalls at the cost of memory. If samples are still being dropped, increase this.
    // @Units: kB
    // @Range: 8 512
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("BUFKB", 4, AP_InertialSensor_RawSampleLogger, _buffer_kb, 32),

    AP_GROUPEND
};

AP_InertialSensor_RawSampleLogger::AP_InertialSensor_RawSampleLogger()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_InertialSensor_RawSampleLogger::init()
{
    // start now if the feature was already enabled at boot, otherwise
    // wait until the user asks for it
    start_thread();
}

/*
  create the writer thread.  This is done on demand rather than at boot
  so that a board which never uses this feature does not pay for the
  thread stack.
 */
void AP_InertialSensor_RawSampleLogger::start_thread()
{
    if (_thread_started || _failed || _enable == 0) {
        return;
    }

    // one attempt only; a failure here is not worth retrying every loop
    _thread_started = true;

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_RawSampleLogger::thread_main, void),
                                      "IMU_rawlog", RAWLOG_THREAD_STACK_SIZE,
                                      AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INS: raw log thread failed");
        _failed = true;
    }
}

/*
  called from the IMU bus thread, once per physical sensor sample.

  All this does is stamp a fixed size record into the ring buffer.  The
  expensive part, turning it into text and getting it onto the card, is
  left to the writer thread.
 */
void AP_InertialSensor_RawSampleLogger::sample(uint8_t instance, SampleType type,
                                               uint64_t sample_us, const Vector3f &s)
{
    if (!_running || instance != _log_instance) {
        return;
    }
    if ((_log_mask & (1U << uint8_t(type))) == 0) {
        return;
    }

    const RawSample smp {
        sample_us,
        { s.x, s.y, s.z },
        uint8_t(type)
    };

    if (!_buffer->push(smp)) {
        /*
          The card could not keep up and the buffer is full.  Waiting
          here would stall the IMU bus thread and take the vehicle with
          it, so the sample is counted instead.  The count is written
          into the file by the writer thread, so the gap is visible when
          the log is analysed.
         */
        _dropped++;
    }
}

/*
  main loop rate housekeeping.  This runs on the main thread and only
  exists to tell the user what the writer thread is doing.
 */
void AP_InertialSensor_RawSampleLogger::periodic()
{
    start_thread();

    if (_running != _announced_running) {
        _announced_running = _running;
        if (_running) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS: raw log %s", _filename);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS: raw log stopped: %s, %u samples",
                          _stop_reason != nullptr ? _stop_reason : "done",
                          unsigned(_samples_written));
        }
    }

    const uint32_t dropped = _dropped;
    const uint32_t now_ms = AP_HAL::millis();
    if (dropped != _announced_dropped && now_ms - _last_drop_report_ms > 5000) {
        _last_drop_report_ms = now_ms;
        _announced_dropped = dropped;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INS: raw log dropped %u samples",
                      unsigned(dropped));
    }
}

/*
  the writer thread.  Owns the file and the write buffer; the IMU thread
  never touches either.
 */
void AP_InertialSensor_RawSampleLogger::thread_main()
{
    while (true) {
        if (_enable == 0) {
            if (_running) {
                stop_logging("disabled", true);
            }
            // a stopped or failed logger is armed again by cycling ENABLE
            _failed = false;
            hal.scheduler->delay(100);
            continue;
        }

        if (!_running) {
            if (_failed) {
                // wait for the user to cycle ENABLE rather than
                // retrying a hopeless open every millisecond
                hal.scheduler->delay(100);
                continue;
            }
            // every failure path inside start_logging() sets _failed,
            // so this cannot spin
            start_logging();
            continue;
        }

        hal.scheduler->delay(1);
        write_samples();
    }
}

/*
  allocate the buffers, open the next free file and let the IMU thread
  start feeding us
 */
bool AP_InertialSensor_RawSampleLogger::start_logging()
{
    EXPECT_DELAY_MS(3000);

    if (_buffer == nullptr) {
        const uint32_t nsamples = (constrain_int32(_buffer_kb, 8, 512) * 1024U) / sizeof(RawSample);
        _buffer = NEW_NOTHROW ObjectBuffer_TS<RawSample>(nsamples);
        if (_buffer == nullptr || _buffer->get_size() == 0) {
            delete _buffer;
            _buffer = nullptr;
            _failed = true;
            _stop_reason = "no memory";
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INS: raw log needs %uk", unsigned(constrain_int32(_buffer_kb, 8, 512)));
            return false;
        }
    }

    if (_writebuf == nullptr) {
        _writebuf = (char *)calloc(1, RAWLOG_WRITE_CHUNK + RAWLOG_MAX_LINE_LEN);
        if (_writebuf == nullptr) {
            _failed = true;
            _stop_reason = "no memory";
            return false;
        }
    }

    _buffer->clear();
    _writebuf_len = 0;
    _write_error = false;
    _dropped = 0;
    _dropped_noted = 0;
    _samples_written = 0;
    _last_flush_ms = AP_HAL::millis();

    if (!open_next_file()) {
        _failed = true;
        _stop_reason = "open failed";
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INS: raw log open failed");
        return false;
    }

    // take a copy of the configuration so the IMU thread never reads an
    // AP_Param, and so the selection cannot change part way through a file
    _log_instance = constrain_int16(_imu_instance, 0, INS_MAX_INSTANCES-1);
    _log_mask = _sensor_mask;

    if (!write_header()) {
        stop_logging("write failed", false);
        _failed = true;
        return false;
    }

    // everything is ready: from here the IMU thread will push samples
    _running = true;
    return true;
}

/*
  find and open the lowest numbered imu_log_NNN.csv that does not
  already exist
 */
bool AP_InertialSensor_RawSampleLogger::open_next_file()
{
    auto &fs = AP::FS();

    struct stat st;
    if (fs.stat(AP_INERTIALSENSOR_RAWLOG_DIRECTORY, &st) != 0) {
        fs.mkdir(AP_INERTIALSENSOR_RAWLOG_DIRECTORY);
    }

    for (uint16_t n = 1; n <= RAWLOG_MAX_FILE_NUM; n++) {
        snprintf(_filename, sizeof(_filename), "%s/imu_log_%03u.csv",
                 AP_INERTIALSENSOR_RAWLOG_DIRECTORY, unsigned(n));
        if (fs.stat(_filename, &st) == 0) {
            // taken, try the next number
            continue;
        }
        _fd = fs.open(_filename, O_WRONLY|O_CREAT|O_TRUNC);
        if (_fd != -1) {
            return true;
        }
        // could not create this one; the card may be full or read only,
        // so there is no point walking the whole range
        break;
    }

    _filename[0] = 0;
    return false;
}

/*
  write the comment block and the column names
 */
bool AP_InertialSensor_RawSampleLogger::write_header()
{
    const auto &ins = AP::ins();
    const uint8_t instance = constrain_int16(_imu_instance, 0, INS_MAX_INSTANCES-1);

    const int hdr_len = snprintf(_writebuf, RAWLOG_WRITE_CHUNK,
                                 "# ArduPilot raw IMU sample log\n"
                                 "# imu=%u accel_rate_hz=%u gyro_rate_hz=%u buffer_samples=%u\n"
                                 "# accel in m/s/s, gyro in rad/s, sensor sample before any filtering\n"
                                 "# comment lines start with '#'\n"
                                 "# '# dropped,N' means N samples were lost to an SD card stall here\n"
                                 "type,sample_us,x,y,z\n",
                                 unsigned(instance),
                                 unsigned(ins.get_accel_rate_hz(instance)),
                                 unsigned(ins.get_gyro_rate_hz(instance)),
                                 unsigned(_buffer->get_size()));
    if (hdr_len <= 0 || hdr_len >= RAWLOG_WRITE_CHUNK) {
        return false;
    }
    _writebuf_len = hdr_len;

    while (_writebuf_len > 0) {
        if (!flush_writebuf(_writebuf_len)) {
            return false;
        }
    }
    return true;
}

/*
  move as much of the ring buffer as we can into the file.  Called
  repeatedly from the writer thread.
 */
void AP_InertialSensor_RawSampleLogger::write_samples()
{
    const uint32_t start_ms = AP_HAL::millis();

    while (true) {
        note_dropped_samples();
        fill_writebuf();
        if (_writebuf_len < RAWLOG_WRITE_CHUNK) {
            // ring buffer is drained
            break;
        }
        if (!flush_writebuf(RAWLOG_WRITE_CHUNK)) {
            handle_write_error();
            return;
        }
        if (AP_HAL::millis() - start_ms > RAWLOG_MAX_PASS_MS) {
            // come back next time round rather than hogging the thread
            return;
        }
    }

    // push out a partial block now and then so a power loss is cheap
    if (_writebuf_len > 0 &&
        AP_HAL::millis() - _last_flush_ms > RAWLOG_FLUSH_INTERVAL_MS) {
        if (!flush_writebuf(_writebuf_len)) {
            handle_write_error();
        }
    }
}

/*
  the card has stopped accepting data.  Close up and stay stopped until
  the user cycles ENABLE, rather than retrying every millisecond.
 */
void AP_InertialSensor_RawSampleLogger::handle_write_error()
{
    if (!_write_error) {
        return;
    }
    stop_logging("write error", false);
    _failed = true;
}

/*
  record any samples the IMU thread had to throw away, so the gap is
  visible in the file
 */
void AP_InertialSensor_RawSampleLogger::note_dropped_samples()
{
    const uint32_t dropped = _dropped;
    if (dropped == _dropped_noted) {
        return;
    }
    if (_writebuf_len > RAWLOG_WRITE_CHUNK) {
        // no headroom for another line; it will be written next pass
        return;
    }
    const int n = snprintf(&_writebuf[_writebuf_len], RAWLOG_MAX_LINE_LEN,
                           "# dropped,%u\n", unsigned(dropped - _dropped_noted));
    if (n <= 0 || n >= RAWLOG_MAX_LINE_LEN) {
        return;
    }
    _writebuf_len += n;
    _dropped_noted = dropped;
}

/*
  format samples into the write buffer until it is full or the ring
  buffer is empty.  Returns the number of samples formatted.
 */
uint16_t AP_InertialSensor_RawSampleLogger::fill_writebuf()
{
    uint16_t count = 0;
    RawSample s;

    while (_writebuf_len < RAWLOG_WRITE_CHUNK && _buffer->pop(s)) {
        const int n = snprintf(&_writebuf[_writebuf_len], RAWLOG_MAX_LINE_LEN,
                               "%c,%llu,%.6f,%.6f,%.6f\n",
                               s.type == uint8_t(SampleType::GYRO) ? 'G' : 'A',
                               (unsigned long long)s.sample_us,
                               double(s.v[0]), double(s.v[1]), double(s.v[2]));
        if (n <= 0 || n >= RAWLOG_MAX_LINE_LEN) {
            // cannot happen for any float value, but never write a
            // partial row if it somehow does
            continue;
        }
        _writebuf_len += n;
        _samples_written++;
        count++;
    }

    return count;
}

/*
  hand len bytes from the front of the write buffer to the filesystem.
  Returns false if logging has been stopped by an error.
 */
bool AP_InertialSensor_RawSampleLogger::flush_writebuf(uint16_t len)
{
    if (_fd == -1 || len == 0) {
        return false;
    }

    EXPECT_DELAY_MS(3000);

    auto &fs = AP::FS();

    /*
      keep the FAT metadata current without syncing on every write, the
      same way AP_Logger does: trim the write so it lands exactly on the
      point the filesystem wants to sync at.
     */
    const uint32_t bytes_until_fsync = fs.bytes_until_fsync(_fd);
    if (bytes_until_fsync > 0 && len > bytes_until_fsync) {
        len = bytes_until_fsync;
    }

    const ssize_t nwritten = fs.write(_fd, _writebuf, len);
    if (nwritten <= 0) {
        _write_error = true;
        return false;
    }

    if (uint32_t(nwritten) == bytes_until_fsync) {
        fs.fsync(_fd);
    }

    if (uint16_t(nwritten) < _writebuf_len) {
        memmove(_writebuf, &_writebuf[nwritten], _writebuf_len - nwritten);
    }
    _writebuf_len -= nwritten;
    _last_flush_ms = AP_HAL::millis();

    return true;
}

/*
  stop the IMU thread feeding us, write out what is left and close the
  file.  drain is false when we are stopping because the card itself has
  failed, as there is then nowhere for the remaining samples to go.
 */
void AP_InertialSensor_RawSampleLogger::stop_logging(const char *reason, bool drain)
{
    // the IMU thread stops pushing as soon as this is clear, so
    // everything below is ours alone
    _running = false;

    if (_fd == -1) {
        _stop_reason = reason;
        return;
    }

    if (drain) {
        // write out the samples the IMU thread already handed us
        while (!_write_error) {
            note_dropped_samples();
            const uint16_t formatted = fill_writebuf();
            if (_writebuf_len == 0) {
                break;
            }
            if (!flush_writebuf(MIN(_writebuf_len, uint16_t(RAWLOG_WRITE_CHUNK)))) {
                break;
            }
            if (formatted == 0 && _buffer->is_empty()) {
                break;
            }
        }
    }

    if (_fd != -1) {
        EXPECT_DELAY_MS(3000);
        AP::FS().fsync(_fd);
        AP::FS().close(_fd);
        _fd = -1;
    }

    _writebuf_len = 0;
    _stop_reason = reason;
}

#endif  // AP_INERTIALSENSOR_RAW_SAMPLE_LOGGER_ENABLED
