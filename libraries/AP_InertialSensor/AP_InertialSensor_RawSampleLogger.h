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

  This is a complete capture of one IMU, not a periodic snapshot: every
  sample the backend produces is written, in the order the sensor
  produced it, with accel and gyro interleaved in a single stream.  The
  values are the sample as handed to the front end by the driver, before
  any of the driver low pass filter, harmonic notch or accumulation the
  rest of the vehicle code sees.

  The SD card is far slower, and far less predictable, than the IMU.  So
  the IMU bus thread only ever pushes a fixed size record into a ring
  buffer and returns; a separate low priority thread formats the records
  as text and writes them out in large aligned blocks.  If the card
  stalls for long enough to fill the ring buffer the samples that could
  not be stored are counted rather than waited for: the bus thread is
  never blocked and the vehicle never panics.  Any such loss is recorded
  in the file itself so the gap can be found when the log is analysed.

  See AP_InertialSensor_RawSampleLogger.cpp for the file format.
 */
#pragma once

#include "AP_InertialSensor_config.h"

#if AP_INERTIALSENSOR_RAW_SAMPLE_LOGGER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

class AP_InertialSensor_RawSampleLogger {
public:
    AP_InertialSensor_RawSampleLogger();

    CLASS_NO_COPY(AP_InertialSensor_RawSampleLogger);

    static const struct AP_Param::GroupInfo var_info[];

    // which sensor a record came from.  The values are the bit numbers
    // used by the _SENS parameter.
    enum class SampleType : uint8_t {
        ACCEL = 0,
        GYRO  = 1,
    };

    // start the writer thread.  Called once from AP_InertialSensor::init()
    void init();

    // housekeeping at main loop rate, used only to talk to the user
    void periodic();

    // take one sample.  Called from the IMU bus thread once per
    // physical sensor sample, so it must stay short and must never
    // block
    void sample(uint8_t instance, SampleType type, uint64_t sample_us, const Vector3f &sample) __RAMFUNC__;

    // true while samples are being written to a file
    bool logging() const { return _running; }

private:

    // one entry in the ring buffer.  Kept small and fixed size: the
    // conversion to text is expensive and is done by the writer thread,
    // not by the IMU thread.
    struct PACKED RawSample {
        uint64_t sample_us;
        float v[3];
        uint8_t type;
    };

    // writer thread
    void start_thread();
    void thread_main();

    // file lifecycle, all called from the writer thread
    bool start_logging();
    void stop_logging(const char *reason, bool drain);
    bool open_next_file();
    bool write_header();

    // move samples from the ring buffer to the file
    void write_samples();
    uint16_t fill_writebuf();
    bool flush_writebuf(uint16_t len);
    void note_dropped_samples();
    void handle_write_error();

    // parameters
    AP_Int8  _enable;
    AP_Int8  _imu_instance;
    AP_Int8  _sensor_mask;
    AP_Int16 _buffer_kb;

    // ring buffer between the IMU bus thread and the writer thread
    ObjectBuffer_TS<RawSample> *_buffer{nullptr};

    // block of text waiting to go to the card
    char *_writebuf{nullptr};
    uint16_t _writebuf_len{0};
    uint32_t _last_flush_ms{0};

    int _fd = -1;
    char _filename[40] {};

    // copies of the parameters taken when the file was opened, so that
    // the IMU thread never reads an AP_Param and the configuration
    // cannot change part way through a file
    uint8_t _log_instance{0};
    uint8_t _log_mask{0};

    // the only state the IMU thread looks at.  Set by the writer thread
    // once the ring buffer and the file are both ready.
    volatile bool _running{false};

    // set once we have tried to create the writer thread
    bool _thread_started{false};

    // set when we have given up.  Cleared by setting ENABLE to 0.
    bool _failed{false};

    // set when the filesystem rejected a write
    bool _write_error{false};

    // samples the IMU thread could not store because the card could not
    // keep up.  Monotonic, incremented by the IMU thread only.
    volatile uint32_t _dropped{0};
    uint32_t _dropped_noted{0};

    uint32_t _samples_written{0};

    // user reporting state, owned by the main thread
    bool _announced_running{false};
    uint32_t _announced_dropped{0};
    uint32_t _last_drop_report_ms{0};
    const char *_stop_reason{nullptr};
};

#endif  // AP_INERTIALSENSOR_RAW_SAMPLE_LOGGER_ENABLED
