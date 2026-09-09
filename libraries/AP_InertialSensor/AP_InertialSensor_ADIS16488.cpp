/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdarg.h>
#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_InertialSensor_ADIS16488.h"

/*
  set to 1 to report every step of the probe to the GCS. The console is
  not reachable on boards whose only serial port carries MAVLink, so
  this goes out as statustext and lands in the ground station messages.
 */
#ifndef AP_INERTIALSENSOR_ADIS16488_DEBUG
#define AP_INERTIALSENSOR_ADIS16488_DEBUG 0
#endif

#if AP_INERTIALSENSOR_ADIS16488_DEBUG
#define ADIS_DEBUG(fmt, args ...) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS16488: " fmt, ##args)
#else
#define ADIS_DEBUG(fmt, args ...)
#endif

// a failed probe leaves the vehicle with no IMU at all, so always say why
#define ADIS_ERROR(fmt, args ...) GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ADIS16488: " fmt, ##args)

/*
  registers are identified by the page they live on as well as their
  address, as the whole map is 13 pages of 64 sixteen bit registers
 */
#define ADIS_REG(page, addr) ((uint16_t)(((page) << 8) | (addr)))
#define REG_PAGE(reg)        ((uint8_t)((reg) >> 8))
#define REG_ADDR(reg)        ((uint8_t)((reg) & 0xFF))

#define PAGE_OUTPUT   0x00
#define PAGE_CONTROL  0x03

// value we hold in current_page until we know which page is selected
#define PAGE_UNKNOWN  0xFF

// PAGE_ID sits at address 0x00 of every page
#define PAGE_ID_ADDR  0x00

// set on the address byte of a frame to make it a write
#define WRITE_FLAG    0x80

/*
  page 0, output data
 */
#define REG_SYS_E_FLAG      ADIS_REG(PAGE_OUTPUT, 0x08)
#define REG_DIAG_STS        ADIS_REG(PAGE_OUTPUT, 0x0A)
// bits 0 to 5 are the gyro and accel start-up self test results
# define DIAG_STS_INERTIAL_MASK 0x003F
#define REG_TEMP_OUT        ADIS_REG(PAGE_OUTPUT, 0x0E)
#define REG_X_GYRO_OUT      ADIS_REG(PAGE_OUTPUT, 0x12)
#define REG_Z_ACCL_OUT      ADIS_REG(PAGE_OUTPUT, 0x26)
#define REG_PROD_ID         ADIS_REG(PAGE_OUTPUT, 0x7E)
# define PROD_ID_16488      0x4068

/*
  page 2, calibration. Only the user scratch registers are touched, and
  only when debugging: they are read/write locations that affect nothing,
  which makes them the right place to prove whether writes land at all.
 */
#define REG_USER_SCR_1      ADIS_REG(0x02, 0x74)

/*
  page 3, control
 */
#define REG_GLOB_CMD        ADIS_REG(PAGE_CONTROL, 0x02)
# define GLOB_CMD_SW_RESET  (1U<<7)
#define REG_FNCTIO_CTRL     ADIS_REG(PAGE_CONTROL, 0x06)
// 00 = DIO1, 01 = DIO2, 10 = DIO3, 11 = DIO4
# define FNCTIO_CTRL_DR_LINE_MASK 0x3
# define FNCTIO_CTRL_DR_POLARITY  (1U<<2)
# define FNCTIO_CTRL_DR_ENABLE    (1U<<3)
#define REG_CONFIG          ADIS_REG(PAGE_CONTROL, 0x0A)
# define CONFIG_POINT_OF_PERCUSSION (1U<<6)
# define CONFIG_LINEAR_G_COMP       (1U<<7)
#define REG_DEC_RATE        ADIS_REG(PAGE_CONTROL, 0x0C)
#define REG_FILTR_BNK_0     ADIS_REG(PAGE_CONTROL, 0x16)
#define REG_FILTR_BNK_1     ADIS_REG(PAGE_CONTROL, 0x18)

/*
  timings
 */
/*
  minimum gap the sensor needs between two 16 bit frames. The datasheet
  gives 2us; the Linux driver for this family allows 5us, so use that
  in the sample loop and a far more relaxed gap while configuring, where
  a few extra microseconds per frame cost us nothing.
 */
#define T_STALL_US       5U
#define T_STALL_INIT_US  20U

/*
  the datasheet gives 500ms for both the power-on start-up time and the
  reset recovery time, as the time until data is available. Waiting any
  less risks resetting the part again while it is still recovering.
 */
#define T_RESET_MS   550U

// time from power on before the part will answer, measured from boot as
// it comes up on the same supply we do
#define T_STARTUP_MS 500U

// how many reset-and-retry rounds we give the part before giving up
#define PROBE_TRIES  5U

// how many times we retry a page switch before calling it lost
#define PAGE_RETRIES 5U

/*
  how many times a probe failure is repeated to the ground station.
  Statustext raised during init is dropped outright, not queued, once
  the streaming channel mask has been worked out but no link is
  streaming yet, and a failing probe sits right in that window.
 */
#define FAILURE_REPEATS 4U

/*
  the output stage always runs at 2460 SPS, and DEC_RATE decimates it
  by D where the register holds D-1 and D tops out at 2048
 */
#define INTERNAL_RATE_HZ 2460U
#define MAX_DEC_RATE     2048U

/*
  the output rate we ask the sensor for. The part has no burst mode so
  every sample costs 14 SPI frames, and the default gives up the top
  decimation step to keep that off the bus.
 */
#ifndef AP_INERTIALSENSOR_ADIS16488_RATE_HZ
#define AP_INERTIALSENSOR_ADIS16488_RATE_HZ 1230
#endif

/*
  which of the four DIOx lines carries data ready. DIO2 is the factory
  default and is what the datasheet wires to the processor IRQ.
 */
#ifndef AP_INERTIALSENSOR_ADIS16488_DRDY_DIO
#define AP_INERTIALSENSOR_ADIS16488_DRDY_DIO 2
#endif

// TEMP_OUT reads 0.00565 degC per LSB with 25degC at zero
#define TEMP_SCALE_C  0.00565f
#define TEMP_OFFSET_C 25.0f

// rate at which we hand an averaged temperature to the frontend
#define TEMP_PUBLISH_HZ 20U

extern const AP_HAL::HAL& hal;

/*
  join the high and low words of a 32 bit output register pair
 */
static inline int32_t combine32(uint16_t high, uint16_t low)
{
    return int32_t((uint32_t(high) << 16) | low);
}

AP_InertialSensor_ADIS16488::AP_InertialSensor_ADIS16488(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation,
                                                         uint8_t drdy_gpio)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , drdy_pin(drdy_gpio)
    , current_page(PAGE_UNKNOWN)
    , stall_us(T_STALL_INIT_US)
    , write_pad(WRITE_PAD_NONE)
    , temp_sum(0)
    , temp_count(0)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADIS16488::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation,
                                   uint8_t drdy_gpio)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_ADIS16488(imu, std::move(dev), rotation, drdy_gpio);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS16488::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS16488)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_ADIS16488))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    /*
      as the sensor does not have a FIFO we need to jump through some
      hoops to ensure we don't lose any samples. This creates a thread
      to do the capture, running at very high priority
     */
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16488::loop, void),
                                      "ADIS16488",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create ADIS16488 thread");
    }
}

/*
  check product ID and set the scaling that goes with it. The raw value
  read is returned so a failed probe can report what it actually saw.
 */
bool AP_InertialSensor_ADIS16488::check_product_id(uint16_t &id)
{
    id = read_reg16(REG_PROD_ID);
    if (id != PROD_ID_16488) {
        return false;
    }

    /*
      the inertial outputs are 32 bit, made of an x_xxxx_OUT high word
      and an x_xxxx_LOW low word. The high word reads 0.02 deg/sec per
      LSB for the gyros and 0.8mg per LSB for the accels, so a full 32
      bit sample is 1/65536 of that.
     */
    gyro_scale = radians(0.02) / 65536.0;
    accel_scale = (0.8e-3 * GRAVITY_MSS) / 65536.0;

    // the accels are a +/-18g part
    _clip_limit = (18.0f - 0.5f) * GRAVITY_MSS;

    return true;
}

bool AP_InertialSensor_ADIS16488::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    // stay slow until we know the part is answering
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    ADIS_DEBUG("probe start, drdy pin %u", (unsigned)drdy_pin);

    /*
      take the part as we find it first. If it is already up and
      identifying correctly there is nothing to reset, and we save the
      better part of a second of boot time. Only if that fails do we
      reset and wait out the full recovery time before looking again.
     */
    uint16_t prod_id = 0;
    bool found = false;
    for (uint8_t i=0; i<PROBE_TRIES; i++) {
        found = check_product_id(prod_id);
        ADIS_DEBUG("try %u PROD_ID 0x%04x", (unsigned)i, (unsigned)prod_id);
        if (found) {
            break;
        }
        /*
          the part shares our supply and needs 500ms from power on
          before data is available, so on the first pass wait out
          whatever is left of that rather than resetting a part that was
          simply not up yet. Note that any delay here runs the mavlink
          delay callback, after which statustext raised from init can be
          dropped, so nothing above this point may depend on it.
         */
        const uint32_t now_ms = AP_HAL::millis();
        if (i == 0 && now_ms < T_STARTUP_MS) {
            hal.scheduler->delay(T_STARTUP_MS - now_ms);
            continue;
        }

        write_reg16(REG_GLOB_CMD, GLOB_CMD_SW_RESET);
        hal.scheduler->delay(T_RESET_MS);
        // the reset puts the part back on page 0
        current_page = PAGE_UNKNOWN;
    }
    if (!found) {
        /*
          0x0000 usually means MISO is stuck low or the part has no
          power, 0xFFFF that MISO is floating or chip select never
          asserts. Anything else is a different part on this bus.
         */
        report_failure("bad PROD_ID 0x%04x want 0x%04x", (unsigned)prod_id, (unsigned)PROD_ID_16488);
        return false;
    }

    /*
      the start-up self test result is latched in DIAG_STS. We only
      care about the gyro and accel bits, the magnetometer and
      barometer in this part are not used by this driver. Reading it
      clears it, so read it before SYS_E_FLAG, whose bit 5 only mirrors
      whether DIAG_STS was non zero.
     */
    const uint16_t diag_sts = read_reg16(REG_DIAG_STS);
    // reading SYS_E_FLAG is what clears the start-up error flags, so it
    // is done for the side effect even when we are not reporting it
    const uint16_t sys_e_flag = read_reg16(REG_SYS_E_FLAG);
    (void)sys_e_flag;
    ADIS_DEBUG("DIAG_STS 0x%04x SYS_E_FLAG 0x%04x", (unsigned)diag_sts, (unsigned)sys_e_flag);
    if ((diag_sts & DIAG_STS_INERTIAL_MASK) != 0) {
        report_failure("self test failed 0x%04x", (unsigned)diag_sts);
        return false;
    }

    /*
      Is the part actually converting?

      A part held in reset, or short of supply, can still answer its
      identity out of factory programmed memory while the rest of the
      map reads as zero and nothing written to it sticks. From the
      register map alone that is indistinguishable from a part that
      simply refuses writes.

      Page 0 carries live measurements, so this tells the two apart.
      Lying still, the z accelerometer reads about a g and temperature
      is only zero at exactly 25C, so all three of these reading zero
      means the sensor is not running rather than not writing.
     */
    const uint16_t temp_raw = read_reg16(REG_TEMP_OUT);
    const uint16_t gyro_raw = read_reg16(REG_X_GYRO_OUT);
    const uint16_t accel_raw = read_reg16(REG_Z_ACCL_OUT);
    ADIS_DEBUG("temp 0x%04x gx 0x%04x az 0x%04x",
               (unsigned)temp_raw, (unsigned)gyro_raw, (unsigned)accel_raw);
    if (temp_raw == 0 && gyro_raw == 0 && accel_raw == 0) {
        report_failure("no data: check RST pin 8 and VDD");
        return false;
    }

    /*
      Find a transfer shape whose writes land.

      Reads exercise neither the write bit nor the data byte, so a board
      that mangles the start or the end of its chip select window reads
      perfectly and writes nothing. Rather than guess which end, try the
      shapes in order of cost and keep the first that can actually change
      a register. A board with no such problem stops at the first.
     */
    bool write_ok = false;
    for (uint8_t pad = WRITE_PAD_NONE; pad <= WRITE_PAD_BOTH; pad++) {
        write_pad = write_pad_t(pad);
        const bool bit_ok = write_bit_ok();
        write_ok = page_write_works();
        (void)bit_ok;
        ADIS_DEBUG("pad %u: bit %s write %s", (unsigned)pad,
                   bit_ok ? "ok" : "LOST", write_ok ? "ok" : "LOST");
        if (write_ok) {
            break;
        }
    }
    if (!write_ok) {
        write_pad = WRITE_PAD_NONE;
        report_failure("no write shape reaches the part");
    }

#if AP_INERTIALSENSOR_ADIS16488_DEBUG
    /*
      Prove whether writes take effect at all before we depend on one.
      A scratch register on another page exercises the page switch and
      the byte pair write without changing how the sensor behaves, and
      the original value is put back afterwards. Writes only reach SRAM,
      the flash copy is untouched without an explicit flash update.
     */
    {
        const uint16_t saved = read_reg16(REG_USER_SCR_1);
        write_reg16(REG_USER_SCR_1, 0xA5A5);
        ADIS_DEBUG("scratch wrote 0xA5A5 read 0x%04x", (unsigned)read_reg16(REG_USER_SCR_1));
        write_reg16(REG_USER_SCR_1, saved);
    }
#endif

    /*
      pick the decimation that lands closest to the rate we want. The
      register holds D-1, where the output rate is 2460/D.
     */
    const uint32_t rate_req = constrain_uint32(AP_INERTIALSENSOR_ADIS16488_RATE_HZ, 2U, INTERNAL_RATE_HZ);
    const uint32_t d = constrain_uint32((INTERNAL_RATE_HZ + rate_req/2) / rate_req, 1U, MAX_DEC_RATE);

    expected_sample_rate_hz = float(INTERNAL_RATE_HZ) / d;

    // we deliberately set the period a bit fast to ensure we don't lose a sample
    period_us = (1000000UL * d) / INTERNAL_RATE_HZ - 20U;

    temp_publish_count = MAX(1U, uint32_t(expected_sample_rate_hz) / TEMP_PUBLISH_HZ);

    /*
      pulse data ready high on the DIOx line the board is wired to.
      This also clears the sync clock input and the alarm indicator,
      neither of which this driver uses.
     */
    const uint16_t fnctio_ctrl = FNCTIO_CTRL_DR_ENABLE | FNCTIO_CTRL_DR_POLARITY |
        ((AP_INERTIALSENSOR_ADIS16488_DRDY_DIO - 1) & FNCTIO_CTRL_DR_LINE_MASK);

    const struct {
        const char *name;
        uint16_t reg;
        uint16_t value;
    } config[] = {
        { "DEC_RATE",    REG_DEC_RATE,    uint16_t(d - 1) },
        // no FIR filtering, the frontend does its own
        { "FILTR_BNK_0", REG_FILTR_BNK_0, 0 },
        { "FILTR_BNK_1", REG_FILTR_BNK_1, 0 },
        // keep the linear-g and point of percussion compensation that
        // the factory default turns on
        { "CONFIG",      REG_CONFIG,      CONFIG_LINEAR_G_COMP | CONFIG_POINT_OF_PERCUSSION },
        { "FNCTIO_CTRL", REG_FNCTIO_CTRL, fnctio_ctrl },
    };

    for (const auto &c : config) {
        if (!write_reg16(c.reg, c.value, true)) {
            /*
              Report enough to tell the three cases apart: the page we
              actually ended up on, and whether the part saw a malformed
              frame. SYS_E_FLAG bit 3 is its SPI communication error,
              set when a transfer was not a multiple of 16 clocks.
             */
            const unsigned got = read_reg16(c.reg);
            const unsigned page = read_reg16_raw(PAGE_ID_ADDR);
            const unsigned sys_e = read_reg16(REG_SYS_E_FLAG);
            report_failure("%s write failed, got 0x%04x", c.name, got);
            report_failure("on page %u, SYS_E_FLAG 0x%04x", page, sys_e);
            return false;
        }
        ADIS_DEBUG("%s = 0x%04x", c.name, (unsigned)c.value);
    }

    // configuration is done, tighten the inter frame gap back up for
    // the sample loop, where it is paid 14 times per sample
    stall_us = T_STALL_US;

    ADIS_DEBUG("ready at %u Hz", (unsigned)expected_sample_rate_hz);

    /*
      frames are only 16 bits each and we need a lot of them per
      sample, so the clock rate matters. The part takes 15MHz.
     */
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
}

/*
  hold off for the minimum gap the sensor needs between two frames.
  Sleeping costs far more than the delay we are waiting for, so spin.
 */
void AP_InertialSensor_ADIS16488::stall(void) const
{
    const uint32_t tstart = AP_HAL::micros();
    while (AP_HAL::micros() - tstart < stall_us) {
    }
}

/*
  report a probe failure to the ground station, repeatedly, so that it
  survives the window during init where statustext goes nowhere
 */
void AP_InertialSensor_ADIS16488::report_failure(const char *fmt, ...) const
{
    char msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    for (uint8_t i=0; i<FAILURE_REPEATS; i++) {
        ADIS_ERROR("%s", msg);
        if (uint8_t(i+1) < FAILURE_REPEATS) {
            hal.scheduler->delay(1000);
        }
    }
}

/*
  send one write command frame, padded as the board needs.

  Two bit positions in a frame are only ever exercised by writes. The
  write bit is the first bit clocked after chip select falls, and the
  data byte is the last thing clocked before it rises again: a read
  carries a zero in the first and does not care about the second, so a
  board that mangles either end of its chip select window passes every
  read and loses every write.

  Padding moves the command away from those ends. A harmless read of
  PAGE_ID ahead of it puts the write bit off the front, and another
  behind it puts the data byte off the back. The part counts SPI clocks
  in groups of sixteen, so it sees the padded frames exactly as it would
  see them sent separately.
 */
bool AP_InertialSensor_ADIS16488::write_frame(uint8_t addr, uint8_t data) const
{
    uint8_t buf[6];
    uint8_t n = 0;

    if (write_pad != WRITE_PAD_NONE) {
        buf[n++] = PAGE_ID_ADDR;
        buf[n++] = 0;
    }

    buf[n++] = uint8_t(addr | WRITE_FLAG);
    buf[n++] = data;

    if (write_pad == WRITE_PAD_BOTH) {
        buf[n++] = PAGE_ID_ADDR;
        buf[n++] = 0;
    }

    return dev->transfer_fullduplex(buf, n);
}

/*
  check whether the write bit is reaching the part.

  PROD_ID is read only, so writing to it does nothing whichever way the
  command lands, but the two cases differ in what follows: a command
  taken as a write leaves no pending read, while one whose write bit was
  lost is a read of PROD_ID and puts 0x4068 on the next frame.
 */
bool AP_InertialSensor_ADIS16488::write_bit_ok(void) const
{
    if (!write_frame(REG_ADDR(REG_PROD_ID), 0x00)) {
        return false;
    }
    stall();

    uint8_t next[2] { 0, 0 };
    if (!dev->transfer_fullduplex(next, sizeof(next))) {
        return false;
    }
    stall();

    return ((next[0] << 8U) | next[1]) != PROD_ID_16488;
}

/*
  check whether a write actually changes a register, which needs the
  write bit, the address and the data byte all to arrive.

  PAGE_ID is the right target: it takes its new value from the low byte
  alone, it reads back at the same address on every page, and putting it
  back afterwards costs one more write.
 */
bool AP_InertialSensor_ADIS16488::page_write_works(void)
{
    write_frame(PAGE_ID_ADDR, PAGE_CONTROL);
    stall();
    const bool ok = (read_reg16_raw(PAGE_ID_ADDR) & 0xFF) == PAGE_CONTROL;

    // leave the part back on the output page whatever happened
    write_frame(PAGE_ID_ADDR, PAGE_OUTPUT);
    stall();
    current_page = PAGE_UNKNOWN;

    return ok;
}

/*
  read a 16 bit register on whatever page is currently selected. This is
  the page agnostic half of read_reg16(), split out so that set_page()
  can confirm a page switch without recursing back into itself.
 */
uint16_t AP_InertialSensor_ADIS16488::read_reg16_raw(uint8_t addr) const
{
    // the contents come back on the frame after the one carrying the
    // request, so this takes two frames
    uint8_t frame[2] { addr, 0 };
    if (!dev->transfer_fullduplex(frame, sizeof(frame))) {
        return 0;
    }
    stall();

    // the second frame asks for PAGE_ID, which is harmless
    frame[0] = 0;
    frame[1] = 0;
    if (!dev->transfer_fullduplex(frame, sizeof(frame))) {
        return 0;
    }
    stall();

    return (frame[0] << 8U) | frame[1];
}

/*
  select the page a register lives on.

  Every other register access depends on this having taken effect, and
  silently sitting on the wrong page would read and write the wrong
  registers entirely, so confirm the switch by reading PAGE_ID back.
 */
bool AP_InertialSensor_ADIS16488::set_page(uint8_t page)
{
    if (page == current_page) {
        return true;
    }

    uint16_t seen = 0;
    for (uint8_t i=0; i<PAGE_RETRIES; i++) {
        // PAGE_ID is the one register that takes a new value from a
        // write to its lower byte alone
        current_page = PAGE_UNKNOWN;
        if (!write_frame(PAGE_ID_ADDR, page)) {
            continue;
        }
        stall();

        /*
          PAGE_ID appears at the same address on every page, so it can
          be read back without knowing where we ended up. Only the low
          byte carries the page code: the datasheet gives PAGE_ID no bit
          format at all, so the upper byte cannot be assumed to be zero.
         */
        seen = read_reg16_raw(PAGE_ID_ADDR);
        if ((seen & 0xFF) == page) {
            current_page = page;
            return true;
        }
    }

    /*
      The read back did not settle. Carry on as if the write took, which
      is what we did before this check existed, rather than lose the
      sensor to a register whose format the datasheet never pins down.
     */
    ADIS_DEBUG("page %u readback 0x%04x", (unsigned)page, (unsigned)seen);
    current_page = page;
    return true;
}

/*
  read a 16 bit register value
 */
uint16_t AP_InertialSensor_ADIS16488::read_reg16(uint16_t reg)
{
    if (!set_page(REG_PAGE(reg))) {
        return 0;
    }
    return read_reg16_raw(REG_ADDR(reg));
}

/*
  write a 16 bit register value.

  Writes go out through transfer_fullduplex() rather than a send only
  transfer. On the wire the two are the same, sixteen clocks with chip
  select held, and driving both down one path was tried as a fix for a
  part that ignored every write: it made no difference, so the two HAL
  routes behave alike here. It is kept because it is the call the HAL
  documents as preferred. The returned bytes are discarded.
 */
bool AP_InertialSensor_ADIS16488::write_reg16(uint16_t reg, uint16_t value, bool confirm)
{
    const uint8_t retries = 8;
    for (uint8_t i=0; i<retries; i++) {
        if (!set_page(REG_PAGE(reg))) {
            continue;
        }
        const uint8_t addr = REG_ADDR(reg);

        // the lower byte goes first, a register takes its new value on
        // the write to the upper byte
        if (!write_frame(addr, uint8_t(value & 0xFF))) {
            continue;
        }
        stall();

        if (!write_frame(addr+1, uint8_t(value >> 8))) {
            continue;
        }
        stall();

        if (!confirm || read_reg16(reg) == value) {
            return true;
        }
    }
    return false;
}

/*
  read one sample of temperature, gyro and accel data
 */
void AP_InertialSensor_ADIS16488::read_sensor(void)
{
    uint16_t vals[NUM_DATA_REGS];

    {
        WITH_SEMAPHORE(dev->get_semaphore());

        if (!set_page(PAGE_OUTPUT)) {
            return;
        }

        /*
          the sensor answers a read on the frame after the one carrying
          the request, so a run of NUM_DATA_REGS+1 frames fetches the
          whole block: each frame asks for the next register while
          returning the contents of the previous request.
         */
        for (uint8_t i=0; i<=NUM_DATA_REGS; i++) {
            // a zero address on the last frame is a read of PAGE_ID,
            // which costs us nothing
            uint8_t frame[2] { 0, 0 };
            if (i < NUM_DATA_REGS) {
                frame[0] = REG_ADDR(REG_TEMP_OUT) + i*2;
            }
            if (!dev->transfer_fullduplex(frame, sizeof(frame))) {
                return;
            }
            if (i > 0) {
                vals[i-1] = (frame[0] << 8U) | frame[1];
            }
            stall();
        }
    }

    /*
      the axes are passed through as the sensor reports them, so that
      board mounting is handled entirely by the rotation given to
      probe(). See the wiring notes in the header.
     */
    Vector3f gyro{float(combine32(vals[IDX_GX_HIGH], vals[IDX_GX_LOW])),
                  float(combine32(vals[IDX_GY_HIGH], vals[IDX_GY_LOW])),
                  float(combine32(vals[IDX_GZ_HIGH], vals[IDX_GZ_LOW]))};
    Vector3f accel{float(combine32(vals[IDX_AX_HIGH], vals[IDX_AX_LOW])),
                   float(combine32(vals[IDX_AY_HIGH], vals[IDX_AY_LOW])),
                   float(combine32(vals[IDX_AZ_HIGH], vals[IDX_AZ_LOW]))};

    gyro *= gyro_scale;
    accel *= accel_scale;

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += TEMP_OFFSET_C + int16_t(vals[IDX_TEMP]) * TEMP_SCALE_C;
    temp_count++;

    if (temp_count >= temp_publish_count) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
}

/*
  sensor read loop
 */
void AP_InertialSensor_ADIS16488::loop(void)
{
    // give the data ready line two sample periods before falling back
    // on our own timing
    const uint32_t drdy_timeout_us = 2 * (period_us + 20U);

    while (true) {
        const uint32_t tstart = AP_HAL::micros();
        bool wait_ok = false;
        if (drdy_pin != 0) {
            // when we have a DRDY pin then wait for it to go high
            wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, drdy_timeout_us);
        }
        read_sensor();
        const uint32_t dt = AP_HAL::micros() - tstart;
        if (dt < period_us) {
            const uint32_t wait_us = period_us - dt;
            if (!wait_ok || wait_us > period_us/2) {
                hal.scheduler->delay_microseconds(wait_us);
            }
        }
    }
}

bool AP_InertialSensor_ADIS16488::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
