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
/*
  the ADIS16488 is a tactical grade 10 DoF sensor. This driver uses the
  gyro, accel and temperature; the magnetometer and barometer on the
  same die are not used.

  Like the ADIS1647x it uses 16 bit registers, but it differs in two
  important ways:

   - the register map is paged. Page 0 holds the output data, page 3
     holds the control registers, and the page is selected by writing
     the PAGE_ID register at address 0x00 of every page

   - it has no burst read mode, so a sample is fetched as a pipelined
     run of 16 bit SPI frames rather than in one transfer

  It also needs to run as the only sensor on the SPI bus for good
  performance.

  A board wires it up in hwdef.dat like this, with SCLK no higher than
  the 15MHz the part takes and DIO2 (the factory default data ready
  line) on a GPIO:

    PD8  DRDY_ADIS16488 INPUT GPIO(93)
    define HAL_DRDY_ADIS16488_PIN 93
    SPIDEV adis16488 SPI2 DEVID1 CS_ADIS16488 MODE3 1*MHZ 11*MHZ
    IMU ADIS16488 SPI:adis16488 ROTATION_ROLL_180 HAL_DRDY_ADIS16488_PIN

  The rotation is the one that suits a part mounted flat and label up
  with pin 1 forward: the sensor axes are right handed with z up, so
  turning them into the vehicle frame is a roll of 180 degrees. The
  data ready pin is optional, without it the driver free runs on its
  own timing.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ADIS16488 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation,
                                            uint8_t drdy_gpio=0);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_ADIS16488(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation,
                                uint8_t drdy_gpio);

    /*
      the output data registers fetched every cycle. They are
      contiguous, running from TEMP_OUT at 0x0E to Z_ACCL_OUT at 0x26,
      and each inertial axis is a 32 bit value split over a low and a
      high word.
     */
    enum data_index : uint8_t {
        IDX_TEMP = 0,
        IDX_GX_LOW, IDX_GX_HIGH,
        IDX_GY_LOW, IDX_GY_HIGH,
        IDX_GZ_LOW, IDX_GZ_HIGH,
        IDX_AX_LOW, IDX_AX_HIGH,
        IDX_AY_LOW, IDX_AY_HIGH,
        IDX_AZ_LOW, IDX_AZ_HIGH,
        NUM_DATA_REGS
    };

    /*
      initialise driver
     */
    bool init();
    bool check_product_id(uint16_t &id);

    // fetch one raw sample block over SPI, without publishing it
    bool fetch_sample(uint16_t *vals);

    // time what a sample actually costs on this bus
    uint32_t measure_sample_cost_us(void);

    // settle on an output rate the bus can sustain
    void choose_rate(uint32_t wanted_hz, uint32_t &decimation);

    void read_sensor(void);
    void loop(void);

    // hold off for the minimum stall period between two SPI frames
    void stall(void) const;

    // report a probe failure to the ground station
    void report_failure(const char *fmt, ...) const FMT_PRINTF(2, 3);

    // select the register page a register lives on
    bool set_page(uint8_t page);

    /*
      how much padding a write command needs inside its chip select
      window for the part to act on it. See write_frame().
     */
    enum write_pad_t : uint8_t {
        WRITE_PAD_NONE = 0,
        WRITE_PAD_LEAD = 1,
        WRITE_PAD_BOTH = 2,
    };

    // send one write command frame
    bool write_frame(uint8_t addr, uint8_t data) const;

    // check whether a write actually changes a register
    bool page_write_works(void);

    // check whether chip select is framing our transfers
    bool cs_frames_transfers(void);

    // read a 16 bit register on whatever page is currently selected
    uint16_t read_reg16_raw(uint8_t addr) const;

    // read a 16 bit register, given as ADIS_REG(page, address)
    uint16_t read_reg16(uint16_t reg);

    // write a 16 bit register, given as ADIS_REG(page, address)
    bool write_reg16(uint16_t reg, uint16_t value, bool confirm=false);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    enum Rotation rotation;
    uint8_t drdy_pin;

    // page currently selected on the sensor, PAGE_UNKNOWN when we have
    // not established it yet
    uint8_t current_page;

    // gap held between two SPI frames, relaxed while configuring
    uint8_t stall_us;

    // padding used around every write command
    write_pad_t write_pad;

    // running temperature average, published to the frontend at 20Hz
    float temp_sum;
    uint16_t temp_count;
    uint16_t temp_publish_count;

    float expected_sample_rate_hz;
    uint32_t period_us;

    float accel_scale;
    float gyro_scale;
};
