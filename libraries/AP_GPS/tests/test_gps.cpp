/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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
#include <AP_gtest.h>

#include <AP_GPS/AP_GPS_NMEA.h>
#include <AP_GPS/AP_GPS_UBLOX_PASSIVE.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class AP_GPS_NMEA_Test
{
public:
    int32_t parse_decimal_100(const char *p) const
    {
        return AP_GPS_NMEA::_parse_decimal_100(p);
    }
};

TEST(AP_GPS_NMEA, parse_decimal_100)
{
    AP_GPS_NMEA_Test test;

    /* Positive numbers with possible round/truncate */
    ASSERT_EQ(100, test.parse_decimal_100("1.0"));
    ASSERT_EQ(100, test.parse_decimal_100("1.00"));
    ASSERT_EQ(100, test.parse_decimal_100("1.001"));
    ASSERT_EQ(101, test.parse_decimal_100("1.006"));

    /* Positive numbers with possible round/truncate with + signal */
    ASSERT_EQ(100, test.parse_decimal_100("+1.0"));
    ASSERT_EQ(100, test.parse_decimal_100("+1.00"));
    ASSERT_EQ(100, test.parse_decimal_100("+1.001"));
    ASSERT_EQ(101, test.parse_decimal_100("+1.006"));

    /* Positive numbers in (0, 1) range, with possible round/truncate */
    ASSERT_EQ(0, test.parse_decimal_100("0.0"));
    ASSERT_EQ(0, test.parse_decimal_100("0.00"));
    ASSERT_EQ(0, test.parse_decimal_100("0.001"));
    ASSERT_EQ(1, test.parse_decimal_100("0.006"));

    /* Negative numbers with possible round/truncate */
    ASSERT_EQ(-100, test.parse_decimal_100("-1.0"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1.00"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1.001"));
    ASSERT_EQ(-101, test.parse_decimal_100("-1.006"));

    /* Integer numbers */
    ASSERT_EQ(100, test.parse_decimal_100("1"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1"));
}

#if AP_GPS_UBLOX_PASSIVE_ENABLED
/*
  a real UBX-NAV-SOL (class 0x01, id 0x06, 52 byte payload) as streamed
  by a u-blox module which has not been configured by ArduPilot
 */
static const uint8_t ubx_nav_sol_frame[] = {
    0xB5, 0x62, 0x01, 0x06, 0x34, 0x00, 0xE4, 0x28, 0x4F, 0x12, 0x00, 0x00,
    0x00, 0x00, 0x84, 0x09, 0x03, 0xDD, 0xFA, 0xCD, 0x8C, 0x16, 0xB4, 0xCD,
    0x5F, 0x05, 0x16, 0xC1, 0x07, 0x1E, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0xB8, 0x00, 0x33, 0x1E, 0x36, 0x00, 0x00, 0x00, 0x28, 0x3D
};

// feed a buffer to the detector, returning the offset of the byte which
// completed a frame, or -1 if no frame was detected
static int detect_offset(const uint8_t *buf, uint16_t len)
{
    UBLOX_PASSIVE_detect_state dstate {};
    for (uint16_t i=0; i<len; i++) {
        if (AP_GPS_UBLOX_PASSIVE::_detect(dstate, buf[i])) {
            return i;
        }
    }
    return -1;
}

TEST(AP_GPS_UBLOX_PASSIVE, detect)
{
    // the frame is detected on its final checksum byte
    EXPECT_EQ(int(sizeof(ubx_nav_sol_frame))-1,
              detect_offset(ubx_nav_sol_frame, sizeof(ubx_nav_sol_frame)));

    // and still is when preceded by junk the module may be emitting
    uint8_t prefixed[8 + sizeof(ubx_nav_sol_frame)];
    memset(prefixed, 0xB5, 8);   // a run of false preamble bytes
    memcpy(&prefixed[8], ubx_nav_sol_frame, sizeof(ubx_nav_sol_frame));
    EXPECT_EQ(int(sizeof(prefixed))-1, detect_offset(prefixed, sizeof(prefixed)));
}

TEST(AP_GPS_UBLOX_PASSIVE, detect_rejects_corruption)
{
    uint8_t frame[sizeof(ubx_nav_sol_frame)];

    // a corrupt payload byte must fail the checksum
    memcpy(frame, ubx_nav_sol_frame, sizeof(frame));
    frame[20] ^= 0xFF;
    EXPECT_EQ(-1, detect_offset(frame, sizeof(frame)));

    // as must a corrupt checksum
    memcpy(frame, ubx_nav_sol_frame, sizeof(frame));
    frame[sizeof(frame)-1] ^= 0xFF;
    EXPECT_EQ(-1, detect_offset(frame, sizeof(frame)));

    // and a truncated frame must not be reported
    EXPECT_EQ(-1, detect_offset(ubx_nav_sol_frame, sizeof(ubx_nav_sol_frame)-1));
}
#endif  // AP_GPS_UBLOX_PASSIVE_ENABLED

AP_GTEST_MAIN()
