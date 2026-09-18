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

//
//  passive (receive only) u-blox UBX GPS driver for ArduPilot
//
//  This driver is derived from AP_GPS_UBLOX, with the entire transmit
//  side removed. It never writes a single byte to the receiver: no
//  polls, no CFG messages, no rate changes, no RTCM injection. It
//  assumes the module has already been configured (by the user, or by
//  a saved configuration) to stream UBX NAV messages on its own, and
//  it simply decodes whatever turns up on the serial port.
//
//  Use it when the receiver must not be reconfigured, or when only the
//  module's TX line is wired to the autopilot.
//
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_UBLOX_PASSIVE_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"

// upper bound on the payload length we will keep framing on. Payloads
// longer than this are treated as a false preamble match rather than a
// message. Messages we don't decode are skipped without being stored,
// so this only has to be large enough to stay in sync with the biggest
// message the receiver may be streaming (UBX-RXM-RAWX is the largest in
// common use at a little over 2KB)
#define UBLOX_PASSIVE_MAX_PAYLOAD 4096

// bit in UBX-NAV-TIMEGPS valid flags marking the week number as valid
#define UBX_PASSIVE_TIMEGPS_VALID_WEEK_MASK 0x2

// lag we report when we have no way of knowing the module generation.
// We never poll UBX-MON-VER, so this is the conservative value the
// u-blox driver uses for an unidentified module. Users who care should
// set GPS_DELAY_MS for their receiver
#define UBLOX_PASSIVE_DEFAULT_LAG_SEC 0.22f

class AP_GPS_UBLOX_PASSIVE : public AP_GPS_Backend
{
public:
    AP_GPS_UBLOX_PASSIVE(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // Methods
    bool read() override;

    // UBX-NAV-PVT carries the RTK carrier solution flags, so a passive
    // module streaming PVT can report up to an RTK fixed solution
    AP_GPS_FixType highest_supported_status(void) override { return AP_GPS_FixType::RTK_FIXED; }

    static bool _detect(struct UBLOX_PASSIVE_detect_state &state, uint8_t data);

    // there is nothing for us to configure, so we are always ready
    bool is_configured(void) const override { return true; }

    bool get_lag(float &lag_sec) const override {
        lag_sec = UBLOX_PASSIVE_DEFAULT_LAG_SEC;
        return true;
    }

    const char *name() const override { return "u-blox-passive"; }

    // this driver is receive only: swallow anything the frontend tries
    // to send us (GPS_INJECT_DATA, RTCM corrections) rather than
    // writing it to the module
    void inject_data(const uint8_t *data, uint16_t len) override {}

private:
    // u-blox UBX protocol essentials
    struct PACKED ubx_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };
    struct PACKED ubx_nav_posllh {
        uint32_t itow;                                  // GPS msToW
        int32_t longitude;
        int32_t latitude;
        int32_t altitude_ellipsoid;
        int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
    };
    struct PACKED ubx_nav_status {
        uint32_t itow;                                  // GPS msToW
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime;                                // milliseconds
    };
    struct PACKED ubx_nav_dop {
        uint32_t itow;                                  // GPS msToW
        uint16_t gDOP;
        uint16_t pDOP;
        uint16_t tDOP;
        uint16_t vDOP;
        uint16_t hDOP;
        uint16_t nDOP;
        uint16_t eDOP;
    };
    struct PACKED ubx_nav_solution {
        uint32_t itow;
        int32_t time_nsec;
        uint16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;                                 // cm
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;                  // cm
        int32_t ecef_x_velocity;                        // cm/s
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;                        // cm/s
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
    };
    struct PACKED ubx_nav_pvt {
        uint32_t itow;
        uint16_t year;
        uint8_t month, day, hour, min, sec;
        uint8_t valid;
        uint32_t t_acc;
        int32_t nano;
        uint8_t fix_type;
        uint8_t flags;
        uint8_t flags2;
        uint8_t num_sv;
        int32_t lon, lat;
        int32_t h_ellipsoid, h_msl;
        uint32_t h_acc, v_acc;
        int32_t velN, velE, velD, gspeed;
        int32_t head_mot;
        uint32_t s_acc;
        uint32_t head_acc;
        uint16_t p_dop;
        uint8_t flags3;
        uint8_t reserved1[5];
        int32_t headVeh;
        int16_t magDec;
        uint16_t magAcc;
    };
    struct PACKED ubx_nav_velned {
        uint32_t itow;                                  // GPS msToW
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
    };
    struct PACKED ubx_nav_timegps {
        uint32_t itow;
        int32_t ftow;
        uint16_t week;
        int8_t leapS;
        uint8_t valid;                                  // leapsvalid | weekvalid | tow valid
        uint32_t tAcc;
    };

    // Receive buffer. Only the messages this driver decodes need to
    // fit; anything longer is skipped as it arrives
    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_dop dop;
        ubx_nav_solution solution;
        ubx_nav_pvt pvt;
        ubx_nav_velned velned;
        ubx_nav_timegps timegps;
    } _buffer;

    // the decoders index these structures straight over the received
    // payload, so their layout has to match the UBX interface
    // description exactly
    static_assert(sizeof(ubx_header) == 6, "ubx_header must be 6 bytes");
    static_assert(sizeof(ubx_nav_posllh) == 28, "UBX-NAV-POSLLH must be 28 bytes");
    static_assert(sizeof(ubx_nav_status) == 16, "UBX-NAV-STATUS must be 16 bytes");
    static_assert(sizeof(ubx_nav_dop) == 18, "UBX-NAV-DOP must be 18 bytes");
    static_assert(sizeof(ubx_nav_solution) == 52, "UBX-NAV-SOL must be 52 bytes");
    static_assert(sizeof(ubx_nav_pvt) == 92, "UBX-NAV-PVT must be 92 bytes");
    static_assert(sizeof(ubx_nav_velned) == 36, "UBX-NAV-VELNED must be 36 bytes");
    static_assert(sizeof(ubx_nav_timegps) == 16, "UBX-NAV-TIMEGPS must be 16 bytes");

    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        MSG_POSLLH = 0x02,
        MSG_STATUS = 0x03,
        MSG_DOP = 0x04,
        MSG_SOL = 0x06,
        MSG_PVT = 0x07,
        MSG_VELNED = 0x12,
        MSG_TIMEGPS = 0x20
    };

    enum ubs_nav_fix_type {
        FIX_NONE = 0,
        FIX_DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_GPS_DEAD_RECKONING = 4,
        FIX_TIME = 5
    };
    enum ubx_nav_status_bits {
        NAV_STATUS_FIX_VALID = 1,
        NAV_STATUS_DGPS_USED = 2
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _msg_id;
    uint16_t        _payload_length;
    uint16_t        _payload_counter;
    uint8_t         _class;

    uint32_t        _last_vel_time;
    uint32_t        _last_pos_time;
    uint32_t        _last_pvt_itow;

    // do we have new position information?
    bool            _new_position;
    // do we have new speed information?
    bool            _new_speed;

    // used to update fix between status and position packets
    AP_GPS_FixType next_fix { AP_GPS_FixType::NONE };

    // true until UBX-NAV-DOP has given us a real HDOP
    bool _no_received_hdop { true };

    // true once the module has been seen streaming UBX-NAV-PVT
    bool _have_pvt_msg;

    // true once a message carrying geodetic position (UBX-NAV-POSLLH or
    // UBX-NAV-PVT) has been seen. Until then UBX-NAV-SOL's ECEF
    // position is used instead
    bool _have_llh_msg;

    // Buffer parse & GPS state update
    bool _parse_gps(void);

    // fill in position and velocity from the ECEF solution in
    // UBX-NAV-SOL, for a module that streams neither POSLLH nor PVT
    bool _position_from_ecef(void);

    // uBlox specific check_new_itow(), handling message length
    void _check_new_itow(uint32_t itow) {
        check_new_itow(itow, _payload_length + sizeof(ubx_header) + 2);
    }
};

#endif  // AP_GPS_UBLOX_PASSIVE_ENABLED
