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
//  Derived from AP_GPS_UBLOX with the transmit side removed. See
//  AP_GPS_UBLOX_PASSIVE.h for the rationale.
//
//  The module is expected to be streaming UBX NAV messages of its own
//  accord. Any combination of the following is decoded:
//
//    UBX-NAV-PVT     (0x01 0x07) - preferred, carries everything
//    UBX-NAV-POSLLH  (0x01 0x02) - position
//    UBX-NAV-VELNED  (0x01 0x12) - velocity
//    UBX-NAV-STATUS  (0x01 0x03) - fix status
//    UBX-NAV-SOL     (0x01 0x06) - fix status, sats, time, ECEF solution
//    UBX-NAV-DOP     (0x01 0x04) - HDOP/VDOP
//    UBX-NAV-TIMEGPS (0x01 0x20) - GPS week number
//
//  Anything else on the port (other UBX classes, NMEA, RTCM) is
//  skipped.
//
//  A fix is only reported once both a position and a velocity have
//  arrived for the same iTOW, so the module has to be streaming at
//  least one of:
//
//    UBX-NAV-PVT                             (everything in one message)
//    UBX-NAV-POSLLH + UBX-NAV-VELNED         (plus STATUS for the fix type)
//    UBX-NAV-SOL                             (converted from ECEF)
//
//  The ECEF path exists so that a module streaming only the legacy
//  UBX-NAV-SOL is usable. Note that it has no hMSL to compare against,
//  so it reports height above the WGS84 ellipsoid as the altitude.
//

#include "AP_GPS_UBLOX_PASSIVE.h"

#if AP_GPS_UBLOX_PASSIVE_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/location.h>

AP_GPS_UBLOX_PASSIVE::AP_GPS_UBLOX_PASSIVE(AP_GPS &_gps,
                                           AP_GPS::Params &_params,
                                           AP_GPS::GPS_State &_state,
                                           AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _params, _state, _port),
    _ck_a(0),
    _ck_b(0),
    _step(0),
    _msg_id(0),
    _payload_length(0),
    _payload_counter(0),
    _class(0),
    _last_vel_time(0),
    _last_pos_time(0),
    _last_pvt_itow(0),
    _new_position(false),
    _new_speed(false),
    _have_pvt_msg(false),
    _have_llh_msg(false)
{
    // make sure no initialisation strings are left queued for this
    // port. AP_GPS::update_instance() keeps feeding the queued blob to
    // the module until it is empty, and this driver must not put
    // anything on the module's RX line
    gps.send_blob_start(state.instance, nullptr, 0);
}

/*
  Process bytes available from the stream.

  The stream is not assumed to contain only messages we recognise, so
  the payload of an unrecognised message is skipped while still being
  run through the checksum. That keeps us framed on a module which is
  also streaming messages this driver has no interest in.
 */
bool
AP_GPS_UBLOX_PASSIVE::read(void)
{
    bool parsed = false;

    const uint16_t numc = MIN(port->available(), 8192U);
    for (uint16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        uint8_t data;
        if (!port->read(data)) {
            break;
        }
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data(&data, 1);
#endif

    reset:
        switch (_step) {

        // Message preamble detection
        //
        // If we fail to match any of the expected bytes, we reset
        // the state machine and re-consider the failed byte as
        // the first byte of the preamble.  This improves our
        // chances of recovering from a mismatch and makes it less
        // likely that we will be fooled by the preamble appearing
        // as data in some other message.
        //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            FALLTHROUGH;
        case 0:
            if (PREAMBLE1 == data) {
                _step++;
            }
            break;

        // Message header processing
        //
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;                       // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length = data;                     // payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte

            _payload_length += (uint16_t)(data<<8);
            if (_payload_length > UBLOX_PASSIVE_MAX_PAYLOAD) {
                // assume any payload bigger than the largest message a
                // receiver may stream is a false preamble match
                _payload_length = 0;
                _step = 0;
                goto reset;
            }
            _payload_counter = 0;                       // prepare to receive payload
            if (_payload_length == 0) {
                // bypass payload and go straight to checksum
                _step++;
            }
            break;

        // Receive message data. Bytes beyond the end of the decode
        // buffer are checksummed and discarded
        //
        case 6:
            _ck_b += (_ck_a += data);                   // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length) {
                _step++;
            }
            break;

        // Checksum and message processing
        //
        case 7:
            _step++;
            if (_ck_a != data) {
                _step = 0;
                goto reset;
            }
            break;
        case 8:
            _step = 0;
            if (_ck_b != data) {
                break;                                  // bad checksum
            }
            if (_parse_gps()) {
                parsed = true;
            }
            break;
        }
    }
    return parsed;
}

/*
  fill in position and velocity from the ECEF solution in
  UBX-NAV-SOL. This is only used for a module which streams neither
  UBX-NAV-POSLLH nor UBX-NAV-PVT, so we have no geodetic position of
  our own and no way to know the geoid undulation.

  Returns true if the converted position was usable.
 */
bool AP_GPS_UBLOX_PASSIVE::_position_from_ecef(void)
{
    // UBX-NAV-SOL reports ECEF position in cm and velocity in cm/s
    const Vector3d ecef{_buffer.solution.ecef_x * 0.01,
                        _buffer.solution.ecef_y * 0.01,
                        _buffer.solution.ecef_z * 0.01};
    Vector3d llh;
    wgsecef2llh(ecef, llh);

    const double rad_to_degE7 = (180.0 / M_PI) * 1.0e7;
    const double lat_degE7 = llh[0] * rad_to_degE7;
    const double lng_degE7 = llh[1] * rad_to_degE7;

    // guard against a garbage solution, in particular an all-zero ECEF
    // vector, which converts to a height near the earth's centre
    if (!check_latlng(int32_t(lat_degE7), int32_t(lng_degE7)) ||
        fabs(llh[2]) > 100000) {
        return false;
    }

    state.location.lat = int32_t(lat_degE7);
    state.location.lng = int32_t(lng_degE7);

    // wgsecef2llh gives height above the WGS84 ellipsoid. Without
    // UBX-NAV-POSLLH or UBX-NAV-PVT we never see hMSL alongside it, so
    // we cannot work out the undulation and have to report the
    // ellipsoid height as the vehicle altitude. It can differ from AMSL
    // by up to about 100m depending on location
    state.have_undulation = false;
    state.location.alt = int32_t(llh[2] * 100);

    // rotate the ECEF velocity into the local NED frame
    const float lat_rad = float(llh[0]);
    const float lng_rad = float(llh[1]);
    const float sin_lat = sinf(lat_rad);
    const float cos_lat = cosf(lat_rad);
    const float sin_lng = sinf(lng_rad);
    const float cos_lng = cosf(lng_rad);

    const Vector3f vel_ecef{_buffer.solution.ecef_x_velocity * 0.01f,
                            _buffer.solution.ecef_y_velocity * 0.01f,
                            _buffer.solution.ecef_z_velocity * 0.01f};

    state.velocity.x = -sin_lat*cos_lng*vel_ecef.x - sin_lat*sin_lng*vel_ecef.y + cos_lat*vel_ecef.z;
    state.velocity.y = -sin_lng*vel_ecef.x + cos_lng*vel_ecef.y;
    state.velocity.z = -(cos_lat*cos_lng*vel_ecef.x + cos_lat*sin_lng*vel_ecef.y + sin_lat*vel_ecef.z);
    state.have_vertical_velocity = true;
    velocity_to_speed_course(state);

    // pAcc is a 3D position accuracy, so using it for both the
    // horizontal and vertical figures overestimates each of them
    state.horizontal_accuracy = _buffer.solution.position_accuracy_3d * 0.01f;
    state.vertical_accuracy = state.horizontal_accuracy;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;

    state.speed_accuracy = _buffer.solution.speed_accuracy * 0.01f;
    state.have_speed_accuracy = true;

    return true;
}

/*
  decode the message in _buffer. Returns true once a complete new
  position and velocity are available
 */
bool
AP_GPS_UBLOX_PASSIVE::_parse_gps(void)
{
    if (_class != CLASS_NAV) {
        // we don't ask for anything, so anything else on the port is
        // simply not ours to deal with
        return false;
    }

    switch (_msg_id) {
    case MSG_POSLLH:
        if (_payload_length != sizeof(ubx_nav_posllh)) {
            return false;
        }
        if (_have_pvt_msg) {
            // PVT is a superset of POSLLH, prefer it
            break;
        }
        _check_new_itow(_buffer.posllh.itow);
        _have_llh_msg = true;
        _last_pos_time        = _buffer.posllh.itow;
        state.location.lng    = _buffer.posllh.longitude;
        state.location.lat    = _buffer.posllh.latitude;
        state.have_undulation = true;
        state.undulation = (_buffer.posllh.altitude_msl - _buffer.posllh.altitude_ellipsoid) * 0.001;
        set_alt_amsl_cm(state, _buffer.posllh.altitude_msl / 10);

        state.status          = next_fix;
        _new_position = true;
        state.horizontal_accuracy = _buffer.posllh.horizontal_accuracy*1.0e-3f;
        state.vertical_accuracy = _buffer.posllh.vertical_accuracy*1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        break;

    case MSG_STATUS:
        if (_payload_length != sizeof(ubx_nav_status)) {
            return false;
        }
        if (_have_pvt_msg) {
            // PVT carries the fix status itself
            break;
        }
        _check_new_itow(_buffer.status.itow);
        if (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) {
            if ((_buffer.status.fix_type == FIX_3D) &&
                (_buffer.status.fix_status & NAV_STATUS_DGPS_USED)) {
                next_fix = AP_GPS_FixType::DGPS;
            } else if (_buffer.status.fix_type == FIX_3D) {
                next_fix = AP_GPS_FixType::FIX_3D;
            } else if (_buffer.status.fix_type == FIX_2D) {
                next_fix = AP_GPS_FixType::FIX_2D;
            } else {
                next_fix = AP_GPS_FixType::NONE;
                state.status = AP_GPS_FixType::NONE;
            }
        } else {
            next_fix = AP_GPS_FixType::NONE;
            state.status = AP_GPS_FixType::NONE;
        }
        break;

    case MSG_DOP:
        if (_payload_length != sizeof(ubx_nav_dop)) {
            return false;
        }
        _no_received_hdop = false;
        _check_new_itow(_buffer.dop.itow);
        state.hdop        = _buffer.dop.hDOP;
        state.vdop        = _buffer.dop.vDOP;
        break;

    case MSG_SOL:
        if (_payload_length != sizeof(ubx_nav_solution)) {
            return false;
        }
        _check_new_itow(_buffer.solution.itow);
        if (_have_pvt_msg) {
            // PVT does not carry the week number, SOL does
            state.time_week = _buffer.solution.week;
            break;
        }
        if (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) {
            if ((_buffer.solution.fix_type == FIX_3D) &&
                (_buffer.solution.fix_status & NAV_STATUS_DGPS_USED)) {
                next_fix = AP_GPS_FixType::DGPS;
            } else if (_buffer.solution.fix_type == FIX_3D) {
                next_fix = AP_GPS_FixType::FIX_3D;
            } else if (_buffer.solution.fix_type == FIX_2D) {
                next_fix = AP_GPS_FixType::FIX_2D;
            } else {
                next_fix = AP_GPS_FixType::NONE;
                state.status = AP_GPS_FixType::NONE;
            }
        } else {
            next_fix = AP_GPS_FixType::NONE;
            state.status = AP_GPS_FixType::NONE;
        }
        if (_no_received_hdop) {
            state.hdop = _buffer.solution.position_DOP;
        }
        state.num_sats    = _buffer.solution.satellites;
        if (next_fix >= AP_GPS_FixType::FIX_2D) {
            state.last_gps_time_ms = AP_HAL::millis();
            state.time_week_ms    = _buffer.solution.itow;
            state.time_week       = _buffer.solution.week;

            if (!_have_llh_msg && _position_from_ecef()) {
                // the module is streaming neither POSLLH nor PVT, so
                // this ECEF solution is all the position we get
                state.status = next_fix;
                _last_pos_time = _buffer.solution.itow;
                _last_vel_time = _buffer.solution.itow;
                _new_position = true;
                _new_speed = true;
            }
        }
        break;

    case MSG_PVT:
        if (_payload_length != sizeof(ubx_nav_pvt)) {
            return false;
        }

        _have_pvt_msg = true;
        _have_llh_msg = true;

        // position
        _check_new_itow(_buffer.pvt.itow);
        // Only adjust if:
        // we already have a valid week,
        // PVT iTOW wrapped,
        // and our time_week_ms still looks like end-of-week
        // (meaning we haven't already accepted the rollover via TIMEGPS/SOL)
        if (state.time_week != 0 &&
            _last_pvt_itow != 0 &&
            _buffer.pvt.itow < _last_pvt_itow &&
            state.time_week_ms > (AP_MSEC_PER_WEEK - (60UL * AP_MSEC_PER_SEC)) &&
            _buffer.pvt.itow < (60UL * AP_MSEC_PER_SEC)) {
            state.time_week++;
        }

        _last_pvt_itow = _buffer.pvt.itow;
        _last_pos_time        = _buffer.pvt.itow;
        state.location.lng    = _buffer.pvt.lon;
        state.location.lat    = _buffer.pvt.lat;
        state.have_undulation = true;
        state.undulation = (_buffer.pvt.h_msl - _buffer.pvt.h_ellipsoid) * 0.001;
        set_alt_amsl_cm(state, _buffer.pvt.h_msl / 10);
        switch (_buffer.pvt.fix_type) {
        case 2:
            state.status = AP_GPS_FixType::FIX_2D;
            break;
        case 3:
        case 4:
            state.status = AP_GPS_FixType::FIX_3D;
            if (_buffer.pvt.flags & 0b00000010) {  // diffsoln
                state.status = AP_GPS_FixType::DGPS;
            }
            if (_buffer.pvt.flags & 0b01000000) {  // carrsoln - float
                state.status = AP_GPS_FixType::RTK_FLOAT;
            }
            if (_buffer.pvt.flags & 0b10000000) {  // carrsoln - fixed
                state.status = AP_GPS_FixType::RTK_FIXED;
            }
            break;
        default:
            state.status = AP_GPS_FixType::NONE;
            break;
        }
        next_fix = state.status;
        _new_position = true;
        state.horizontal_accuracy = _buffer.pvt.h_acc*1.0e-3f;
        state.vertical_accuracy = _buffer.pvt.v_acc*1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        // SVs
        state.num_sats    = _buffer.pvt.num_sv;
        // velocity
        _last_vel_time         = _buffer.pvt.itow;
        state.ground_speed     = _buffer.pvt.gspeed*0.001f;          // m/s
        state.ground_course    = wrap_360(_buffer.pvt.head_mot * 1.0e-5f);       // Heading 2D deg * 100000
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.pvt.velN * 0.001f;
        state.velocity.y = _buffer.pvt.velE * 0.001f;
        state.velocity.z = _buffer.pvt.velD * 0.001f;
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.pvt.s_acc*0.001f;
        _new_speed = true;
        // dop
        if (_no_received_hdop) {
            state.hdop        = _buffer.pvt.p_dop;
            state.vdop        = _buffer.pvt.p_dop;
        }

        if (_buffer.pvt.fix_type >= 2) {
            state.last_gps_time_ms = AP_HAL::millis();
        }

        // time
        state.time_week_ms    = _buffer.pvt.itow;
        break;

    case MSG_TIMEGPS:
        if (_payload_length != sizeof(ubx_nav_timegps)) {
            return false;
        }
        _check_new_itow(_buffer.timegps.itow);
        if (_buffer.timegps.valid & UBX_PASSIVE_TIMEGPS_VALID_WEEK_MASK) {
            state.time_week_ms = _buffer.timegps.itow;
            state.time_week = _buffer.timegps.week;
            state.last_gps_time_ms = AP_HAL::millis();
        }
        break;

    case MSG_VELNED:
        if (_payload_length != sizeof(ubx_nav_velned)) {
            return false;
        }
        if (_have_pvt_msg) {
            // PVT carries the velocity itself
            break;
        }
        _check_new_itow(_buffer.velned.itow);
        _last_vel_time         = _buffer.velned.itow;
        state.ground_speed     = _buffer.velned.speed_2d*0.01f;          // m/s
        state.ground_course    = wrap_360(_buffer.velned.heading_2d * 1.0e-5f);       // Heading 2D deg * 100000
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.velned.ned_north * 0.01f;
        state.velocity.y = _buffer.velned.ned_east * 0.01f;
        state.velocity.z = _buffer.velned.ned_down * 0.01f;
        velocity_to_speed_course(state);
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.velned.speed_accuracy*0.01f;
        _new_speed = true;
        break;

    default:
        // a NAV message we don't decode
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}

/*
  detect a u-blox module streaming UBX. Identical framing to
  AP_GPS_UBLOX::_detect, but this driver has its own copy so that it
  can be built with the configuring u-blox driver compiled out
 */
bool
AP_GPS_UBLOX_PASSIVE::_detect(struct UBLOX_PASSIVE_detect_state &state, uint8_t data)
{
reset:
    switch (state.step) {
    case 1:
        if (PREAMBLE2 == data) {
            state.step++;
            break;
        }
        state.step = 0;
        FALLTHROUGH;
    case 0:
        if (PREAMBLE1 == data) {
            state.step++;
        }
        break;
    case 2:
        state.step++;
        state.ck_b = state.ck_a = data;
        break;
    case 3:
        state.step++;
        state.ck_b += (state.ck_a += data);
        break;
    case 4:
        state.step++;
        state.ck_b += (state.ck_a += data);
        state.payload_length = data;
        break;
    case 5:
        state.step++;
        state.ck_b += (state.ck_a += data);
        state.payload_counter = 0;
        break;
    case 6:
        state.ck_b += (state.ck_a += data);
        if (++state.payload_counter == state.payload_length) {
            state.step++;
        }
        break;
    case 7:
        state.step++;
        if (state.ck_a != data) {
            state.step = 0;
            goto reset;
        }
        break;
    case 8:
        state.step = 0;
        if (state.ck_b == data) {
            // a valid UBlox packet
            return true;
        }
        goto reset;
    }
    return false;
}

#endif  // AP_GPS_UBLOX_PASSIVE_ENABLED
