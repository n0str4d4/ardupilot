#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Linux/UARTDriver.h>

#include <AP_Common/AP_Common.h>
#include <AP_Common/NMEA.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <AP_GPS/AP_GPS_NMEA.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/GPS_Backend.h>
// #include <AP_GPS_NMEA.h>
// #include <AP_GPS_NMEA_V2.h>
// #include <AP_GPS.h>
//#include "AP_GPS_Backend_V2.h"

//#include <AP_HAL/utility.h>
//#include <AP_Common/AP_Common.h>
//#include <AP_SerialManager/AP_SerialManager.h>
// a quiet nan for invalid values
#define QNAN nanf("GPS")

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)        (_x - '0')
#define hexdigit(x) ((x)>9?'A'+((x)-10):'0'+(x))



 enum GPS_Status {
        NO_GPS = 0,                  ///< No GPS connected/detected
        NO_FIX = 1,                  ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,           ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,           ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4,      ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK_FLOAT = 5, ///< Receiving valid messages and 3D RTK Float
        GPS_OK_FIX_3D_RTK_FIXED = 6, ///< Receiving valid messages and 3D RTK Fixed
    };

 /*
      The GPS_State structure is filled in by the backend driver as it
      parses each message from the GPS.
     */
struct GPS_State {
        uint8_t instance; // the instance number of this GPS

        // all the following fields must all be filled by the backend driver
        GPS_Status status;                  ///< driver fix status
        uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
        uint16_t time_week;                 ///< GPS week number
        Location location;                  ///< last fix location
        float ground_speed;                 ///< ground speed in m/s
        float ground_course;                ///< ground course in degrees, wrapped 0-360
        float gps_yaw;                      ///< GPS derived yaw information, if available (degrees)
        uint32_t gps_yaw_time_ms;           ///< timestamp of last GPS yaw reading
        bool  gps_yaw_configured;           ///< GPS is configured to provide yaw
        uint16_t hdop;                      ///< horizontal dilution of precision, scaled by a factor of 100 (155 means the HDOP value is 1.55)
        uint16_t vdop;                      ///< vertical dilution of precision, scaled by a factor of 100 (155 means the VDOP value is 1.55)
        uint8_t num_sats;                   ///< Number of visible satellites
        Vector3f velocity;                  ///< 3D velocity in m/s, in NED format
        float speed_accuracy;               ///< 3D velocity RMS accuracy estimate in m/s
        float horizontal_accuracy;          ///< horizontal RMS accuracy estimate in m
        float vertical_accuracy;            ///< vertical RMS accuracy estimate in m
        float gps_yaw_accuracy;           ///< heading accuracy of the GPS in degrees
        bool have_vertical_velocity;      ///< does GPS give vertical velocity? Set to true only once available.
        bool have_speed_accuracy;         ///< does GPS give speed accuracy? Set to true only once available.
        bool have_horizontal_accuracy;    ///< does GPS give horizontal position accuracy? Set to true only once available.
        bool have_vertical_accuracy;      ///< does GPS give vertical position accuracy? Set to true only once available.
        bool have_gps_yaw;                ///< does GPS give yaw? Set to true only once available.
        bool have_gps_yaw_accuracy;       ///< does the GPS give a heading accuracy estimate? Set to true only once available
        float undulation;                   //<height that WGS84 is above AMSL at the current location
        bool have_undulation;               ///<do we have a value for the undulation
        uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
        bool announced_detection;           ///< true once we have announced GPS has been seen to the user
        uint64_t last_corrected_gps_time_us;///< the system time we got the last corrected GPS timestamp, microseconds
        bool corrected_timestamp_updated;  ///< true if the corrected timestamp has been updated
        uint32_t lagged_sample_count;       ///< number of samples with 50ms more lag than expected

        // all the following fields must only all be filled by RTK capable backend drivers
        uint32_t rtk_time_week_ms;         ///< GPS Time of Week of last baseline in milliseconds
        uint16_t rtk_week_number;          ///< GPS Week Number of last baseline
        uint32_t rtk_age_ms;               ///< GPS age of last baseline correction in milliseconds  (0 when no corrections, 0xFFFFFFFF indicates overflow)
        uint8_t  rtk_num_sats;             ///< Current number of satellites used for RTK calculation
        uint8_t  rtk_baseline_coords_type; ///< Coordinate system of baseline. 0 == ECEF, 1 == NED
        int32_t  rtk_baseline_x_mm;        ///< Current baseline in ECEF x or NED north component in mm
        int32_t  rtk_baseline_y_mm;        ///< Current baseline in ECEF y or NED east component in mm
        int32_t  rtk_baseline_z_mm;        ///< Current baseline in ECEF z or NED down component in mm
        uint32_t rtk_accuracy;             ///< Current estimate of 3D baseline accuracy (receiver dependent, typical 0 to 9999)
        int32_t  rtk_iar_num_hypotheses;   ///< Current number of integer ambiguity hypotheses
        
        // UBX Relative Position and Heading message information
        float relPosHeading;               ///< Reported Heading in degrees
        float relPosLength;                ///< Reported Position horizontal distance in meters
        float relPosD;                     ///< Reported Vertical distance in meters
        float accHeading;                  ///< Reported Heading Accuracy in degrees
        uint32_t relposheading_ts;        ///< True if new data has been received since last time it was false
    };


const AP_HAL::HAL& hal = AP_HAL::get_HAL();                     //Declare "hal" reference variable. This variable is pointing to Ap_HAL::HAL class's object. Here, AP_HAL is library and HAL is a class in that library. This reference variable can be used to get access to hardware specific functions.                     
GPS_State default_state = {0}; 

class AP_GPS_NMEA_Custom {

public:
    AP_GPS_NMEA_Custom(): state(default_state), port(nullptr) {}; // Implement the default constructor
    AP_GPS_NMEA_Custom(GPS_State &_state, Linux::UARTDriver *_port): state(_state), port(_port) {};
    bool read();

private:
 /// Coding for the GPS sentences that the parser handles
    enum _sentence_types : uint16_t {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,
        _GPS_SENTENCE_GGA = 64,
        _GPS_SENTENCE_VTG = 96,
        _GPS_SENTENCE_HDT = 128,
        _GPS_SENTENCE_PHD = 138, // extension for AllyStar GPS modules
        _GPS_SENTENCE_THS = 160, // True heading with quality indicator, available on Trimble MB-Two
        _GPS_SENTENCE_KSXT = 170, // extension for Unicore, 21 fields
        _GPS_SENTENCE_AGRICA = 193, // extension for Unicore, 65 fields
        _GPS_SENTENCE_VERSIONA = 270, // extension for Unicore, version, 10 fields
        _GPS_SENTENCE_UNIHEADINGA = 290, // extension for Unicore, uniheadinga, 20 fields
        _GPS_SENTENCE_OTHER = 0
    };

    /// Update the decode state machine with a new character
    ///
    /// @param	c		The next character in the NMEA input stream
    /// @returns		True if processing the character has resulted in
    ///					an update to the GPS state
    ///
    bool                        _decode(char c);

    /// Parses the @p as a NMEA-style decimal number with
    /// up to 3 decimal digits.
    ///
    /// @returns		The value expressed by the string in @p,
    ///					multiplied by 100.
    ///
    static int32_t _parse_decimal_100(const char *p);

    /// Parses the current term as a NMEA-style degrees + minutes
    /// value with up to four decimal digits.
    ///
    /// This gives a theoretical resolution limit of around 1cm.
    ///
    /// @returns		The value expressed by the string in _term,
    ///					multiplied by 1e7.
    ///
    uint32_t    _parse_degrees();

    /// Processes the current term when it has been deemed to be
    /// complete.
    ///
    /// Each GPS message is broken up into terms separated by commas.
    /// Each term is then processed by this function as it is received.
    ///
    /// @returns		True if completing the term has resulted in
    ///					an update to the GPS state.
    bool                        _term_complete();

    /// return true if we have a new set of NMEA messages
    bool _have_new_message(void);


    Linux::UARTDriver *port;
    GPS_State &state;
    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    uint32_t _crc32;                                            ///< CRC for unicore messages
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[30];                                                     ///< buffer for the current term within the current sentence
    uint16_t _sentence_type;                                     ///< the sentence type currently being processed
    bool _is_unicore;                                           ///< true if in a unicore '#' sentence
    uint16_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    uint16_t _sentence_length;
    bool _sentence_done;                                        ///< set when a sentence has been fully decoded

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    float   _new_gps_yaw;                                        ///< yaw parsed from a term
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term
    uint8_t _new_quality_indicator;                                     ///< GPS quality indicator parsed from a term

    uint32_t _last_RMC_ms;
    uint32_t _last_GGA_ms;
    uint32_t _last_VTG_ms;
    uint32_t _last_yaw_ms;
    uint32_t _last_vvelocity_ms;
    uint32_t _last_vaccuracy_ms;
    uint32_t _last_3D_velocity_ms;
    uint32_t _last_KSXT_pos_ms;
    uint32_t _last_AGRICA_ms;
    uint32_t _last_fix_ms;

    /*
      the $PHD message is an extension from AllyStar that gives
      vertical velocity and more accuracy estimates. It is designed as
      a mapping from ublox UBX protocol messages to NMEA. So class 1,
      message 12 is a mapping to NMEA of the NAV-VELNED UBX message
      and contains the same fields. Class 1 message 26 is called
      "NAV-PVERR", but does not correspond to a UBX message

      example:
        $PHD,01,12,TIIITTITT,,245808000,0,0,0,0,0,10260304,0,0*27
        $PHD,01,26,TTTTTTT,,245808000,877,864,1451,11,11,17*17
     */
    struct {
        uint8_t msg_class;
        uint8_t msg_id;
        uint32_t itow;
        int32_t fields[8];
    } _phd;

    /*
      The KSXT message is an extension from Unicore that gives 3D velocity and yaw
      example: $KSXT,20211016083433.00,116.31296102,39.95817066,49.4911,223.57,-11.32,330.19,0.024,,1,3,28,27,,,,-0.012,0.021,0.020,,*2D
     */
    struct {
        double fields[21];
    } _ksxt;

     bool _expect_agrica;

    // last time we sent type specific config strings
    uint32_t last_config_ms;

    // send type specific config strings
    void send_config(void);

};



bool AP_GPS_NMEA_Custom::read(void)
{
    int16_t numc;
    bool parsed = false;

    //send_config();

    numc = port->available();
    while (numc--) {
        char c = port->read();

        if (_decode(c)) {
            parsed = true;
        }
    }
    return parsed;
}

/*
  decode one character, return true if we have successfully completed a sentence, false otherwise
 */
bool AP_GPS_NMEA_Custom::_decode(char c)
{
    _sentence_length++;
        
    switch (c) {
    case ';':
        // header separator for unicore
        if (!_is_unicore) {
            return false;
        }
        FALLTHROUGH;
    case ',': // term terminators
        _parity ^= c;
        if (_is_unicore) {
            _crc32 = crc_crc32(_crc32, (const uint8_t *)&c, 1);
        }
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*': {
        if (_sentence_done) {
            return false;
        }
        bool valid_sentence = false;
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;
    }

    case '$': // sentence begin
    case '#': // unicore message begin
        _is_unicore = (c == '#');
        _term_number = _term_offset = 0;
        _parity = 0;
        _crc32 = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _sentence_length = 1;
        _sentence_done = false;
        _new_gps_yaw = QNAN;
        return false;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term) {
        _parity ^= c;
        if (_is_unicore) {
            _crc32 = crc_crc32(_crc32, (const uint8_t *)&c, 1);
        }
    }

    return false;
}

int32_t AP_GPS_NMEA_Custom::_parse_decimal_100(const char *p)
{
    char *endptr = nullptr;
    long ret = 100 * strtol(p, &endptr, 10);
    int sign = ret < 0 ? -1 : 1;

    if (ret >= (long)INT32_MAX) {
        return INT32_MAX;
    }
    if (ret <= (long)INT32_MIN) {
        return INT32_MIN;
    }
    if (endptr == nullptr || *endptr != '.') {
        return ret;
    }

    if (isdigit(endptr[1])) {
        ret += sign * 10 * DIGIT_TO_VAL(endptr[1]);
        if (isdigit(endptr[2])) {
            ret += sign * DIGIT_TO_VAL(endptr[2]);
            if (isdigit(endptr[3])) {
                ret += sign * (DIGIT_TO_VAL(endptr[3]) >= 5);
            }
        }
    }
    return ret;
}

/*
  parse a NMEA latitude/longitude degree value. The result is in degrees*1e7
 */
uint32_t AP_GPS_NMEA_Custom::_parse_degrees()
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;

    // scan for decimal point or end of field
    for (p = _term; *p && isdigit(*p); p++)
        ;
    q = _term;

    // convert degrees
    while ((p - q) > 2 && *q) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }

    // convert minutes
    while (p > q && *q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }

    // convert fractional minutes
    if (*p == '.') {
        q = p + 1;
        float frac_scale = 0.1f;
        while (*q && isdigit(*q)) {
            frac_min += DIGIT_TO_VAL(*q) * frac_scale;
            q++;
            frac_scale *= 0.1f;
        }
    }
    ret = (deg * (int32_t)10000000UL);
    ret += (min * (int32_t)10000000UL / 60);
    ret += (int32_t) (frac_min * (1.0e7f / 60.0f));
    return ret;
}

/*
  see if we have a new set of NMEA messages
 */
bool AP_GPS_NMEA_Custom::_have_new_message()
{
    if (_last_RMC_ms == 0 ||
        _last_GGA_ms == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _last_RMC_ms > 150 ||
        now - _last_GGA_ms > 150) {
        return false;
    }
    if (_last_VTG_ms != 0 && 
        now - _last_VTG_ms > 150) {
        return false;
    }

    /*
      if we have seen a message with 3D velocity data messages then
      wait for them again. This is important as the
      have_vertical_velocity field will be overwritten by
      fill_3d_velocity()
     */
    if (_last_vvelocity_ms != 0 &&
        now - _last_vvelocity_ms > 150 &&
        now - _last_vvelocity_ms < 1000) {
        // waiting on a message with velocity
        return false;
    }
    if (_last_vaccuracy_ms != 0 &&
        now - _last_vaccuracy_ms > 150 &&
        now - _last_vaccuracy_ms < 1000) {
        // waiting on a message with velocity accuracy
        return false;
    }

    // prevent these messages being used again
    if (_last_VTG_ms != 0) {
        _last_VTG_ms = 1;
    }

    if (now - _last_yaw_ms > 300) {
        // we have lost GPS yaw
        state.have_gps_yaw = false;
    }

    if (now - _last_KSXT_pos_ms > 500) {
        // we have lost KSXT
        _last_KSXT_pos_ms = 0;
    }

#if AP_GPS_NMEA_UNICORE_ENABLED
    if (now - _last_AGRICA_ms > 500) {
        if (_last_AGRICA_ms != 0) {
            // we have lost AGRICA
            state.have_gps_yaw = false;
            state.have_vertical_velocity = false;
            state.have_speed_accuracy = false;
            state.have_horizontal_accuracy = false;
            state.have_vertical_accuracy = false;
            state.have_undulation = false;
            _last_AGRICA_ms = 0;
        }
    }
#endif // AP_GPS_NMEA_UNICORE_ENABLED

    _last_fix_ms = now;

    _last_GGA_ms = 1;
    _last_RMC_ms = 1;
    return true;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_NMEA_Custom::_term_complete()
{
    // handle the last term in a message
    if (_is_checksum_term) {
        _sentence_done = true;
        const uint32_t crc = strtoul(_term, nullptr, 16);
        const bool crc_ok = _is_unicore? (_crc32 == crc) : (_parity == crc);
        if (crc_ok) {
            uint32_t now = AP_HAL::millis();
            switch (_sentence_type) {
            case _GPS_SENTENCE_RMC:
                _last_RMC_ms = now;
                //time                        = _new_time;
                //date                        = _new_date;
                if (_last_KSXT_pos_ms == 0 && _last_AGRICA_ms == 0) {
                    state.location.lat     = _new_latitude;
                    state.location.lng     = _new_longitude;
                }
                if (_last_3D_velocity_ms == 0 ||
                    now - _last_3D_velocity_ms > 1000) {
                    state.ground_speed     = _new_speed*0.01f;
                    state.ground_course    = wrap_360(_new_course*0.01f);
                }
                if (state.status >= GPS_Status::GPS_OK_FIX_3D) {
                    //make_gps_time(_new_date, _new_time * 10);
                    if (_last_AGRICA_ms != 0) {
                        //state.time_week_ms = _last_itow_ms;
                    }
                }
                //set_uart_timestamp(_sentence_length);
                state.last_gps_time_ms = now;
                if (_last_vvelocity_ms == 0 ||
                    now - _last_vvelocity_ms > 1000) {
                    //fill_3d_velocity();
                }
                break;
            case _GPS_SENTENCE_GGA:
                _last_GGA_ms = now;
                if (_last_KSXT_pos_ms == 0 && _last_AGRICA_ms == 0) {
                    //set_alt_amsl_cm(state, _new_altitude);
                    state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;
                }
                state.num_sats      = _new_satellite_count;
                state.hdop          = _new_hdop;
                switch(_new_quality_indicator) {
                case 0: // Fix not available or invalid
                    state.status = GPS_Status::NO_FIX;
                    break;
                case 1: // GPS SPS Mode, fix valid
                    state.status = GPS_Status::GPS_OK_FIX_3D;
                    break;
                case 2: // Differential GPS, SPS Mode, fix valid
                    state.status = GPS_Status::GPS_OK_FIX_3D_DGPS;
                    break;
                case 3: // GPS PPS Mode, fix valid
                    state.status = GPS_Status::GPS_OK_FIX_3D;
                    break;
                case 4: // Real Time Kinematic. System used in RTK mode with fixed integers
                    state.status = GPS_Status::GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 5: // Float RTK. Satellite system used in RTK mode, floating integers
                    state.status = GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 6: // Estimated (dead reckoning) Mode
                    state.status = GPS_Status::NO_FIX;
                    break;
                default://to maintain compatibility with MAV_GPS_INPUT and others
                    state.status = GPS_Status::GPS_OK_FIX_3D;
                    break;
                }
                break;
            case _GPS_SENTENCE_VTG:
                _last_VTG_ms = now;
                if (_last_3D_velocity_ms == 0 ||
                    now - _last_3D_velocity_ms > 1000) {
                    state.ground_speed  = _new_speed*0.01f;
                    state.ground_course = wrap_360(_new_course*0.01f);
                    if (_last_vvelocity_ms == 0 ||
                        now - _last_vvelocity_ms > 1000) {
                        //fill_3d_velocity();
                    }
                }
                // VTG has no fix indicator, can't change fix status
                break;
            case _GPS_SENTENCE_HDT:
            case _GPS_SENTENCE_THS:
                if (_last_AGRICA_ms != 0 || _expect_agrica) {
                    // use AGRICA
                    break;
                }
                if (isnan(_new_gps_yaw)) {
                    // empty sentence
                    break;
                }
                _last_yaw_ms = now;
                state.gps_yaw = wrap_360(_new_gps_yaw*0.01f);
                state.have_gps_yaw = true;
                state.gps_yaw_time_ms = now;
                // remember that we are setup to provide yaw. With
                // a NMEA GPS we can only tell if the GPS is
                // configured to provide yaw when it first sends a
                // HDT sentence.
                state.gps_yaw_configured = true;
                break;
            case _GPS_SENTENCE_PHD:
                if (_last_AGRICA_ms != 0) {
                    // prefer AGRICA
                    break;
                }
                if (_phd.msg_id == 12) {
                    state.velocity.x = _phd.fields[0] * 0.01;
                    state.velocity.y = _phd.fields[1] * 0.01;
                    state.velocity.z = _phd.fields[2] * 0.01;
                    state.have_vertical_velocity = true;
                    _last_vvelocity_ms = now;
                    // we prefer a true 3D velocity when available
                    //velocity_to_speed_course(state);
                    _last_3D_velocity_ms = now;
                } else if (_phd.msg_id == 26) {
                    state.horizontal_accuracy = MAX(_phd.fields[0],_phd.fields[1]) * 0.001;
                    state.have_horizontal_accuracy = true;
                    state.vertical_accuracy = _phd.fields[2] * 0.001;
                    state.have_vertical_accuracy = true;
                    state.speed_accuracy = MAX(_phd.fields[3],_phd.fields[4]) * 0.001;
                    state.have_speed_accuracy = true;
                    _last_vaccuracy_ms = now;
                }
                break;
            case _GPS_SENTENCE_KSXT:
                if (_last_AGRICA_ms != 0 || _expect_agrica) {
                    // prefer AGRICA
                    break;
                }
                state.location.lat     = _ksxt.fields[2]*1.0e7;
                state.location.lng     = _ksxt.fields[1]*1.0e7;
                //set_alt_amsl_cm(state, _ksxt.fields[3]*1.0e2);
                _last_KSXT_pos_ms = now;
                if (_ksxt.fields[9] >= 1) {
                    // we have 3D fix
                    constexpr float kmh_to_mps = 1.0 / 3.6;
                    state.velocity.y = _ksxt.fields[16] * kmh_to_mps;
                    state.velocity.x = _ksxt.fields[17] * kmh_to_mps;
                    state.velocity.z = _ksxt.fields[18] * -kmh_to_mps;
                    state.have_vertical_velocity = true;
                    _last_vvelocity_ms = now;
                    // we prefer a true 3D velocity when available
                    //velocity_to_speed_course(state);
                    _last_3D_velocity_ms = now;
                }
                if (is_equal(3.0f, float(_ksxt.fields[10]))) {
                    // have good yaw (from RTK fixed moving baseline solution)
                    _last_yaw_ms = now;
                    state.gps_yaw = _ksxt.fields[4];
                    state.have_gps_yaw = true;
                    state.gps_yaw_time_ms = now;
                    state.gps_yaw_configured = true;
                }
                break;
#if AP_GPS_NMEA_UNICORE_ENABLED
            case _GPS_SENTENCE_AGRICA: {
                const auto &ag = _agrica;
                _last_AGRICA_ms = now;
                _last_vvelocity_ms = now;
                _last_vaccuracy_ms = now;
                _last_3D_velocity_ms = now;
                state.location.lat = ag.lat*1.0e7;
                state.location.lng = ag.lng*1.0e7;
                state.undulation   = -ag.undulation;
                state.have_undulation = true;
                set_alt_amsl_cm(state, ag.alt*1.0e2);
                state.velocity = ag.vel_NED;
                velocity_to_speed_course(state);
                state.speed_accuracy = ag.vel_stddev.length();
                state.horizontal_accuracy = ag.pos_stddev.xy().length();
                state.vertical_accuracy = ag.pos_stddev.z;
                state.have_vertical_velocity = true;
                state.have_speed_accuracy = true;
                state.have_horizontal_accuracy = true;
                state.have_vertical_accuracy = true;
                check_new_itow(ag.itow, _sentence_length);
                break;
            }
            case _GPS_SENTENCE_VERSIONA: {
                _have_unicore_versiona = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "NMEA %s %s %s",
                              _versiona.type,
                              _versiona.version,
                              _versiona.build_date);
                break;
            }
            case _GPS_SENTENCE_UNIHEADINGA: {
#if GPS_MOVING_BASELINE
                const auto &ag = _agrica;
                const auto &uh = _uniheadinga;
                if (now - _last_AGRICA_ms > 500 || ag.heading_status != 4) {
                    // we need heading_status from AGRICA
                    state.have_gps_yaw = false;
                    break;
                }
                const float dist = uh.baseline_length;
                const float bearing = uh.heading;
                const float alt_diff = dist*tanf(radians(-uh.pitch));
                state.relPosHeading = bearing;
                state.relPosLength = dist;
                state.relPosD = alt_diff;
                state.relposheading_ts = now;
                if (calculate_moving_base_yaw(bearing, dist, alt_diff)) {
                    state.have_gps_yaw_accuracy = true;
                    state.gps_yaw_accuracy = uh.heading_sd;
                    _last_yaw_ms = now;
                }
                state.gps_yaw_configured = true;
#endif // GPS_MOVING_BASELINE
                break;
            }
#endif // AP_GPS_NMEA_UNICORE_ENABLED
            }
            // see if we got a good message
            return _have_new_message();
        }
        // we got a bad message, ignore it
        return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        /*
          special case for $PHD message
         */
        if (strcmp(_term, "PHD") == 0) {
            _sentence_type = _GPS_SENTENCE_PHD;
            return false;
        }
        if (strcmp(_term, "KSXT") == 0) {
            _sentence_type = _GPS_SENTENCE_KSXT;
            return false;
        }
#if AP_GPS_NMEA_UNICORE_ENABLED
        if (strcmp(_term, "AGRICA") == 0 && _expect_agrica) {
            _sentence_type = _GPS_SENTENCE_AGRICA;
            return false;
        }
        if (strcmp(_term, "VERSIONA") == 0) {
            _sentence_type = _GPS_SENTENCE_VERSIONA;
            return false;
        }
        if (strcmp(_term, "UNIHEADINGA") == 0 && _expect_agrica) {
            _sentence_type = _GPS_SENTENCE_UNIHEADINGA;
            return false;
        }
#endif
        /*
          The first two letters of the NMEA term are the talker
          ID. The most common is 'GP' but there are a bunch of others
          that are valid. We accept any two characters here.
         */
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = _GPS_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "RMC") == 0) {
            _sentence_type = _GPS_SENTENCE_RMC;
        } else if (strcmp(term_type, "GGA") == 0) {
            _sentence_type = _GPS_SENTENCE_GGA;
        } else if (strcmp(term_type, "HDT") == 0) {
            _sentence_type = _GPS_SENTENCE_HDT;
        } else if (strcmp(term_type, "THS") == 0) {
            _sentence_type = _GPS_SENTENCE_THS;
        } else if (strcmp(term_type, "VTG") == 0) {
            _sentence_type = _GPS_SENTENCE_VTG;
        } else {
            _sentence_type = _GPS_SENTENCE_OTHER;
        }
        return false;
    }

    // 32 = RMC, 64 = GGA, 96 = VTG, 128 = HDT, 160 = THS
    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
        switch (_sentence_type + _term_number) {
        // operational status
        //
        case _GPS_SENTENCE_RMC + 2: // validity (RMC)
            break;
        case _GPS_SENTENCE_GGA + 6: // Fix data (GGA)
            if (_term[0] > '0') {
                _new_quality_indicator = _term[0] - '0';
            } else {
                _new_quality_indicator = 0;
            }
            break;
        case _GPS_SENTENCE_GGA + 7: // satellite count (GGA)
            _new_satellite_count = atol(_term);
            break;
        case _GPS_SENTENCE_GGA + 8: // HDOP (GGA)
            _new_hdop = (uint16_t)_parse_decimal_100(_term);
            break;

        // time and date
        //
        case _GPS_SENTENCE_RMC + 1: // Time (RMC)
        case _GPS_SENTENCE_GGA + 1: // Time (GGA)
            _new_time = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 9: // Date (GPRMC)
            _new_date = atol(_term);
            break;

        // location
        //
        case _GPS_SENTENCE_RMC + 3: // Latitude
        case _GPS_SENTENCE_GGA + 2:
            _new_latitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 4: // N/S
        case _GPS_SENTENCE_GGA + 3:
            if (_term[0] == 'S')
                _new_latitude = -_new_latitude;
            break;
        case _GPS_SENTENCE_RMC + 5: // Longitude
        case _GPS_SENTENCE_GGA + 4:
            _new_longitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 6: // E/W
        case _GPS_SENTENCE_GGA + 5:
            if (_term[0] == 'W')
                _new_longitude = -_new_longitude;
            break;
        case _GPS_SENTENCE_GGA + 9: // Altitude (GPGGA)
            _new_altitude = _parse_decimal_100(_term);
            break;

        // course and speed
        //
        case _GPS_SENTENCE_RMC + 7: // Speed (GPRMC)
        case _GPS_SENTENCE_VTG + 5: // Speed (VTG)
            _new_speed = (_parse_decimal_100(_term) * 514) / 1000;       // knots-> m/sec, approximates * 0.514
            break;
        case _GPS_SENTENCE_HDT + 1: // Course (HDT)
            _new_gps_yaw = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_THS + 1: // Course (THS)
            _new_gps_yaw = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 8: // Course (GPRMC)
        case _GPS_SENTENCE_VTG + 1: // Course (VTG)
            _new_course = _parse_decimal_100(_term);
            break;

        case _GPS_SENTENCE_PHD + 1: // PHD class
            _phd.msg_class = atol(_term);
            break;
        case _GPS_SENTENCE_PHD + 2: // PHD message
            _phd.msg_id = atol(_term);
            break;
        case _GPS_SENTENCE_PHD + 5: // PHD message, itow
            _phd.itow = strtoul(_term, nullptr, 10);
            break;
        case _GPS_SENTENCE_PHD + 6 ... _GPS_SENTENCE_PHD + 11: // PHD message, fields
            _phd.fields[_term_number-6] = atol(_term);
            break;
        case _GPS_SENTENCE_KSXT + 1 ... _GPS_SENTENCE_KSXT + 22: // KSXT message, fields
            _ksxt.fields[_term_number-1] = atof(_term);
            break;
#if AP_GPS_NMEA_UNICORE_ENABLED
        case _GPS_SENTENCE_AGRICA + 1 ... _GPS_SENTENCE_AGRICA + 65: // AGRICA message
            parse_agrica_field(_term_number, _term);
            break;
        case _GPS_SENTENCE_VERSIONA + 1 ... _GPS_SENTENCE_VERSIONA + 20:
            parse_versiona_field(_term_number, _term);
            break;
#if GPS_MOVING_BASELINE
        case _GPS_SENTENCE_UNIHEADINGA + 1 ... _GPS_SENTENCE_UNIHEADINGA + 28: // UNIHEADINGA message
            parse_uniheadinga_field(_term_number, _term);
            break;
#endif
#endif
        }
    }

    return false;
}


class GPSModuleTest {
    friend class AP_GPS_NMEA_Custom;
public:
    void setup();
    void loop();

private:

    Linux::UARTDriver* NMEA_GPS_UART;
    GPS_State* NMEA_GPS_STATE;
    AP_GPS_NMEA_Custom* NMEA_Backend;// AP_GPS_NMEA(*this, params[instance], state[instance], _port[instance]);
    uint32_t last_read_ms, now;
    
    //AP_GPS* NMEA_GPS;
};

void GPSModuleTest::setup() {
    // Initialize SerialManager to get the GPS UART port
    //NMEA_GPS_UART = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_GPS);
    //NMEA_GPS_UART = hal.serial(0).set_de;
    

    NMEA_GPS_UART = (Linux::UARTDriver*) hal.serial(3);
    NMEA_GPS_UART->set_device_path("/dev/ttyS0");
	// if not relying on Serial Manager to find the serial interface that supports/talk GPS protocol
    // NMEA_GPS_UART = hal.serial(0); or hal->serial(0);

    if (NMEA_GPS_UART != nullptr) {
        // Set baud rate to 9600 and initialize the GPS UART
        NMEA_GPS_UART->begin(9600);
        // having the flow control, Character size being setup on system startup from the linux shell
        // we might not need to do this here, but still, try the DISABLED/NONE flow control 
        NMEA_GPS_UART->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        hal.console->printf("GPS UART initialized at 9600 baud\n");

        //NMEA_GPS_STATE->instance = 0;

        //NMEA_Backend = AP_GPS_NMEA_Custom(NMEA_GPS_STATE, NMEA_GPS_UART);
        
    } else {
        hal.console->printf("Failed to initialize GPS UART\n");

    }

    // return NEW_NOTHROW AP_GPS_NMEA(*this, params[instance], state[instance], _port[instance]);

}

void GPSModuleTest::loop() {
    now = AP_HAL::millis();

    if (NMEA_GPS_UART != nullptr && NMEA_GPS_UART->available()) {
        char gps_data[100];  // Buffer to hold GPS data
        uint8_t i = 0;


        // Read from GPS UART until a full sentence is captured
        while (NMEA_GPS_UART->available() && i < sizeof(gps_data) - 1) {
            gps_data[i++] = NMEA_GPS_UART->read();
        }
        gps_data[i] = '\0';  // Null-terminate the string

        // Print the raw NMEA sentence
        last_read_ms = AP_HAL::millis();
        hal.console->printf("@ %i, Received GPS Data: %s\n", last_read_ms, gps_data);

        // bool result = NMEA_Backend->read();
        // if(result){
        //     hal.console->printf("GPS LAT= %f,  GPS LONG=%f,  GPS Fix=%s", 
        //                     NMEA_GPS_STATE->location.lat,
        //                     NMEA_GPS_STATE->location.lng,
        //                     NMEA_GPS_STATE->status)
        // }

    }

    //re-init uart 
    if(now - last_read_ms > 4000u )
    {
        NMEA_GPS_UART->end();
        NMEA_GPS_UART->begin(9600);
        // having the flow control, Character size being setup on system startup from the linux shell
        // we might not need to do this here, but still, try the DISABLED/NONE flow control 
        NMEA_GPS_UART->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        hal.console->printf("GPS UART re-initialized at 9600 baud\n");
    }
}

// Instantiate and use the GPSModuleTest in ArduPilot's architecture
GPSModuleTest gps_module;

void setup() {
    gps_module.setup();
}

void loop() {
    gps_module.loop();
    hal.scheduler->delay(100); // Small delay to avoid overwhelming the serial buffer
}



// Register above functions in HAL board level
AP_HAL_MAIN();