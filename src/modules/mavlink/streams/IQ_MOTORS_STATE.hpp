/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef IQ_MOTORS_STATE_HPP
#define IQ_MOTORS_STATE_HPP

#include <uORB/topics/iq_motors_state.h>

class MavlinkStreamIQMotorsState : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamIQMotorsState(mavlink); }

    static constexpr const char *get_name_static() { return "IQ_MOTORS_STATE"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_IQ_MOTORS_STATE; }

    // const char *get_name() const { return MavlinkStreamIQMotorsState::get_name_static(); }
    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        // return MAVLINK_MSG_ID_IQ_MOTORS_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
        return _sub.advertised() ? MAVLINK_MSG_ID_IQ_MOTORS_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }

private:
    explicit MavlinkStreamIQMotorsState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _sub{ORB_ID(iq_motors_state)};

    /* do not allow top copying this class */
    // MavlinkStreamIQMotorsState(MavlinkStreamIQMotorsState &);
    // MavlinkStreamIQMotorsState& operator = (const MavlinkStreamIQMotorsState &);

// protected:
//     explicit MavlinkStreamIQMotorsState(Mavlink *mavlink) : MavlinkStream(mavlink) {}


    bool send() override
    {
        // orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_raw);
                    // memcpy(control, actuator_raw.control, sizeof control); // updates if control[4] matches

        // struct iq_motors_state_s _iq_motors_state;
        iq_motors_state_s _data{};

        if (_sub.copy(&_data)) {
            mavlink_iq_motors_state_t _msg_iq_motors_state{};  //make sure mavlink_ca_trajectory_t is the definition of your custom MAVLink message
            _sub.copy(&_data);

            _msg_iq_motors_state.timestamp = _data.timestamp;
            _msg_iq_motors_state.mean_velocity[0] = _data.mean_velocity[0];
            _msg_iq_motors_state.mean_velocity[1] = _data.mean_velocity[1];
            _msg_iq_motors_state.mean_volts[0] = _data.mean_volts[0];
            _msg_iq_motors_state.mean_volts[1] = _data.mean_volts[1];
            _msg_iq_motors_state.pulse_volts[0] = _data.pulse_volts[0];
            _msg_iq_motors_state.pulse_volts[1] = _data.pulse_volts[1];
            _msg_iq_motors_state.pulse_phase[0] = _data.pulse_phase[0];
            _msg_iq_motors_state.pulse_phase[1] = _data.pulse_phase[1];
            _msg_iq_motors_state.tec[0] = _data.tec[0];
            _msg_iq_motors_state.tec[1] = _data.tec[1];
            _msg_iq_motors_state.tuc[0] = _data.tuc[0];
            _msg_iq_motors_state.tuc[1] = _data.tuc[1];
            _msg_iq_motors_state.connection_state = _data.connection_state;

            mavlink_msg_iq_motors_state_send_struct(_mavlink->get_channel(), &_msg_iq_motors_state);
            PX4_WARN("MAVLINK IQ SENT");

            return true;
        }
        PX4_WARN("MAVLINK IQ NOT SENT");

        return false;
    }
};

#endif // IQ_MOTORS_STATE
