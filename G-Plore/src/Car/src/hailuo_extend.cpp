/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 32.21.0 Mon Jan  6 11:33:01 2020.
 */

#include <string.h>

#include "hailuo_extend.h"
//#include "main.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

int hailuo_extend_tcm_gear_fd_pack(
    uint8_t *dst_p,
    const struct hailuo_extend_tcm_gear_fd_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[3] |= pack_left_shift_u8(src_p->tcm_gear_fd, 0u, 0xffu);

    return (8);
}

int hailuo_extend_tcm_gear_fd_unpack(
    struct hailuo_extend_tcm_gear_fd_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(dst_p, 0, sizeof(*dst_p));

    dst_p->tcm_gear_fd |= unpack_right_shift_u8(src_p[3], 0u, 0xffu);

    return (0);
}

uint8_t hailuo_extend_tcm_gear_fd_tcm_gear_fd_encode(double value)
{
    return (uint8_t)(value - -125.0);
}

double hailuo_extend_tcm_gear_fd_tcm_gear_fd_decode(uint8_t value)
{
    return ((double)value + -125.0);
}

bool hailuo_extend_tcm_gear_fd_tcm_gear_fd_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int hailuo_extend_engine_spedd_0x0_cf00400_pack(
    uint8_t *dst_p,
    const struct hailuo_extend_engine_spedd_0x0_cf00400_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[3] |= pack_left_shift_u16(src_p->engine_speed, 0u, 0xffu);
    dst_p[4] |= pack_right_shift_u16(src_p->engine_speed, 8u, 0xffu);

    return (8);
}

int hailuo_extend_engine_spedd_0x0_cf00400_unpack(
    struct hailuo_extend_engine_spedd_0x0_cf00400_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(dst_p, 0, sizeof(*dst_p));

    dst_p->engine_speed |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->engine_speed |= unpack_left_shift_u16(src_p[4], 8u, 0xffu);

    return (0);
}

uint16_t hailuo_extend_engine_spedd_0x0_cf00400_engine_speed_encode(double value)
{
    return (uint16_t)(value / 0.125);
}

double hailuo_extend_engine_spedd_0x0_cf00400_engine_speed_decode(uint16_t value)
{
    return ((double)value * 0.125);
}

bool hailuo_extend_engine_spedd_0x0_cf00400_engine_speed_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int hailuo_extend_vcu_fd_0x18_fd0821_pack(
    uint8_t *dst_p,
    const struct hailuo_extend_vcu_fd_0x18_fd0821_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->veh_mass, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->front_hydraulic_brake_fd, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->rear_hydraulic_brake_fd, 0u, 0xffu);
    dst_p[3] |= pack_left_shift_u8(src_p->drive_mode_fd, 0u, 0x03u);
    dst_p[3] |= pack_left_shift_u8(src_p->load_brake_fd, 2u, 0x0cu);
    dst_p[3] |= pack_left_shift_u8(src_p->compartment_up, 4u, 0x30u);
    dst_p[3] |= pack_left_shift_u8(src_p->compartment_down, 6u, 0xc0u);
    dst_p[4] |= pack_left_shift_u8(src_p->compartment_up_highest, 0u, 0x03u);
    dst_p[4] |= pack_left_shift_u8(src_p->compartment_up_lowest, 2u, 0x0cu);
    dst_p[4] |= pack_left_shift_u8(src_p->vehicle_fault_level, 4u, 0x30u);
    dst_p[5] |= pack_left_shift_u8(src_p->veh_spd, 0u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->fuel_signal, 0u, 0xffu);

    return (8);
}

int hailuo_extend_vcu_fd_0x18_fd0821_unpack(
    struct hailuo_extend_vcu_fd_0x18_fd0821_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(dst_p, 0, sizeof(*dst_p));

    dst_p->veh_mass |= unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->front_hydraulic_brake_fd |= unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->rear_hydraulic_brake_fd |= unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->drive_mode_fd |= unpack_right_shift_u8(src_p[3], 0u, 0x03u);
    dst_p->load_brake_fd |= unpack_right_shift_u8(src_p[3], 2u, 0x0cu);
    dst_p->compartment_up |= unpack_right_shift_u8(src_p[3], 4u, 0x30u);
    dst_p->compartment_down |= unpack_right_shift_u8(src_p[3], 6u, 0xc0u);
    dst_p->compartment_up_highest |= unpack_right_shift_u8(src_p[4], 0u, 0x03u);
    dst_p->compartment_up_lowest |= unpack_right_shift_u8(src_p[4], 2u, 0x0cu);
    dst_p->vehicle_fault_level |= unpack_right_shift_u8(src_p[4], 4u, 0x30u);
    dst_p->veh_spd |= unpack_right_shift_u8(src_p[5], 0u, 0xffu);
    dst_p->fuel_signal |= unpack_right_shift_u8(src_p[6], 0u, 0xffu);

    return (0);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_veh_mass_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_veh_mass_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_veh_mass_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_front_hydraulic_brake_fd_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_front_hydraulic_brake_fd_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_front_hydraulic_brake_fd_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_rear_hydraulic_brake_fd_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_rear_hydraulic_brake_fd_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_rear_hydraulic_brake_fd_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_drive_mode_fd_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_drive_mode_fd_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_drive_mode_fd_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_load_brake_fd_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_load_brake_fd_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_load_brake_fd_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_compartment_down_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_compartment_down_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_compartment_down_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_highest_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_highest_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_highest_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_lowest_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_lowest_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_compartment_up_lowest_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_vehicle_fault_level_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_vehicle_fault_level_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_vehicle_fault_level_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_veh_spd_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_veh_spd_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_veh_spd_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t hailuo_extend_vcu_fd_0x18_fd0821_fuel_signal_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_vcu_fd_0x18_fd0821_fuel_signal_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_vcu_fd_0x18_fd0821_fuel_signal_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int hailuo_extend_adm_vcu_control_0x18_fd0721_pack(
    uint8_t *dst_p,
    const struct hailuo_extend_adm_vcu_control_0x18_fd0721_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->amble_brake, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->compartment_control, 0u, 0x03u);
    dst_p[1] |= pack_left_shift_u8(src_p->emergency_brake, 2u, 0x04u);
    dst_p[1] |= pack_left_shift_u8(src_p->load_brake, 3u, 0x08u);
    dst_p[1] |= pack_left_shift_u8(src_p->engine_start, 4u, 0x10u);
    dst_p[1] |= pack_left_shift_u8(src_p->engine_stop, 5u, 0x20u);
    dst_p[1] |= pack_left_shift_u8(src_p->road_dryor_wet, 6u, 0x40u);
    dst_p[1] |= pack_left_shift_u8(src_p->switch_dynamicor_economical, 7u, 0x80u);
    dst_p[2] |= pack_left_shift_u8(src_p->slope, 0u, 0x03u);

    return (8);
}

int hailuo_extend_adm_vcu_control_0x18_fd0721_unpack(
    struct hailuo_extend_adm_vcu_control_0x18_fd0721_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(dst_p, 0, sizeof(*dst_p));

    dst_p->amble_brake |= unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->compartment_control |= unpack_right_shift_u8(src_p[1], 0u, 0x03u);
    dst_p->emergency_brake |= unpack_right_shift_u8(src_p[1], 2u, 0x04u);
    dst_p->load_brake |= unpack_right_shift_u8(src_p[1], 3u, 0x08u);
    dst_p->engine_start |= unpack_right_shift_u8(src_p[1], 4u, 0x10u);
    dst_p->engine_stop |= unpack_right_shift_u8(src_p[1], 5u, 0x20u);
    dst_p->road_dryor_wet |= unpack_right_shift_u8(src_p[1], 6u, 0x40u);
    dst_p->switch_dynamicor_economical |= unpack_right_shift_u8(src_p[1], 7u, 0x80u);
    dst_p->slope |= unpack_right_shift_u8(src_p[2], 0u, 0x03u);

    return (0);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_amble_brake_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_amble_brake_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_amble_brake_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_compartment_control_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_compartment_control_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_compartment_control_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_emergency_brake_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_emergency_brake_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_emergency_brake_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_load_brake_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_load_brake_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_load_brake_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_engine_start_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_engine_start_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_engine_start_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_engine_stop_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_engine_stop_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_engine_stop_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_road_dryor_wet_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_road_dryor_wet_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_road_dryor_wet_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_switch_dynamicor_economical_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_switch_dynamicor_economical_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_switch_dynamicor_economical_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0721_slope_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0721_slope_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0721_slope_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

int hailuo_extend_adm_vcu_control_0x18_fd0621_pack(
    uint8_t *dst_p,
    const struct hailuo_extend_adm_vcu_control_0x18_fd0621_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->acc_ped, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->acc_ped_enable, 0u, 0x03u);
    dst_p[1] |= pack_left_shift_u8(src_p->adm_fault_level, 2u, 0x0cu);
    dst_p[1] |= pack_left_shift_u8(src_p->fog_light, 4u, 0x30u);
    dst_p[1] |= pack_left_shift_u8(src_p->horn, 6u, 0xc0u);
    dst_p[2] |= pack_left_shift_u8(src_p->hydraulic_brake, 0u, 0xffu);
    dst_p[3] |= pack_left_shift_u8(src_p->high_beam_ligh, 0u, 0x03u);
    dst_p[3] |= pack_left_shift_u8(src_p->low_beam_light, 2u, 0x0cu);
    dst_p[3] |= pack_left_shift_u8(src_p->turn_signal, 4u, 0x30u);
    dst_p[3] |= pack_left_shift_u8(src_p->double_light, 6u, 0xc0u);
    dst_p[4] |= pack_left_shift_u8(src_p->night_light, 0u, 0x03u);
    dst_p[4] |= pack_left_shift_u8(src_p->defroster_control, 2u, 0x0cu);
    dst_p[4] |= pack_left_shift_u8(src_p->wiper_control, 4u, 0x30u);

    return (8);
}

int hailuo_extend_adm_vcu_control_0x18_fd0621_unpack(
    struct hailuo_extend_adm_vcu_control_0x18_fd0621_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(dst_p, 0, sizeof(*dst_p));

    dst_p->acc_ped |= unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->acc_ped_enable |= unpack_right_shift_u8(src_p[1], 0u, 0x03u);
    dst_p->adm_fault_level |= unpack_right_shift_u8(src_p[1], 2u, 0x0cu);
    dst_p->fog_light |= unpack_right_shift_u8(src_p[1], 4u, 0x30u);
    dst_p->horn |= unpack_right_shift_u8(src_p[1], 6u, 0xc0u);
    dst_p->hydraulic_brake |= unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->high_beam_ligh |= unpack_right_shift_u8(src_p[3], 0u, 0x03u);
    dst_p->low_beam_light |= unpack_right_shift_u8(src_p[3], 2u, 0x0cu);
    dst_p->turn_signal |= unpack_right_shift_u8(src_p[3], 4u, 0x30u);
    dst_p->double_light |= unpack_right_shift_u8(src_p[3], 6u, 0xc0u);
    dst_p->night_light |= unpack_right_shift_u8(src_p[4], 0u, 0x03u);
    dst_p->defroster_control |= unpack_right_shift_u8(src_p[4], 2u, 0x0cu);
    dst_p->wiper_control |= unpack_right_shift_u8(src_p[4], 4u, 0x30u);

    return (0);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_acc_ped_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_acc_ped_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_acc_ped_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_acc_ped_enable_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_acc_ped_enable_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_acc_ped_enable_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_adm_fault_level_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_adm_fault_level_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_adm_fault_level_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_fog_light_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_fog_light_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_fog_light_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_horn_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_horn_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_horn_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_hydraulic_brake_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_hydraulic_brake_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_hydraulic_brake_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_high_beam_ligh_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_high_beam_ligh_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_high_beam_ligh_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_low_beam_light_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_low_beam_light_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_low_beam_light_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_turn_signal_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_turn_signal_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_turn_signal_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_double_light_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_double_light_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_double_light_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_night_light_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_night_light_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_night_light_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_defroster_control_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_defroster_control_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_defroster_control_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t hailuo_extend_adm_vcu_control_0x18_fd0621_wiper_control_encode(double value)
{
    return (uint8_t)(value);
}

double hailuo_extend_adm_vcu_control_0x18_fd0621_wiper_control_decode(uint8_t value)
{
    return ((double)value);
}

bool hailuo_extend_adm_vcu_control_0x18_fd0621_wiper_control_is_in_range(uint8_t value)
{
    return (value <= 3u);
}
