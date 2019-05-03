/*
 * gen_utils.h
 *
 *  Created on: Mar 21, 2013
 *
 *  General utilities unit. Different functions for different applications
 */

#ifndef GEN_UTILS_H_
#define GEN_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

typedef union {uint8_t b[2]; uint16_t w; int16_t wi;} uni_8x2_16;
typedef union {uint8_t b[4]; float32_t f; uint32_t v32;} uni_8x4_32;

// Macros for absolute of a number
#define ABS_V(X) (((X)>=0)?(X):(-(X)))

// Macros for maximum of two numbers
#define MAX_V(X,Y) (((X)>=(Y))?(X):(Y))

// Macros for minimum of two numbers
#define MIN_V(X,Y) (((X)<=(Y))?(X):(Y))

// Data access macroses
#define BYTEPTR(X) ((uint8_t *) &X)

// Compares strings. Returns true if same
bool compare_str(const char *str_sample, char *str_to_compare, uint8_t size);

// Calculates CRC-16 checksum
void set_crc16(uint8_t *buf, uint8_t size);

float32_t sqr_v(float32_t x);
float32_t abs_v(float32_t x);

// returns angle in range (0..180), x2 degrees, by floating-point any angle
uint8_t get_angle_x2degrees(float32_t fangle);

// returns 10^(db_val/10)
float32_t exp10_table_value(uint8_t db_val);

void copy_4bytes_to_ptr(uni_8x4_32 fv, uint8_t *ptr);
void copy_4bytes_from_ptr(uni_8x4_32 *pfv, uint8_t *ptr);
void copy_2bytes_to_ptr(uni_8x2_16 iv16, uint8_t *ptr);
void copy_2bytes_from_ptr(uni_8x2_16 *piv16, uint8_t *ptr);

float32_t calc_triangle_angle(float32_t r1, float32_t r2, float32_t r_opposite);

bool device_is_robot(uint8_t device_type);
bool device_is_hedgehog(uint8_t device_type);
bool device_is_beacon_class(uint8_t device_type);

uint8_t device_type_hedgehog_cor(uint8_t device_type, bool hedgehog);

bool dashboard_dev_unlocked(uint8_t ind);

uint8_t ustx_answer_dsize(uint8_t subtype);

uint32_t uuid_v32(uint8_t ofs);// Returns 32 bits of 96-bit device unique ID

#ifdef __cplusplus
}
#endif

#endif /* GEN_UTILS_H_ */
