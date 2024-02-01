/*
 * xytheta.h
 *
 *  Created on: Aug 15, 2023
 *      Author: Zain Irsyad
 */

#ifndef XYTHETA_H_
#define XYTHETA_H_

#include <stdint.h>

typedef struct {
	float x;
	float y;
	float theta;
} xytheta_f32_t;

typedef struct {
	double x;
	double y;
	double theta;
} xytheta_f64_t;

typedef struct {
	int8_t x;
	int8_t y;
	float theta;
} xytheta_i8_t;

typedef struct {
	int16_t x;
	int16_t y;
	float theta;
} xytheta_i16_t;

typedef struct {
	int32_t x;
	int32_t y;
	float theta;
} xytheta_i32_t;

typedef struct {
	int64_t x;
	int64_t y;
	float t;
} xytheta_i64_t;

#endif /* XYTHETA_H_ */