/*
 * polar_type.h
 *
 *  Created on: Sep 20, 2023
 *      Author: zainir17
 */

#ifndef POLAR_TYPE_H_
#define POLAR_TYPE_H_

#include <stdint.h>

typedef struct {
	float r;
	float theta;
} polar_f32_t;

typedef struct {
	double r;
	double theta;
} polar_f64_t;

#endif /* POLAR_TYPE_H_ */