/*
 * configs.h
 *
 *  Created on: Jul 11, 2020
 *      Author: ahmed
 */

#ifndef SRC_CONFIGS_H_
#define SRC_CONFIGS_H_


/* ------------- Project Configurations ------------*/

// Enable or disable exercise specific code parts
#define FP5 0
#define FP6 1

// Use this define on my laptop due to different syntax to instantiate SIFT object
#define MY_LAPTOP 0

// Threshold to drop lidar readings as an absolute difference from the mean x value
#define THRESHOLD_X_MEAN 	0.1 // 10cm

// Min Distance. Used when calculating TTC Camera
#define MIN_DISTANCE 		100.0

// Choose to print steps progress or not
#define PRINT_STEPS	0


#endif /* SRC_CONFIGS_H_ */
