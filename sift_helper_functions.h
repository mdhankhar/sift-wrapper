/*
 * sift_helper_functions.h
 *
 *  Created on: Feb 23, 2013
 *      Author: Mukul
 */

#ifndef SIFT_HELPER_FUNCTIONS_H_
#define SIFT_HELPER_FUNCTIONS_H_

#include <stdarg.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>

// Data structure for a keypoint.  Lists of keypoints are linked
// by the "next" field.
typedef struct KeypointSt
{
    float row, col;         //Subpixel location of keypoint.
    float scale, ori;       // Scale and orientation (range [-PI,PI])
    unsigned char *descrip; // Vector of descriptor values
    struct KeypointSt *next;// Pointer to next keypoint in list
}*Keypoint;

// This function
void DumpObjectPoseKey(int num_object_images, char* object_path_prefix);

void FindMatches(const Keypoint keys_1, const Keypoint keys_2, CvPoint2D32f& coords, int& count);
Keypoint CheckForMatch(const Keypoint key, const Keypoint klist);
int DistSquared(Keypoint k1, Keypoint k2);
Keypoint ReadKeysOrDie(FILE *fp);
Keypoint ReadKeyFileOrDie(char *filename);

#endif /* SIFT_HELPER_FUNCTIONS_H_ */
