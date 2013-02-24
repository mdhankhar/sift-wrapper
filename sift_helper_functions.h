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

class SingleObjectPosesInfo
{
public:
    SingleObjectPosesInfo(int best_pose_match_coordinates_x = 0, int best_pose_match_coordinates_y = 0, int best_pose_match_count = 0);

    ~SingleObjectPosesInfo();

    void SetBestMatchedObjectPoseInfo(int best_pose_match_coordinates_x, int best_pose_match_coordinates_y, int best_pose_match_count);

    void Initialize(char* object_name, int num_object_poses, char* object_path_prefix);

    void LoadAllObjectPoseKeypointsFromFiles();


    void GetBestObjectPoseMatch(Keypoint scene_keypoints);


    CvPoint2D32f best_pose_match_coordinates_2D32f_;
    int best_pose_match_count_;
    char object_name_[256];

private:
    Keypoint object_pose_keypoints_array_[100]; //object pose keys array
    int num_object_poses_;
    char object_path_prefix_[256];

};



// This function
void DumpObjectPoseKey(int num_object_images, char* object_path_prefix);

void FindMatches(const Keypoint keys_1, const Keypoint keys_2, CvPoint2D32f& coords, int& count);
Keypoint CheckForMatch(const Keypoint key, const Keypoint klist);
int DistSquared(Keypoint k1, Keypoint k2);
Keypoint ReadKeysOrDie(FILE *fp);
Keypoint ReadKeyFileOrDie(char *filename);

#endif /* SIFT_HELPER_FUNCTIONS_H_ */
