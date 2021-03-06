/*
 * SIFTWrapper.cpp
 *
 *  Created on: Jan 17, 2013
 *      Author: Mukul
 */

#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "sift_helper_functions.h"

// this param defines how many sift keypoints must match to an object
// to declare that the object has been identified in a scene image
#define OBJECT_MIN_KEYPOINT_MATCHES 5



int main(int argc, char**argv)
{

    if (argc < 3)
    {
        printf("USAGE:");
        printf("STEP 1: To learn objects:\n");
        printf("Enter 2 params: <num object images> <path/prefix to object image sequence ex. myobjects/objA_ (without sequence number or file extension 1.pgm or 2.pgm)>\n");
        printf("\nSTEP 2: To detect learned objects in live camera feed:\n");
        printf("Enter 4 params minimum: <total number of objects>\n");
        printf("<object 1 name> <object 1 num of pose images> <path/prefix to object image sequence ex. myobjects/objA_ (without sequence number or file extension 1.pgm or 2.pgm)>\n");
        printf("<object 2 name> <object 2 num of pose images> <path/prefix to object image sequence ex. myobjects/objB_ (without sequence number or file extension 1.pgm or 2.pgm)>\n");

        exit(0);
    }

    // STEP 1: Learn objects
    if (argc == 3)
    {
        int num_object_images = atoi(argv[1]);
        char* object_path_prefix = argv[2];
        // load object images
        printf("%d object poses with prefix:%s\n", num_object_images, object_path_prefix);
        DumpObjectPoseKey(num_object_images, object_path_prefix);
        return 0;
    }

    // STEP 2: Identify learned objects in live camera feed
    int total_num_objects_in_database = atoi(argv[1]);

    // assuming no more than 100 objects
    SingleObjectPosesInfo all_objects_info[100];

    for (int object_index = 0; object_index < total_num_objects_in_database;
            object_index++)
    {
        all_objects_info[object_index].Initialize(argv[3 * object_index + 2], atoi(argv[3 * object_index + 3]), argv[3 * object_index + 4]);
        all_objects_info[object_index].LoadAllObjectPoseKeypointsFromFiles();
    }

    // start the camera at index 0
    CvCapture* camera_feed = cvCreateCameraCapture(0);
    // grab 1st frame from the camera, to get frame width/height
    IplImage* camera_frame = cvQueryFrame(camera_feed);

    // create new image to store 1 channel gray scale version of current RGB camera frame
    IplImage* current_scene_image_gray = cvCreateImage(cvSize(camera_frame->width, camera_frame->height), 8, 1);

    char image_file_name[256];
    char system_command[512];

    // initialize font to overlay object name over output display image
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 2.0, 2.0, 0, 2);
    cvNamedWindow("scene", 1);
    cvMoveWindow("scene", 0, 0);
    while (true)
    {
        camera_frame = cvQueryFrame(camera_feed);
        cvCvtColor(camera_frame, current_scene_image_gray, CV_BGR2GRAY);
        memset(image_file_name, 0, sizeof(image_file_name));
        sprintf(image_file_name, "scene.pgm");
        cvSaveImage(image_file_name, current_scene_image_gray);

        // call siftWin32 to extract keypoints
        memset(system_command, 0, sizeof(system_command));
        sprintf(system_command, "sift <scene.pgm> scene.key");
        system(system_command);

        Keypoint current_scene_keypoints = ReadKeyFileOrDie((char*) "scene.key");

        // do matching one by one with all object poses
        for (int object_index = 0; object_index < total_num_objects_in_database;
                object_index++)
        {
            all_objects_info[object_index].GetBestObjectPoseMatch(current_scene_keypoints);

            if (all_objects_info[object_index].best_pose_match_count_ >= OBJECT_MIN_KEYPOINT_MATCHES)
            {
                char text[128];
                sprintf(text, "%s", all_objects_info[object_index].object_name_);
                cvPutText(current_scene_image_gray, text, cvPoint(all_objects_info[object_index].best_pose_match_coordinates_2D32f_.x - 75, all_objects_info[object_index].best_pose_match_coordinates_2D32f_.y), &font, cvScalar(255, 255, 255));
            }
        }
        cvShowImage("scene", current_scene_image_gray);

        // exit when any key pressed
        if(cvWaitKey(1) >= 0) break;
    }

    // release camera feed
    cvReleaseCapture(&camera_feed);
}



