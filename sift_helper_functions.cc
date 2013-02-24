/*
 * sift_helper_functions.cc
 *
 *  Created on: Feb 23, 2013
 *      Author: Mukul
 */

#include "sift_helper_functions.h"

void FindMatches(const Keypoint keys_1, const Keypoint keys_2, CvPoint2D32f& coords, int& count)
{
    Keypoint k, match;
    count = 0;

    coords.x = 0;
    coords.y = 0;

    //  Match the keys in list keys_1 to their best matches in keys_2.
    for (k = keys_1; k != NULL; k = k->next)
    {
        match = CheckForMatch(k, keys_2);
        if (match != NULL)
        {
            count++;
            coords.x += match->col;
            coords.y += match->row;
        }
    }
    coords.x /= count;
    coords.y /= count;
}

// This searches through the keypoints in klist for the two closest
// matches to key.  If the closest is less than 0.6 times distance to
// second closest, then return the closest match.  Otherwise, return
// NULL.
Keypoint CheckForMatch(const Keypoint key, const Keypoint klist)
{
    int dsq, distsq1 = 100000000, distsq2 = 100000000;
    Keypoint k, minkey = NULL;

    //  Find the two closest matches, and put their squared distances in
    //  distsq1 and distsq2.
    for (k = klist; k != NULL; k = k->next)
    {
        dsq = DistSquared(key, k);

        if (dsq < distsq1)
        {
            distsq2 = distsq1;
            distsq1 = dsq;
            minkey = k;
        }
        else if (dsq < distsq2)
        {
            distsq2 = dsq;
        }
    }

    //  Check whether closest distance is less than 0.6 of second.
    if (10 * 10 * distsq1 < 6 * 6 * distsq2)
        return minkey;
    else
        return NULL;
}

// Return squared distance between two keypoint descriptors.
int DistSquared(Keypoint k1, Keypoint k2)
{
    int i, dif, distsq = 0;
    unsigned char *pk1, *pk2;

    pk1 = k1->descrip;
    pk2 = k2->descrip;

    for (i = 0; i < 128; i++)
    {
        dif = (int) *pk1++ - (int) *pk2++;
        distsq += dif * dif;
    }
    return distsq;
}

// This reads a keypoint file from a given filename and returns the list
// of keypoints.
Keypoint ReadKeyFileOrDie(char *filename)
{
    FILE *file;

    file = fopen(filename, "r");
    if (!file)
    {
        printf("Could not open file: %s", filename);
        exit(0);
    }

    return ReadKeysOrDie(file);
}

// Read keypoints from the given file pointer and return the list of
// keypoints.  The file format starts with 2 integers giving the total
// number of keypoints and the size of descriptor vector for each
// keypoint (currently assumed to be 128). Then each keypoint is
// specified by 4 floating point numbers giving subpixel row and
// column location, scale, and orientation (in radians from -PI to
// PI).  Then the descriptor vector for each keypoint is given as a
// list of integers in range [0,255].
Keypoint ReadKeysOrDie(FILE *fp)
{
    int i, j, num, len, val;
    Keypoint k, keys = NULL;

    if (fscanf(fp, "%d %d", &num, &len) != 2)
    {
        printf("Error! Invalid keypoint file beginning.\n");
        exit(0);
    }

    if (len != 128)
    {
        printf("Keypoint descriptor length invalid (should be 128).\n");
        exit(0);
    }

    for (i = 0; i < num; i++)
    {
        //Allocate memory for the keypoint.
        k = (Keypoint) malloc(sizeof(struct KeypointSt));
        k->next = keys;
        keys = k;
        k->descrip = (unsigned char*) malloc(len);

        if (fscanf(fp, "%f %f %f %f", &(k->row), &(k->col), &(k->scale), &(k->ori)) != 4)
        {
            printf("Invalid keypoint file format.\n");
            exit(0);
        }

        for (j = 0; j < len; j++)
        {
            if (fscanf(fp, "%d", &val) != 1 || val < 0 || val > 255)
            {
                printf("Invalid keypoint file value.\n");
                exit(0);
            }

            k->descrip[j] = (unsigned char) val;
        }
    }
    return keys;
}

// only dump object key images
void DumpObjectPoseKey(int num_object_images, char* object_path_prefix)
{
    char system_command[512];
    IplImage* pose_loaded_from_jpg;
    for (int pose = 0; pose < num_object_images; pose++)
    {
        memset(system_command, 0, sizeof(system_command));
        sprintf(system_command, "%s%d.jpg", object_path_prefix, pose);
        printf("loading jpg file %s\n", system_command);
        pose_loaded_from_jpg = cvLoadImage(system_command, CV_LOAD_IMAGE_GRAYSCALE);
        if (pose_loaded_from_jpg)
        {

            memset(system_command, 0, sizeof(system_command));
            sprintf(system_command, "%s%d.pgm", object_path_prefix, pose);
            printf("SUCCESS!....saved file %s\n", system_command);
            cvSaveImage(system_command, pose_loaded_from_jpg);
            cvReleaseImage(&pose_loaded_from_jpg);
        }
        memset(system_command, 0, sizeof(system_command));
        sprintf(system_command, "siftWin32.exe <%s%d.pgm> %s%d.key", object_path_prefix, pose, object_path_prefix, pose);
        printf("Calling syscmd:%s\n", system_command);
        system(system_command);
    }
}


