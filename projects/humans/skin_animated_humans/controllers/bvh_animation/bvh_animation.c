/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/bvh_util.h>
#include <webots/robot.h>
#include <webots/skin.h>
#include <webots/supervisor.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TIME_STEP 32

static void print_usage(const char *command) {
  printf("Usage: %s -d <skin_device_name> [-f <motion_file_path> | -s <start_frame_index> | -e <end_frame_index> | -l]\n",
         command);
  printf("Options:\n");
  printf("  -d: Skin device name.\n");
  printf("  -f: path to motion file.\n");
  printf("  -s: scale factor for motion translation. Default is 20.\n");
  printf("  -e: index of ending motion frame.\n");
  printf("  -l: loop motion without resetting to initial position.\n");
}

int main(int argc, char **argv) {
  wb_robot_init();

  char *skin_device_name = NULL;
  char *motion_file_path = NULL;
  int end_frame_index = 0;
  int scale = 20;
  bool loop = false;
  int c;
  while ((c = getopt(argc, argv, "d:f:s:e:l")) != -1) {
    switch (c) {
      case 'd':
        skin_device_name = optarg;
        break;
      case 'f':
        motion_file_path = optarg;
        break;
      case 's':
        scale = atoi(optarg);
        break;
      case 'e':
        end_frame_index = atoi(optarg);
        break;
      case 'l':
        loop = true;
        break;
      case '?':
        printf("?\n");
        if (optopt == 'd' || optopt == 'f' || optopt == 's' || optopt == 'e')
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        else
          fprintf(stderr, "Unknown option `-%c'.\n", optopt);
      default:
        print_usage(argv[0]);
        return 1;
    }
  }

  if (skin_device_name == NULL || motion_file_path == NULL) {
    fprintf(stderr, "Missing required arguments -d and -f.\n");
    print_usage(argv[0]);
    return 1;
  }

  WbDeviceTag skin = wb_robot_get_device(skin_device_name);

  // Open a BVH animation file.
  WbuBvhMotion bvh_motion = wbu_bvh_read_file(motion_file_path);
  if (bvh_motion == NULL) {
    wb_robot_cleanup();
    return -1;
  }

  int i, j;

  // Get the number of bones in the Skin device
  const int skin_bone_count = wb_skin_get_bone_count(skin);
  if (skin_bone_count == 0) {
    printf("The Skin model has no bones to animate.\n");
    return 0;
  }

  // Get the number of joints and frames in the BVH file.
  const int bvh_joint_count = wbu_bvh_get_joint_count(bvh_motion);
  const int bvh_frame_count = wbu_bvh_get_frame_count(bvh_motion);
  printf("The BVH file \"%s\" has %d joints, and %d frames.\n", motion_file_path, bvh_joint_count, bvh_frame_count);

  // Get the bone names in the Skin device
  char **joint_name_list;
  joint_name_list = (char **)malloc((skin_bone_count) * sizeof(char *));
  int root_bone_index = -1;
  for (i = 0; i < skin_bone_count; ++i) {
    const char *name = wb_skin_get_bone_name(skin, i);
    joint_name_list[i] = (char *)malloc(strlen(name) + 1);
    strcpy(joint_name_list[i], name);
    if (strcmp(name, "Hips") == 0)
      root_bone_index = i;
  }

  // Optional: Print the list of names in the Skin model.
  printf("Human model joins:\n");
  for (i = 0; i < skin_bone_count; ++i)
    printf("  Joint %d: %s\n", i, joint_name_list[i]);

  if (root_bone_index < 0)
    fprintf(stderr, "Root joint not found\n");

  // Find correspondencies between the Skin's bones and BVH's joint.
  // For example 'hip' could be bone 0 in Skin device, and joint 5 in BVH motion file
  int *index_skin_to_bvh = (int *)malloc(skin_bone_count * sizeof(int));
  for (i = 0; i < skin_bone_count; ++i) {
    index_skin_to_bvh[i] = -1;

    if (i == 24 || i == 25 || i == 26 || i == 15 || i == 16 || i == 17)
      continue;

    const char *skin_name = joint_name_list[i];
    for (j = 0; j < bvh_joint_count; ++j) {
      const char *bvh_name = wbu_bvh_get_joint_name(bvh_motion, j);
      if (strcmp(skin_name, bvh_name) == 0)
        index_skin_to_bvh[i] = j;
    }
  }

  // Pass absolute and relative joint T pose orientation to BVH utility library
  for (i = 0; i < skin_bone_count; ++i) {
    if (index_skin_to_bvh[i] < 0)
      continue;
    const double *global_t_pose = wb_skin_get_bone_orientation(skin, i, true);
    wbu_bvh_set_model_t_pose(bvh_motion, global_t_pose, index_skin_to_bvh[i], true);
    const double *local_t_pose = wb_skin_get_bone_orientation(skin, i, false);
    wbu_bvh_set_model_t_pose(bvh_motion, local_t_pose, index_skin_to_bvh[i], false);
  }

  // Set factor converting from BVH skeleton scale to Webots skeleton scale.
  // Only translation values are scaled by this factor.
  wbu_bvh_set_scale(bvh_motion, scale);

  double initial_root_position[3] = {0.0, 0.0, 0.0};
  double root_position_offset[3] = {0.0, 0.0, 0.0};
  const double *skin_root_position = wb_skin_get_bone_position(skin, root_bone_index, false);
  if (root_bone_index >= 0) {
    const double *current_root_position = wbu_bvh_get_root_translation(bvh_motion);
    // Use initial Skin position as zero reference position
    for (i = 0; i < 3; ++i) {
      root_position_offset[i] = skin_root_position[i] - current_root_position[i];
      initial_root_position[i] = current_root_position[i];
    }
  }

  // Check end frame index
  if (end_frame_index > 0 && end_frame_index >= bvh_frame_count) {
    fprintf(stderr, "Invalid end frame index %d. This motion has %d frames.\n", end_frame_index, bvh_frame_count);
    end_frame_index = bvh_frame_count;
  } else
    end_frame_index = bvh_frame_count;

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < skin_bone_count; ++i) {
      if (index_skin_to_bvh[i] < 0)
        continue;

      // Get joint rotation for each joint.
      // Note that we need to pass the joint index according to BVH file.
      const double *orientation = wbu_bvh_get_joint_rotation(bvh_motion, index_skin_to_bvh[i]);
      wb_skin_set_bone_orientation(skin, i, orientation, false);
    }

    // Offset the position by a desired value if needed.
    const double *root_position;
    if (root_bone_index >= 0) {
      root_position = wbu_bvh_get_root_translation(bvh_motion);
      double position[3];
      for (i = 0; i < 3; ++i)
        position[i] = root_position[i] + root_position_offset[i];
      wb_skin_set_bone_position(skin, root_bone_index, position, false);
    }

    // Fetch the next animation frame.
    // The simulation update rate is lower than the BVH frame rate, so 4 BVH motion frames are fetched.
    const int current_frame_index = wbu_bvh_get_frame_index(bvh_motion);
    const int remaining_frames = end_frame_index - current_frame_index;
    if (remaining_frames <= 4) {
      if (loop && root_bone_index >= 0) {
        // Save new global position offset
        // based on last frame and not on loaded frame (1 over 4)
        wbu_bvh_goto_frame(bvh_motion, end_frame_index - 1);
        root_position = wbu_bvh_get_root_translation(bvh_motion);
        for (i = 0; i < 3; ++i)
          root_position_offset[i] += root_position[i] - initial_root_position[i];
      }
      wbu_bvh_goto_frame(bvh_motion, 1);  // skip initial pose
    } else {
      int f = 4;
      while (f > 0) {
        wbu_bvh_step(bvh_motion);
        --f;
      }
    }
  }

  // Cleanup
  for (i = 0; i < skin_bone_count; ++i)
    free(joint_name_list[i]);
  free(joint_name_list);
  free(index_skin_to_bvh);
  wbu_bvh_cleanup(bvh_motion);
  wb_robot_cleanup();

  return 0;
}
