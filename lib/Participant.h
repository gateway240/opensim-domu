#ifndef PARTICIPANT_H
#define PARTICIPANT_H
#include <string>
#include <vector>

typedef struct {
  int ID;
  int Age;
  char Gender; // 'M' or 'F'
  char Leg;    // Assuming 'L' or 'R'
  double Height;
  std::vector<std::string>
      Invalid_trials; // Store invalid trials as a vector of strings
  int IAD;
  int Left_knee_width;
  int Right_knee_width;
  int Left_ankle_width;
  int Right_ankle_width;
  int Left_thigh_length;
  int Right_thigh_length;
  int Left_shank_length;
  int Right_shank_length;
  double Mass;
  double ICD;
  double Left_knee_width_mocap;
  double Right_knee_width_mocap;
} Participant;

#endif // PARTICIPANT_H