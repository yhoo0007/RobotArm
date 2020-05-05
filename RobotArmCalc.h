#ifndef ROBOT_ARM_CALC_H
#define ROBOT_ARM_CALC_H

#define max(a, b) (a > b ? a : b)

#define getPickerAngle(angleB, angleC) (angleB - angleC)
#define getPickerSteps(stepsB, stepsC) (stepsB - stepsC)
#define angleToSteps(angle, ratio) ((angle) / DEGREE_PER_STEP * (ratio))
#define stepsToAngle(steps, ratio) ((steps) / (ratio) * DEGREE_PER_STEP)

char coordToAngle(float x, float y, float z, float *baseAngle, float *armAAngle, float *armBAngle, float *pickerAngle);
int calcTimeMillis(int steps, int maxTime, int minTime, int stepRange);

#endif
