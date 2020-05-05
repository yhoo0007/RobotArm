#include "RobotArmCalc.h"
#include "esp32-hal.h"
#include "math.h"  // math functions
#include "RobotArmConfig.h"  // get arm dimensions
#include "RobotArmDebug.h"  // debug prints


#define RAD_TO_DEG 57.2958  // 180 / pi  // already defined in Arduino.h
#define constrain(x, a, b) (x < a ? a : x > b ? b : x)


char coordToAngle(float x, float y, float z, float *baseAngle, float *armAAngle, float *armBAngle, float *pickerAngle) {
    float _baseAngle, _armAAngle, _armBAngle, _pickerAngle;
    // calculate base angle
    if (fabs(x) == 0) {
        _baseAngle = 90;
    } else if (fabs(y) == 0) {
        _baseAngle = x < 0 ? 180 : 0;
    } else {
        _baseAngle = atan(y / x);
        _baseAngle *= RAD_TO_DEG;
        if (x < 0) _baseAngle += 180;
    }
    
    // calculate arm A angle
    float r1 = sqrt(x * x + y * y);
    float r2 = z - Z_CENTER_TO_ORIGIN;
    float r3 = sqrt(r1 * r1 + r2 * r2);
    _armAAngle = (acos((ARM_B2 - ARM_A2 - r3 * r3) / (-2 * ARM_A * r3)) + atan(r2 / r1)) * RAD_TO_DEG;

    // calculate arm B angle
    _armBAngle = acos((r3 * r3 - ARM_A2 - ARM_B2) / (-2 * ARM_A * ARM_B)) * RAD_TO_DEG;

    // calculate picker angle
    // _pickerAngle = 180 - (_armBAngle - (90 - _armAAngle));
    _pickerAngle = 270 - _armBAngle - _armAAngle;

    // validate
    if (_baseAngle < BASE_MIN_ANGLE || _baseAngle > BASE_MAX_ANGLE ||
        _armAAngle < ARM_A_MIN_ANGLE || _armAAngle > ARM_A_MAX_ANGLE ||
        _armBAngle < ARM_B_MIN_ANGLE || _armBAngle > ARM_B_MAX_ANGLE ||
        isnan(_baseAngle) || isnan(_armAAngle) || isnan(_armBAngle))
        return -1;

    // return angles
    *baseAngle = _baseAngle;
    *armAAngle = _armAAngle;
    *armBAngle = _armBAngle;
    *pickerAngle = _pickerAngle;
    return 0;
}


int calcTimeMillis(int steps, int maxTime, int minTime, int stepRange) {
    R_DPRINTLN("CalcTime Max: " + String(maxTime) + " Min: " + String(minTime) + " Steps: " + String(steps));
    return (maxTime - minTime) * sin(((double)abs(steps) * M_PI) / (stepRange * 2)) + 100;
}
