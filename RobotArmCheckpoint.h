#ifndef ROBOT_ARM_CHECKPOINT_H
#define ROBOT_ARM_CHECKPOINT_H

#define NUM_CP 20

#define CP_TYPE_NONE 0
#define CP_TYPE_MOVE_TO 1
#define CP_TYPE_DELAY 2
#define CP_TYPE_VALVE 3


class Checkpoint {
    public:
    int type = CP_TYPE_NONE;

    float x = 0;
    float y = 0;
    float z = 0;

    int duration = 0;

    bool state = false;
    int pin;
};


#endif
