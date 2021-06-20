#ifndef ROBOT_ARM_CHECKPOINT_H
#define ROBOT_ARM_CHECKPOINT_H

#define NUM_CP 20

#define CP_TYPE_NONE 0
#define CP_TYPE_MOVE_TO 1
#define CP_TYPE_DELAY 2
#define CP_TYPE_GPIO 3
#define CP_TYPE_FUNC 4


class Checkpoint {
    public:
    int type = CP_TYPE_NONE;

    // position cp variables
    float x = 0;
    float y = 0;
    float z = 0;

    // delay cp variables
    int duration = 0;

    // GPIO cp variables
    bool state = false;
    int pin;

    // func cp variables
    void (*func)();
};


#endif
