#ifndef PICKER_CONTROLLER_H
#define PICKER_CONTROLLER_H


class PickerController {
    public:
    void init();
    void move(float angle, int timeMillis);
    bool done();
    void toggle(bool state);

    bool running = false;

    private:
    float angle;
    int steps;
};


#endif