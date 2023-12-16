#pragma once

class Controller {
public:
    virtual ~Controller() {}

    // returns true when task has finished
    virtual bool step(float dt) = 0;
};
