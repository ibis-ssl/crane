//
// Created by hans on 2019/09/06.
//

#pragma once

//-------------include----------------//
#include <behavior_tree/status_converter/status_converter.h>

//------------namespace---------------//
//--------------class-----------------//
class Inverter : StatusConverter {
public:
    explicit Inverter(std::shared_ptr<Component> base) : StatusConverter("Inverter", base) {}

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        base->status = base->run(world_model, my_id);
        if (base->status == Status::SUCCESS) {
            this->status = Status::FAILURE;
        } else if (base->status == Status::FAILURE) {
            this->status = Status::SUCCESS;
        } else {
            this->status = Status::RUNNING;
        }
        return status;
    }
};