//
// Created by hans on 2019/09/04.
//

#pragma once

//-------------include----------------//
#include <behavior_tree/status_converter/status_converter.h>
//------------namespace---------------//
//--------------class-----------------//

/**
 * 判定は常にRUNNING
 */
class AlwaysRunning : public StatusConverter {
public:
    explicit AlwaysRunning(std::shared_ptr<Component> base) : StatusConverter("AlwaysRunning", base) {}

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        base->status = base->run(world_model, my_id);
        status = Status::RUNNING;
        return Status::RUNNING;
    }

    static auto build(std::shared_ptr<Component> base) {
        return std::make_shared<AlwaysRunning>(base);
    }
};

/**
 * 判定は常にFAILURE
 */
class AlwaysFailure : public StatusConverter {
public:
    explicit AlwaysFailure(std::shared_ptr<Component> base) : StatusConverter("AlwaysFailure", base) {}

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        base->status = base->run(world_model, my_id);
        status = Status::FAILURE;
        return Status::FAILURE;
    }

    static auto build(std::shared_ptr<Component> base) {
        return std::make_shared<AlwaysFailure>(base);
    }
};

/**
 * 判定は常にSUCCESS
 */
class AlwaysSuccess : public StatusConverter {
public:
    explicit AlwaysSuccess(std::shared_ptr<Component> base) : StatusConverter("AlwaysSuccess", base) {}

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        base->status = base->run(world_model, my_id);
        status = Status::SUCCESS;
        return status;
    }

    static auto build(std::shared_ptr<Component> base) {
        return std::make_shared<AlwaysSuccess>(base);
    }
};