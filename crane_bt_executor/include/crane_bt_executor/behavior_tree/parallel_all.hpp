//
// Created by hans on 2019/08/19.
//

#pragma once

//-------------include----------------//
#include <composite/composite.h>

//------------namespace---------------//
//--------------class-----------------//

/**
 * 全てのタスクを並列実行（既に成功したものは実行しない）
 * 全て成功したら成功判定
 * 一つでも失敗すれば失敗判定
 */
class ParallelAll : public Composite {
public:
    ParallelAll() {
        name = "ParallelAll";
    }

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        uint8_t num_success = 0;
        for (auto &c : children) {
            if (c->status == Status::SUCCESS) {
                num_success++;
                continue;
            }
            c->status = c->run(world_model, my_id);
            if (c->status == Status::FAILURE) {
                return Status::FAILURE;
            } else if (c->status == Status::SUCCESS) {
                num_success++;
            }
        }
        if (num_success == children.size()) {
            return Status::SUCCESS;
        }
        return Status::RUNNING;
    }
};