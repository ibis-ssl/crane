//
// Created by hans on 2019/08/19.
//

#pragma once

//-------------include----------------//
#include <composite/composite.h>

//------------namespace---------------//
//--------------class-----------------//

/**
 * 全てのタスクを並列実行(失敗したものは実行しない)
 * 一つでも成功判定が出たら成功
 * 全て失敗したら失敗判定
 */
class ParallelOne : public Composite {
public:
    ParallelOne() {
        name = "ParallelOne";
    }

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        uint8_t num_failure = 0;
        for (auto &c : children) {
            if (c->status == Status::FAILURE) {
                num_failure++;
                continue;
            }
            c->status = c->run(world_model, my_id);

            if (c->status == Status::SUCCESS) {
                return Status::SUCCESS;
            } else if (c->status == Status::FAILURE) {
                num_failure++;
            }
        }

        if (num_failure == children.size()) {
            return Status::FAILURE;
        }
        return Status::RUNNING;
    }
};