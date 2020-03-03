//
// Created by hans on 2019/08/16.
//

#pragma once

//-------------include----------------//
#include <composite/composite.h>

//------------namespace---------------//
//--------------class-----------------//

/**
 * 逐次実行．（先頭のものが成功するまで次にいかない）
 * 全て成功すれば成功判定
 * 途中で失敗が出れば失敗判定
 */
class Sequence : public Composite {
public:
    Sequence() {
        name = "Sequence";
    }

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        for (auto &c : children) {
            if (c->status == Status::SUCCESS)
                continue;

            c->status = c->run(world_model, my_id);

            if (c->status != Status::SUCCESS) {
                if (c->status == Status::FAILURE) {
                    return Status::FAILURE;
                }
                return c->status;
            }
        }
        return Status::SUCCESS;
    }
};
