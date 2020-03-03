//
// Created by hans on 2019/08/19.
//

#pragma once

//-------------include----------------//
#include <composite/composite.h>
//------------namespace---------------//
//--------------class-----------------//
/**
 * 失敗しないものが出るまで先頭から順番に実行し続ける
 * 失敗ではない場合，その判定が返される
 * 全て失敗すれば失敗判定
 */
class Selector : public Composite {
public:
    Selector() {
        name = "Selector";
    }

    virtual Status run(WorldModel &world_model, uint8_t my_id) override {
        for (auto &c : children) {
            c->status = c->run(world_model, my_id);
            if (c->status != Status::FAILURE) {
                return c->status;
            }
        }
        return Status::FAILURE;
    }
};