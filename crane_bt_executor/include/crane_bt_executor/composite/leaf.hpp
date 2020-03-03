//
// Created by hans on 17/10/23.
//
#pragma once

//-------------include----------------//
//from custom header file
#include <composite/component.h>

//-------------class--------------------//
class Leaf : public Component {
private:
public:
    Leaf() {}

    virtual void serialize(ptree &my_tree) override;
};
