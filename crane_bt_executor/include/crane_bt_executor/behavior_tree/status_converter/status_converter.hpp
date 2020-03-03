//
// Created by hans on 2019/09/06.
//

#pragma once

//-------------include----------------//
#include <composite/component.h>
#include <memory>

//------------namespace---------------//
//--------------class-----------------//
class StatusConverter : public Component {
protected:
    std::shared_ptr<Component> base;
public:
    explicit StatusConverter(std::string name, std::shared_ptr<Component> base) : Component(), base(base) {
        this->name = name;
    }

    virtual void serialize(ptree &my_tree) override;
};