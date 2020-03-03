//
// Created by hans on 17/10/23.
//
#pragma once

//-------------include----------------//
//from standard library
#include <memory> // std::shared_ptr
#include <vector>
//from custom header file
#include <composite/component.h>

//-------------class--------------------//
class Composite : public Component {
public:
    std::vector<std::shared_ptr<Component>> children;
public:
    Composite() {}

    void addChild(std::shared_ptr<Component> child) {
        children.emplace_back(child);
    }

    void clearChild() {
        children.clear();
    }

    virtual void serialize(ptree &my_tree) override;
};