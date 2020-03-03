//
// Created by hans on 17/10/23.
//
#pragma once

//-------------include----------------//
//from  standard library
#include <vector>
#include <string>

//-------------namespace------------------//
//-------------class--------------------//

class WorldModel;
namespace boost::property_tree {
    template<class Key, class Data, class KeyCompare>
    class basic_ptree;

    typedef basic_ptree<std::string, std::string, std::less<std::string>> ptree;
}
using boost::property_tree::ptree;

enum class Status {
    NONE,
    FAILURE,
    SUCCESS,
    RUNNING,
};

class Component {
public:
    std::string name;
    Status status = Status::RUNNING;
public:
    Component() {}

    virtual Status run(WorldModel &world_model, uint8_t my_id) = 0;

    virtual void serialize(ptree &my_tree) = 0;
};
