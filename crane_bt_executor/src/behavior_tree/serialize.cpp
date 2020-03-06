// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#include <boost/property_tree/ptree.hpp>
#include <crane_bt_executor/behavior_tree/status_converter/status_converter.hpp>
#include <crane_bt_executor/composite/composite.hpp>
#include <crane_bt_executor/composite/leaf.hpp>

#include <utility>

void StatusConverter::serialize(ptree &my_tree) {
    my_tree.put("name", name);
    my_tree.put("status", static_cast<uint8_t>(status));

    ptree children;
    ptree base_info;
    base->serialize(base_info);
    children.push_back(std::make_pair("", base_info));
    my_tree.add_child("children", children);
}

void Composite::serialize(ptree &my_tree) {
    my_tree.put("name", name);
    my_tree.put("status", static_cast<uint8_t>(status));

    ptree children;
    for (auto child : this->children) {
        ptree info;
        child->serialize(info);
        children.push_back(std::make_pair("", info));
    }

    my_tree.add_child("children", children);
}

void Leaf::serialize(ptree &my_tree) {
    my_tree.put("name", name);
    my_tree.put("status", static_cast<uint8_t>(status));
}
