// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_utils/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

class ParameterTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ParameterTest, DeclareAndGetValue)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  int int_val = crane::get_or_declare_parameter(*node, "test_int", 42);
  EXPECT_EQ(int_val, 42);

  double double_val = crane::get_or_declare_parameter(*node, "test_double", 3.14);
  EXPECT_DOUBLE_EQ(double_val, 3.14);

  bool bool_val = crane::get_or_declare_parameter(*node, "test_bool", true);
  EXPECT_TRUE(bool_val);

  std::string str_val = crane::get_or_declare_parameter(*node, "test_str", std::string("hello"));
  EXPECT_EQ(str_val, "hello");

  std::string literal_str_val = crane::get_or_declare_parameter(*node, "test_literal_str", "world");
  EXPECT_EQ(literal_str_val, "world");
}

TEST_F(ParameterTest, DeclareAndGetByReference)
{
  auto node = std::make_shared<rclcpp::Node>("test_node_ref");

  int int_val = 100;
  crane::get_or_declare_parameter(*node, "ref_int", int_val);
  EXPECT_EQ(int_val, 100);

  double double_val = 2.718;
  crane::get_or_declare_parameter(*node, "ref_double", double_val);
  EXPECT_DOUBLE_EQ(double_val, 2.718);

  bool bool_val = false;
  crane::get_or_declare_parameter(*node, "ref_bool", bool_val);
  EXPECT_FALSE(bool_val);

  std::string str_val = "ref_test";
  crane::get_or_declare_parameter(*node, "ref_str", str_val);
  EXPECT_EQ(str_val, "ref_test");

  // 参照渡し版の戻り値を代入しても動作することの確認
  int assigned_val = 200;
  assigned_val = crane::get_or_declare_parameter(*node, "ref_assigned", assigned_val);
  EXPECT_EQ(assigned_val, 200);
}

TEST_F(ParameterTest, AlreadyDeclaredDoesNotThrow)
{
  auto node = std::make_shared<rclcpp::Node>("test_node_already_declared");

  node->declare_parameter("existing_param", 10);

  // 宣言済みパラメータに対して get_or_declare_parameter を呼んでも
  // 例外が飛ばず既存値を取得できること
  EXPECT_NO_THROW({
    int val = crane::get_or_declare_parameter(*node, "existing_param", 20);
    EXPECT_EQ(val, 10);
  });

  // 参照渡し版でも既存値が設定されること
  EXPECT_NO_THROW({
    int val = 99;
    crane::get_or_declare_parameter(*node, "existing_param", val);
    EXPECT_EQ(val, 10);
  });
}

TEST_F(ParameterTest, NodePointerOverload)
{
  auto node = std::make_shared<rclcpp::Node>("test_node_ptr");

  int val = crane::get_or_declare_parameter(node.get(), "ptr_int", 55);
  EXPECT_EQ(val, 55);

  double d_val = 1.23;
  crane::get_or_declare_parameter(node.get(), "ptr_double", d_val);
  EXPECT_DOUBLE_EQ(d_val, 1.23);

  std::string s_val = crane::get_or_declare_parameter(node.get(), "ptr_str", "foo");
  EXPECT_EQ(s_val, "foo");
}
