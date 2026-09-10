// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_utils/package.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(TestPackage, GetPackageSharePath)
{
  // 存在するパッケージ
  auto share_path = crane::get_package_share_path("crane_utils");
  ASSERT_TRUE(share_path.has_value());
  EXPECT_TRUE(std::filesystem::exists(*share_path));

  // 存在しないパッケージ
  auto invalid_path = crane::get_package_share_path("non_existent_package_xyz_999");
  EXPECT_FALSE(invalid_path.has_value());
}

TEST(TestPackage, ResolvePackagePath)
{
  // 空パス
  EXPECT_EQ(crane::resolve_package_path("crane_utils", ""), "");

  // 絶対パス
  std::filesystem::path abs_path = "/usr/local/test.yaml";
  EXPECT_EQ(crane::resolve_package_path("crane_utils", abs_path), abs_path);

  // 存在するパッケージの相対パス (デフォルト sub_dir: "config")
  auto share_path = crane::get_package_share_path("crane_utils");
  ASSERT_TRUE(share_path.has_value());

  auto resolved = crane::resolve_package_path("crane_utils", "test.yaml");
  EXPECT_EQ(resolved, *share_path / "config" / "test.yaml");

  // カスタム sub_dir
  auto resolved_custom = crane::resolve_package_path("crane_utils", "test.yaml", "custom_dir");
  EXPECT_EQ(resolved_custom, *share_path / "custom_dir" / "test.yaml");

  // 存在しないパッケージ名の場合はそのままのパスを返却
  EXPECT_EQ(
    crane::resolve_package_path("non_existent_package_xyz_999", "relative.yaml"), "relative.yaml");
}

TEST(TestPackage, ResolvePackagePathWithLogger)
{
  auto logger = rclcpp::get_logger("test_logger");

  // 空パス
  EXPECT_EQ(crane::resolve_package_path(logger, "crane_utils", ""), "");

  // 絶対パス
  std::filesystem::path abs_path = "/var/log/test.yaml";
  EXPECT_EQ(crane::resolve_package_path(logger, "crane_utils", abs_path), abs_path);

  // 存在するパッケージ
  auto share_path = crane::get_package_share_path("crane_utils");
  ASSERT_TRUE(share_path.has_value());
  auto resolved = crane::resolve_package_path(logger, "crane_utils", "config.yaml");
  EXPECT_EQ(resolved, *share_path / "config" / "config.yaml");

  // 存在しないパッケージ名（WARN ログを出力してフォールバック）
  EXPECT_EQ(
    crane::resolve_package_path(logger, "non_existent_package_xyz_999", "relative.yaml"),
    "relative.yaml");
}
