// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <QApplication>

#include "crane_commander.hpp"

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  crane::CraneCommander commander;
  commander.show();
  return QApplication::exec();
}
