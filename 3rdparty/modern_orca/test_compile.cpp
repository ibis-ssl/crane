// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <iostream>
#include <modern_orca/modern_orca.hpp>
int main()
{
  modern_orca::Vector2D v(1, 2);
  std::cout << "Modern ORCA Library compiled successfully\!" << std::endl;
  std::cout << "Vector: (" << v.x() << ", " << v.y() << ")" << std::endl;
  return 0;
}
