#include <modern_orca/modern_orca.hpp>
#include <iostream>
int main() {
    modern_orca::Vector2D v(1, 2);
    std::cout << "Modern ORCA Library compiled successfully\!" << std::endl;
    std::cout << "Vector: (" << v.x() << ", " << v.y() << ")" << std::endl;
    return 0;
}
