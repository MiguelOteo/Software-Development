#include <iostream>

int divide(const int num, const int den) {
    return num/den;
}

int main(void) {
    int a = 7;
    int b = 3;
    int c = divide(a, b);
    std::cout << a << "/" << b << "=" << c << std::endl;
    return 0;
}

