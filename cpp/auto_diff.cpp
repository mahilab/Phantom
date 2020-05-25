#include <complex>
#include <iostream>

typedef std::complex<double> dual;

constexpr double small = 1.0e-12;

inline double auto_diff(dual(*f)(dual), double x) {
    return f(dual(x, small)).imag() / small;
}

dual f(dual x) {
    return log(x)*sin(x);
}

inline double auto_diff(dual(*g)(dual,dual), dual x, dual y) {
    return g(x,y).imag() / small;
}

dual g(dual x, dual y) {
    return log(x)*sin(y);
}

int main(int argc, char const *argv[])
{
    // auto dfdx = auto_diff(f,2);
    // std::cout << dfdx << std::endl;

    auto dgdx = auto_diff(g, dual(2,small), 5);
    auto dgdy = auto_diff(g, 2, dual(5,small));
    std::cout << dgdx << std::endl;
    std::cout << dgdy << std::endl;

    return 0;
}
