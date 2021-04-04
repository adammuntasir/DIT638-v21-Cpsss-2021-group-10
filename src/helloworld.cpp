#include <iostream>
#include "PrimeChecker.hpp"
/*including prime checker as header file*/

int main(int argc, char** argv) {
if (argc == 2) {
int number = std::stoi(argv[1]);
    PrimeChecker pc;
    std::cout << "Group 10; " << number << " is a prime number? " << pc.isPrime(number) << std::endl;
}
    return 0; /*a good comment*/
}
