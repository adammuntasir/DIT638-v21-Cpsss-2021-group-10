#include <iostream>
#include "PrimeChecker.hpp"
/*including prime checker as header file*/

int main(int argc, char** argv) {
if (argc == 2) { /*checking is correct number of arguments given*/
int number = std::stoi(argv[1]);
    PrimeChecker pc;
    std::cout << "Group 10; " << number << " is a prime number? " << pc.isPrime(number) << std::endl;
}
    return 0; /*this is to return 0 when its ok*/
}
