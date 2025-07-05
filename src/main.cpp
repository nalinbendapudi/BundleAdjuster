#include <iostream>
#include "check_libs.h"

int main() {
    std::cout << "Hello from Bundle Adjustment main!" << std::endl;

    // Create an instance of CheckLibs and run tests for each library    
    CheckLibs checker;

    std::cout << "Testing libraries..." << std::endl;
    checker.testEigen();
    checker.testOpenCV();
    checker.testCeres();
    checker.testPCL();

    return 0;
}
