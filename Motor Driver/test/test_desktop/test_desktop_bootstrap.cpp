#include <unity.h>

// Include here the bootstraps for testing packages
#include "example_package/example_package_bootstrap.hpp"

void RUN_UNITY_TESTS() {
    UNITY_BEGIN();
    RUN_EXAMPLE_PACKAGE_TESTS();
    UNITY_END();
}

int main(int argc, char **argv) {
    RUN_UNITY_TESTS();
    return 0;
}