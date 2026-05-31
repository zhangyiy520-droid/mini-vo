/** @file test_initializer.cpp
 *  Stub — fill in with real test images and known geometry.
 */
#include "mini_vo/initializer.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>

int main()
{
    std::cout << "test_initializer: stub — TODO" << std::endl;

    // TODO: load a pair of calibrated images
    // TODO: verify R ≈ I, t ≈ 0 for identical views
    // TODO: verify triangulated points have Z > 0
    // TODO: verify median-depth filtering rejects extreme outliers

    return 0;
}
