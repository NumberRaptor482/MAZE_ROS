//
// Created by User on 7/5/2025.
//

#include "MathUtils.h"
#include <cmath>


double MathUtils::lowPassFilter(double previous, double input, double alpha) {
    return alpha * input + (1 - alpha) * previous;
}

double MathUtils::epsilonEquals(double a, double b, double epsilon) {
    return std::abs(a - b) <= epsilon;
}

double MathUtils::epsilonEquals(double a, double b) {
    return MathUtils::epsilonEquals(a, b, 10e-16);
}

double MathUtils::inputModulus(double input, double minInput, double maxInput) {
    double modulus = maxInput - minInput;

    // Normalize input relative to minInput
    double normalized = input - minInput;

    // Wrap using floor (handles negatives correctly)
    normalized -= std::floor(normalized / modulus) * modulus;

    // Return wrapped input back in [minInput, maxInput)
    return normalized + minInput;
}