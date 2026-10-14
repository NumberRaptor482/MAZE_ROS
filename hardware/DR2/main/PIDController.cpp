//
// Created by User on 7/4/2025.
//

#include "PIDController.h"

#include <stddef.h>
#include <cmath>
#include "MathUtils.h"

PIDController::PIDController(double pP, double pI, double pD, double pDT) {
    kP = pP;
    kI = pI;
    kD = pD;
    dt = pDT;
}

double PIDController::getkP() {
    return  kP;
}

void PIDController::setkP(double pP) {
    kP = pP;
}

double PIDController::getkI() {
    return  kI;
}

void PIDController::setkI(double pI) {
    kI = pI;
}

double PIDController::getkD() {
    return kD;
}

void PIDController::setkD(double pD) {
    kD = pD;
}

double PIDController::getDt() {
    return dt;
}

void PIDController::setDt(double pDt) {
    dt = pDt;
}

/* INTEGRATION ACCUMULATION LIMITS(WIND-UP) */
/* Setting I-Range to zero disables it on controller */
void PIDController::setIRange(double pIRange) {
    setIRange(-pIRange, pIRange);
}

/* Setting I-Range to zero disables it on controller */
void PIDController::setIRange(double pINegativeR, double pIPositiveR) {
    setINegativeRange(pINegativeR);
    setIPositiveRange(pIPositiveR);
}

double PIDController::getIPositiveRange() {
    return iPositiveRange;
}

/* Setting I-Range to zero disables it on controller */
void PIDController::setIPositiveRange(double pIPositiveR) {
    iPositiveRange = pIPositiveR;
}

double PIDController::getINegativeRange() {
    return iNegativeRange;
}

/* Setting I-Range to zero disables it on controller */
void PIDController::setINegativeRange(double pINegativeR) {
    iNegativeRange = pINegativeR;
}

bool PIDController::getInIRange() {
    return getError() > getINegativeRange() && getError() <= getIPositiveRange();
}

/* INTEGRATION ACCUMULATION LIMITS */
/* Setting Accumulation limits to zero disables it on controller */
void PIDController::setAccumLimits(double pAccumLimit) {
    setAccumLimits(-pAccumLimit, pAccumLimit);
}

/* Setting Accumulation limits to zero disables it on controller */
void PIDController::setAccumLimits(double pINegativeAL, double pIPositiveAL) {
    setINegativeAccumLimit(-pINegativeAL);
    setIPositiveAccumLimit(pIPositiveAL);
}

double PIDController::getIPositiveAccumLimit() {
    return iPositiveAccumLimit;
}

/* Setting Accumulation limit to zero disables it on controller */
void PIDController::setIPositiveAccumLimit(double pIPositiveAL) {
    iPositiveAccumLimit = pIPositiveAL;
}

double PIDController::getINegativeAccumLimit() {
    return  iNegativeAccumLimit;
}

/* Setting Accumulation limit to zero disables it on controller */
void PIDController::setINegativeAccumLimit(double pINegativeAL) {
    iNegativeAccumLimit = pINegativeAL;
}

double PIDController::getIAccumulation() {
    return iAccum;
}

/* Sets integral accumulation back to zero */
void PIDController::resetIAccumulation() {
    iAccum = 0;
};

/* D-FILTER */
double PIDController::getDFilter() {
    return dFilter;
}

/* Setting D-filter to zero disables it on controller */
void PIDController::setDFilter(double pDFilter) {
    dFilter = pDFilter;
}

/* Sets integral accumulation and derivative error to zero*/
void PIDController::reset() {
    iAccum = 0;
    prevError = NAN;
    rOCTolerance = 0;
}

/* CONTINOUS WRAP */
bool PIDController::getContinuousWrap() {
    return continousWrap;
}

double PIDController::getLowerDiscontinuity() {
    return lowerDiscontinuity;
}

double PIDController::getHigherDiscontinuity() {
    return  higherDiscontinuity;
}

void PIDController::enableContinuousWrap(double pLowerDisconinuity, double pHigherDiscontinuity) {
    continousWrap = true;
    lowerDiscontinuity = pLowerDisconinuity;
    higherDiscontinuity = pHigherDiscontinuity;
}

void PIDController::disableContinuousWrap() {
    continousWrap = false;
}


double PIDController::getReference() {
    return  reference;
}

void PIDController::setReference(double pReference) {
    setReference(pReference, false);
}

void PIDController::setReference(double pReference, bool resetROCError) {
    reference = pReference;
    if(resetROCError) prevError = NAN;
}

double PIDController::getMeasure() {
    return measure;
}

void PIDController::setMeasure(double pMeasure) {
    measure = pMeasure;
}

double PIDController::getError() {
    if (continousWrap) {
        double errorBound = (higherDiscontinuity - lowerDiscontinuity) / 2.0;
        return MathUtils::inputModulus(reference - measure, -errorBound, errorBound);
    }

    return reference - measure;
}

/* Returns previous ROC error, but due to return nature, good enough for most use cases */
double PIDController::getROCError() {
    return prevROCError;
}

/* Not used on controller, just for telemetry */
double PIDController::getErrorTolerance() {
    return errorTolerance;
}

/* Not used on controller, just for telemetry */
void PIDController::setErrorTolerance(double pTol) {
    errorTolerance = pTol;
}

/* Not used on controller, just for telemetry */
double PIDController::getROCTolerance() {
    return rOCTolerance;
}

/* Not used on controller, just for telemetry */
void PIDController::setROCTolerance(double pTol) {
    rOCTolerance = pTol;
}

/* Not used on controller, just for telemetry */
bool PIDController::inErrorTolerance() {
    return std::abs(getError()) < getErrorTolerance();
}

/* Not used on controller, just for telemetry */
bool PIDController::inROCTolerance() {
    return std::abs(getROCError()) < getROCTolerance();
}

double PIDController::calculate(double pReference, bool resetROCError, double pMeasure) {
    setReference(pReference, resetROCError);
    setMeasure(pMeasure);
    return calculate();
}

double PIDController::calculate(double pReference, double pMeasure) {
    return calculate(pReference, false, pMeasure);
}

double PIDController::calculate() {
    if(std::isnan(prevError)) {
        prevError = getError();
    }

    double pOutput = getkP() * getError();

    /* God forbid if some units need values less 10e-16 to be inside the epsilon */
    if(
        !MathUtils::epsilonEquals(iPositiveRange,0)
        ||
        !MathUtils::epsilonEquals(iNegativeRange,0)) {
        if(getInIRange()) iAccum += getError() * dt;
        else iAccum = 0;
    } else iAccum += getError() * dt;

    if(
        (!MathUtils::epsilonEquals(iPositiveAccumLimit, 0)
        ||
        !MathUtils::epsilonEquals(iNegativeAccumLimit, 0))
        &&
        !MathUtils::epsilonEquals(getkI(), 0)) {
        if(getIAccumulation() < getINegativeAccumLimit())
            iAccum = getINegativeAccumLimit() / getkI();
        else if(getIAccumulation() > getIPositiveAccumLimit())
            iAccum = getIPositiveAccumLimit() / getkI();
    }

    double iOutput = iAccum * getkI();

    double errorROC = (getError() - prevError) / dt;
    if(!MathUtils::epsilonEquals(dFilter, 0)) {
        errorROC = MathUtils::lowPassFilter(
            prevROCError, errorROC, getDFilter());
    }

    double dOutput = getkD() * errorROC;

    prevROCError = errorROC;
    prevError = getError();

    return pOutput + iOutput + dOutput;
}