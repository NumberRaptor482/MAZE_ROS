// //
// // Created by User on 7/4/2025.
// //

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


// class PIDController {
// public:
//     PIDController(double pP, double pI, double pD, double pDT);

//     double getkP();
//     void setkP(double pP);

//     double getkI();
//     void setkI(double pI);

//     double getkD();
//     void setkD(double pD);

//     double getDt();
//     void setDt(double pDt);

//     /* INTEGRATION RANGE */
//     /* Setting I-Range to zero disables it on controller */
//     void setIRange(double pIRange);
//     /* Setting I-Range to zero disables it on controller */
//     void setIRange(double pINegativeR, double pIPositiveR);

//     double getIPositiveRange();
//     /* Setting I-Range to zero disables it on controller */
//     void setIPositiveRange(double pIPositiveR);

//     double getINegativeRange();
//     /* Setting I-Range to zero disables it on controller */
//     void setINegativeRange(double pINegativeR);

//     bool getInIRange();

//     /* INTEGRATION ACCUMULATION LIMITS(WIND-UP) */
//     /* Setting Accumulation limits to zero disables it on controller */
//     void setAccumLimits(double accumLimit);
//     /* Setting Accumulation limits to zero disables it on controller */
//     void setAccumLimits(double pINegativeAL, double pIPositiveAL);

//     double getIPositiveAccumLimit();
//     /* Setting Accumulation limit to zero disables it on controller */
//     void setIPositiveAccumLimit(double pIPositiveAL);

//     double getINegativeAccumLimit();
//     /* Setting Accumulation limit to zero disables it on controller */
//     void setINegativeAccumLimit(double pINegativeAL);

//     double getIAccumulation();
//     /* Sets integral accumulation back to zero */
//     void resetIAccumulation();

//     /* D-FILTER */
//     double getDFilter();
//     /* Setting D-filter to zero disables it on controller */
//     void setDFilter(double pDFilter);

//     /* Sets integral accumulation and derivative error to zero*/
//     void reset();

//     /* CONTINUOS WRAP */
//     bool getContinuousWrap();
//     double getLowerDiscontinuity();
//     double getHigherDiscontinuity();

//     void enableContinuousWrap(double pLowerDisconinuity, double pHigherDiscontinuity);
//     void disableContinuousWrap();

//     double getReference();
//     void setReference(double pReference);
//     void setReference(double pReference, bool resetROCError);

//     double getMeasure();
//     void setMeasure(double pMeasure);

//     double getError();
//     double getROCError();

//     /* Not used on controller, just for telemetry */
//     double getErrorTolerance();
//     /* Not used on controller, just for telemetry */
//     void setErrorTolerance(double pTol);

//     /* Not used on controller, just for telemetry */
//     double getROCTolerance();
//     /* Not used on controller, just for telemetry */
//     void setROCTolerance(double pTol);

//     /* Not used on controller, just for telemetry */
//     bool inErrorTolerance();
//     /* Not used on controller, just for telemetry */
//     bool inROCTolerance();

//     double calculate(double pReference, bool resetROCError, double pMeasure);
//     double calculate(double pReference, double pMeasure);
//     double calculate();
// private:
//     double kP;

//     double kI;
//     double iPositiveRange = 0;
//     double iNegativeRange= 0;
//     double iPositiveAccumLimit = 0;
//     double iNegativeAccumLimit = 0;

//     double kD;
//     double dFilter = 0;

//     double dt;

//     bool continousWrap = false;
//     double lowerDiscontinuity = 0.0;
//     double higherDiscontinuity = 0.0;

//     double reference = 0;
//     double measure = 0;
//     double prevError = 0 / 0;
//     double prevROCError = 0.0;

//     double errorTolerance = 0;
//     double rOCTolerance = 0;

//     double iAccum = 0;
// };



#endif //PIDCONTROLLER_H