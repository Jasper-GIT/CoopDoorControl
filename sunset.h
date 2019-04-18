//#include <time.h>
#include <math.h> //for the floor function
//#include <stdlib.h>
//#include <stdio.h>

#define pi 3.14159265359;

double calcSunsetUTC(double JD, double latitude, double longitude);
double calcSunriseUTC(double JD, double latitude, double longitude);
double calcSunEqOfCenter(double t);
double calcJDFromJulianCent(double t);
double calcJD(int year,int month,int day);
double calcHourAngleSunset(double lat, double solarDec);
double calcHourAngleSunrise(double lat, double solarDec);
double calcSunDeclination(double t);
double calcSunApparentLong(double t);
double calcSunTrueLong(double t);
double calcTimeJulianCent(double jd);
double calcEquationOfTime(double t);
double calcGeomMeanAnomalySun(double t);
double calcEccentricityEarthOrbit(double t);
double calcObliquityCorrection(double t);
double calcGeomMeanLongSun(double t);
double calcMeanObliquityOfEcliptic(double t);
double radToDeg(double angleRad);
double  degToRad(double angleDeg);
double calcSolNoonUTC(double t, double longitude);
void CalculateSunset(void);
