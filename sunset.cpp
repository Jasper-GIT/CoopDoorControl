#include "sunset.h"

/* Convert degree angle to radians */
double  degToRad(double angleDeg){
  return (3.14159265359 * angleDeg / 180.0);
}
/*Convert radians to degree angle*/
double radToDeg(double angleRad){
  return (180.0 * angleRad / 3.14159265359);
}


double calcMeanObliquityOfEcliptic(double t){
  double seconds = 21.448 - t*(46.8150 + t*(0.00059 - t*(0.001813)));
  double e0 = 23.0 + (26.0 + (seconds/60.0))/60.0;

  return e0;              // in degrees
}



/***********************************************************************
  Name:    calGeomMeanLongSun							
  Type:    Function									
  Purpose: calculate the Geometric Mean Longitude of the Sun		
  Arguments:										
    t : number of Julian centuries since J2000.0				
  Return value:										
    the Geometric Mean Longitude of the Sun in degrees			
***********************************************************************/
double calcGeomMeanLongSun(double t){
  double L = 280.46646 + t * (36000.76983 + 0.0003032 * t);
  while (L > 360.0){
    L -= 360.0;
  }
  while (L < 0.0){
    L += 360.0;
  }
  return L;		// in degrees
}



double calcObliquityCorrection(double t){
  double e0 = calcMeanObliquityOfEcliptic(t);
  double omega = 125.04 - 1934.136 * t;
  double e = e0 + 0.00256 * cos(degToRad(omega));
  return e;               // in degrees
}



/***********************************************************************
  Name:    calcEccentricityEarthOrbit						
  Type:    Function									
  Purpose: calculate the eccentricity of earth's orbit			
  Arguments:										
  t : number of Julian centuries since J2000.0				
  Return value:										
  the unitless eccentricity							
***********************************************************************/
double calcEccentricityEarthOrbit(double t){
  double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  return e;		// unitless
}

double calcGeomMeanAnomalySun(double t){
  double M = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  return M;		// in degrees
}


double calcEquationOfTime(double t){
  double epsilon = calcObliquityCorrection(t);               
  double l0 = calcGeomMeanLongSun(t);
  double e = calcEccentricityEarthOrbit(t);
  double m = calcGeomMeanAnomalySun(t);
  double y = tan(degToRad(epsilon)/2.0);
  y *= y;
  double sin2l0 = sin(2.0 * degToRad(l0));
  double sinm   = sin(degToRad(m));
  double cos2l0 = cos(2.0 * degToRad(l0));
  double sin4l0 = sin(4.0 * degToRad(l0));
  double sin2m  = sin(2.0 * degToRad(m));
  double Etime = y * sin2l0 - 2.0 * e * sinm + 4.0 * e * y * sinm * cos2l0 - 0.5 * y * y * sin4l0 - 1.25 * e * e * sin2m;

  return radToDeg(Etime)*4.0;	// in minutes of time
}


double calcTimeJulianCent(double jd){
  double T = ( jd - 2451545)/36525;
  return T;
}


double calcSunTrueLong(double t){
  double l0;
  double c;
  double O;
  
  l0 = calcGeomMeanLongSun(t);
  c = calcSunEqOfCenter(t);

  O = l0 + c;
  return O;               // in degrees
}


/***********************************************************************
  Name:    calcSunTrueLong								
  Type:    Function								
  Purpose: calculate the true longitude of the sun				
  Arguments:										
    t : number of Julian centuries since J2000.0				
  Return value:										
    sun's true longitude in degrees						
***********************************************************************/
double calcSunApparentLong(double t){
  double o;
  double  omega;
  double  lambda; 
	
  o = calcSunTrueLong(t);
  omega = 125.04 - 1934.136 * t;
  lambda = o - 0.00569 - 0.00478 * sin(degToRad(omega));
  return lambda;          // in degrees
}


double calcSunDeclination(double t){
  double e;
  double lambda;
  double sint;
  double theta;
	
	
  e = calcObliquityCorrection(t);
  lambda = calcSunApparentLong(t);

  sint = sin(degToRad(e)) * sin(degToRad(lambda));
  theta = radToDeg(asin(sint));
  return theta;           // in degrees
}


double calcHourAngleSunrise(double lat, double solarDec){
  double latRad;
  double sdRad;
  double HA;
	
  latRad = degToRad(lat);
  sdRad  = degToRad(solarDec);

  HA = (acos(cos(degToRad(90.833))/(cos(latRad)*cos(sdRad))-tan(latRad) * tan(sdRad)));

  return HA;              // in radians
}

double calcHourAngleSunset(double lat, double solarDec){	
  double latRad;
  double sdRad;
  double HA;
	
  latRad = degToRad(lat);
  sdRad  = degToRad(solarDec);

  HA = (acos(cos(degToRad(90.833))/(cos(latRad)*cos(sdRad))-tan(latRad) * tan(sdRad)));

  return -HA;              // in radians
}

/***********************************************************************
  Name:    calcJD									
  Type:    Function									
  Purpose: Julian day from calendar day						
  Arguments:										
    year : 4 digit year								
    month: January = 1								
    day  : 1 - 31									
  Return value:										
    The Julian day corresponding to the date					
  Note:											
    Number is returned for start of day.  Fractional days should be	
      added later.									
***********************************************************************/
double calcJD(int year,int month,int day){	
  if (month <= 2){
    year -= 1;
    month += 12;
  }
  int A = floor(year/100);
  int B = 2 - A + floor(A/4);

  double JD = floor(365.25*(year + 4716)) + floor(30.6001*(month+1)) + day + B - 1524.5;
  return JD;
}


/***********************************************************************
  Name:    calcJDFromJulianCent							
  Type:    Function									
  Purpose: convert centuries since J2000.0 to Julian Day.			
  Arguments:										
    t : number of Julian centuries since J2000.0				
  Return value:										
    the Julian Day corresponding to the t value				
***********************************************************************/
double calcJDFromJulianCent(double t){
  double JD = t * 36525.0 + 2451545.0;
  return JD;
}



/***********************************************************************
  Name:    calcSunEqOfCenter							
  Type:    Function									
  Purpose: calculate the equation of center for the sun			
  Arguments:										
    t : number of Julian centuries since J2000.0				
  Return value:										
    in degrees										
***********************************************************************/
double calcSunEqOfCenter(double t){
  double m = calcGeomMeanAnomalySun(t);
  double mrad = degToRad(m);
  double sinm = sin(mrad);
  double sin2m = sin(mrad+mrad);
  double sin3m = sin(mrad+mrad+mrad);
  
  double C = sinm * (1.914602 - t * (0.004817 + 0.000014 * t)) + sin2m * (0.019993 - 0.000101 * t) + sin3m * 0.000289;
  return C;		// in degrees
}


double calcSunriseUTC(double JD, double latitude, double longitude){
  double t = calcTimeJulianCent(JD);
  // *** First pass to approximate sunrise
  double eqTime = calcEquationOfTime(t);
  double solarDec = calcSunDeclination(t);
  double hourAngle = calcHourAngleSunrise(latitude, solarDec);
  double delta = longitude - radToDeg(hourAngle);
  double timeDiff = 4 * delta;	// in minutes of time	
  double timeUTC = 720 + timeDiff - eqTime;	// in minutes	
  double newt = calcTimeJulianCent(calcJDFromJulianCent(t) + timeUTC/1440.0); 

  eqTime = calcEquationOfTime(newt);
  solarDec = calcSunDeclination(newt);
		
  hourAngle = calcHourAngleSunrise(latitude, solarDec);
  delta = longitude - radToDeg(hourAngle);
  timeDiff = 4 * delta;
  timeUTC = 720 + timeDiff - eqTime; // in minutes

  return timeUTC;
}

	
	
/***********************************************************************
  Name:    calcSolNoonUTC								
  Type:    Function									
  Purpose: calculate the Universal Coordinated Time (UTC) of solar	
           for the given day at the given location on earth		
  Arguments:										
    t : number of Julian centuries since J2000.0				
    longitude : longitude of observer in degrees				
  Return value:										
    time in minutes from zero Z							
***********************************************************************/
 
double calcSolNoonUTC(double t, double longitude){
  // First pass uses approximate solar noon to calculate eqtime
  double tnoon = calcTimeJulianCent(calcJDFromJulianCent(t) + longitude / 360.0);
  double eqTime = calcEquationOfTime(tnoon);
  double solNoonUTC = 720 + (longitude * 4) - eqTime; // min
 
  double newt = calcTimeJulianCent(calcJDFromJulianCent(t) - 0.5 + solNoonUTC / 1440.0);
 
  eqTime = calcEquationOfTime(newt);
  // var solarNoonDec = calcSunDeclination(newt);
  solNoonUTC = 720 + (longitude * 4) - eqTime; // min
 
  return solNoonUTC;
}

	
	
	
	
double calcSunsetUTC(double JD, double latitude, double longitude){
  double t = calcTimeJulianCent(JD);
  // *** First pass to approximate sunset
  double noonmin = calcSolNoonUTC(t, longitude);
  double tnoon = calcTimeJulianCent(JD + noonmin / 1440.0);
 
  double eqTime = calcEquationOfTime(tnoon);
  double solarDec = calcSunDeclination(tnoon);
  double hourAngle = calcHourAngleSunset(latitude, solarDec);
  double delta = longitude - radToDeg(hourAngle);
  double timeDiff = 4 * delta;	// in minutes of time	
  double timeUTC = 720 + timeDiff - eqTime;	// in minutes	
  double newt = calcTimeJulianCent(calcJDFromJulianCent(t) + timeUTC/1440.0); 


  eqTime = calcEquationOfTime(newt);
  solarDec = calcSunDeclination(newt);
		
		
  hourAngle = calcHourAngleSunset(latitude, solarDec);
  delta = longitude - radToDeg(hourAngle);
  timeDiff = 4 * delta;
  timeUTC = 720 + timeDiff - eqTime; // in minutes

  return timeUTC;
}