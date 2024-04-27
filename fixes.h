#ifndef FIXES_H
#define FIXES_H

#include "fix.h"

class Fixes
{
  public:

  #define NUM_FIXES 20  // 20 works 50 doesn't
  #define MIN_AGE 1000L // TODO - increase

  Fix fixes[NUM_FIXES+1];
  int numFixes = 0;

  double minLat = 0;
  double maxLat = 0;
  double minLng = 0;
  double maxLng = 0;

  double range = 0.0;

  bool add( Fix fix )
  {
    //Serial.println("add");

    if( numFixes > 0 )
    {
      if( fixes[0].age() < MIN_AGE ) // don't have fixes too close together in time
      {
        Serial.println("too early");
        return false;
      }

      if( fix.distanceTo(fixes[0])< 1) // for small motions just replace the old fix with a new one TODO =- increase
      {
        Serial.println("too close");
        fixes[0] = fix;
        return false;
      }


      if( fix.hdop > 160 )
      {
        Serial.println("bad hdop");
        return false;
      }
    }

    //Serial.println("moving");
    // move them all down
    for(int i = numFixes; i >= 1; i -- )
      fixes[i] = fixes[i-1];
    numFixes ++;

    if( numFixes > NUM_FIXES )
      numFixes = NUM_FIXES;

    fixes[0] = fix;

    //Serial.println("added");
    return true;
  };

  void calcScale(Fix other)
  {
    for( int i = 0; i < numFixes; i ++ )
    {
      if( i == 0 )
      {
        minLat = fixes[i].lat;
        maxLat = fixes[i].lat;
        minLng = fixes[i].lng;
        maxLng = fixes[i].lng;
      }

      minLat = min(minLat, fixes[i].lat);
      maxLat = max(maxLat, fixes[i].lat);
      minLng = min(minLng, fixes[i].lng);
      maxLng = max(maxLng, fixes[i].lng);
    }

    // make room to display us on the map too
    if( other.goodFix())
    {
      minLat = min(minLat, other.lat);
      maxLat = max(maxLat, other.lat);
      minLng = min(minLng, other.lng);
      maxLng = max(maxLng, other.lng);
    }

    range = max(maxLat-minLat,maxLng-minLng);

    double minRangeM = 10.0;
    float minRangeDeg = degForM(minRangeM);
    if( range < minRangeDeg )
      range = minRangeDeg ; // protect from div/0

    range = range * 1.2; //give us a margin

  };

  double degForM(double m)
  {
    // one degree of lat is 111,111m
    float mPerDeg = 111111;
    
    float deg = m / mPerDeg;

    return deg;
  };

  double x( int i, double screenWidth )
  {
    return x(fixes[i].lng, screenWidth);
  };

  double y( int i, double screenHeight )
  {
    return y(fixes[i].lat, screenHeight);
  };
  
  double pixForM(float m, double screenWidth)
  {
    double deg = degForM(m);
    double p = screenWidth * deg/range;
    return p;
  }

  double x( double lng, double screenWidth )
  {
    return screenWidth/2.0 + screenWidth * (lng - ((minLng+maxLng)/2.0))/range;
  };

  double y( double lat, double screenHeight )
  {
    return screenHeight - (screenHeight/2.0 + screenHeight * (lat - ((minLat+maxLat)/2.0))/range);
  };

};


#endif
