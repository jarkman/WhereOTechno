#ifndef FIXES_H
#define FIXES_H

#include "fix.h"

class Fixes
{
  public:

  #define NUM_FIXES 800  
  // when the two global Fixes are statically allocated, 20 works 50 doesn't
  // when they are on the heap it works (ie, does not crash) with 800
  // might yet be achingly slow to draw though


  // 240k available
  // One fix is ~120 bytes
  // so ~2000 fixes should be possible
  // remember we have 2 of these so maybe 800 is a rational max
  // might need a better way to move them down too!

  #define MIN_AGE 1000L // TODO - increase

  Fix fixes[NUM_FIXES+1];
  int numFixes = 0;

  double minLat = 0;
  double maxLat = 0;
  double minLng = 0;
  double maxLng = 0;

  double range = 0.0;

  double screenWidth = 0;
  double screenHeight = 0;
  double screenWidthInM = 0;

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

      if( fix.distanceTo(fixes[0])< 5) // for small motions just replace the old fix with a new one
      {
        Serial.println("too close");
        fixes[0] = fix;
        return false;
      }


      if( fix.hdop > 200 )
      {
        Serial.println("bad hdop");
        return false;
      }
    }

    //Serial.println("moving");
    // move them all down
    for(int i = numFixes-1; i >= 1; i -- )
      fixes[i] = fixes[i-1];
    numFixes ++;

    if( numFixes > NUM_FIXES )
      numFixes = NUM_FIXES;

    fixes[0] = fix;

    //Serial.println("added");
    return true;
  };

  void calcScale(Fix other, int _screenWidth, int _screenHeight)
  {
    bool started = false;

    screenWidth = _screenWidth;
    screenHeight = _screenHeight;

    for( int i = 0; i < numFixes; i ++ )
    {
      if( fixes[i].goodFix())
      {
        if(! started )
        {
          minLat = fixes[i].lat;
          maxLat = fixes[i].lat;
          minLng = fixes[i].lng;
          maxLng = fixes[i].lng;
          started = true;
        }

        minLat = min(minLat, fixes[i].lat);
        maxLat = max(maxLat, fixes[i].lat);
        minLng = min(minLng, fixes[i].lng);
        maxLng = max(maxLng, fixes[i].lng);
      }
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


    Serial.print("range is ");
    Serial.print(range, 6);
    
    Serial.print(" deg which is ");
    Serial.print(mForDeg(range));
    Serial.println(" m");
    
    double minRangeM = 10.0;
    float minRangeDeg = degForM(minRangeM);
    if( range < minRangeDeg )
      range = minRangeDeg ; // protect from div/0

    range = range * 1.1; //give us a margin

    screenWidthInM = mForDeg(range);

  };

  double degForM(double m)
  {
    // one degree of lat is 111,111m
    float mPerDeg = 111111;
    
    float deg = m / mPerDeg;

    return deg;
  };

  double mForDeg(double deg)
  {
    // one degree of lat is 111,111m
    float mPerDeg = 111111;
    
    float m = deg * mPerDeg;

    return m;
  };

  double x( int i )
  {
    return x(fixes[i].lng);
  };

  double y( int i )
  {
    return y(fixes[i].lat);
  };
  
  double pixForM(float m)
  {
    double deg = degForM(m);
    double p = screenWidth * deg/range;
    return p;
  }

  double x( double lng )
  {
    return screenWidth/2.0 + screenWidth * (lng - ((minLng+maxLng)/2.0))/range;
  };

  double y( double lat )
  {
    return screenHeight - (screenHeight/2.0 + screenHeight * (lat - ((minLat+maxLat)/2.0))/range);
  };

};


#endif
