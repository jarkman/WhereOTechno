#ifndef FIX_H
#define FIX_H

class Fix
{
  public:

  char techno[8] = "Techno!";
  uint32_t id;
  double lat = 0;
  double lng = 0;
  uint32_t ageWhenReceived = 0;
  uint32_t timeWhenReceived = 0;
  uint32_t satellites = 0;
  int32_t hdop = 0;
  double batteryVoltage = 0;

  bool goodFix()
  {
    return lat != 0 && lng != 0 ;
  }
  
  double distanceTo( Fix theirFix )
  {
    return TinyGPSPlus::distanceBetween(lat, lng, theirFix.lat, theirFix.lng);
  };

  double headingTo( Fix theirFix )
  {
    return TinyGPSPlus::courseTo(lat, lng, theirFix.lat, theirFix.lng);
  };

  void decode(uint8_t*buf, uint32_t len)
  {

  if( len != sizeof( Fix ))
  {
    Serial.println("decode - wrong size");
    return;
  }

  if( 0 != strncmp( (char*) buf, "Techno!", strlen(techno)))
  {
    Serial.println("not techno");
    return;
  }

  memcpy(this, buf, len);

  timeWhenReceived = millis(); // so we can calculate age in our own timebase
  
};

uint32_t age()
{
  return ageWhenReceived + (millis() - timeWhenReceived);
};

int encode(uint8_t*buf)
{
  ageWhenReceived = ageWhenReceived + (millis() - timeWhenReceived); // so we send total time since fix - doing a sneaky thing here, using this to mean sometime 'since received form gps' and sometimes 'since received from lora


  memcpy(buf, this, sizeof( Fix ));
  return sizeof( Fix );

};

};


#endif