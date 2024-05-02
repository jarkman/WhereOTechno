#ifndef FIX_H
#define FIX_H

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t getMacAddress();

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
  float batteryVoltage = 0;
  float rssi = 0;
  float snr = 0;

  bool goodFix()
  {
    return lat != 0 && lng != 0 ;
  }
  
  double batteryFraction()
  {
    return batteryVoltage; // fmap(batteryVoltage, 340, 623 * 0.67, 0.0, 1.0);
  }

  double distanceTo( Fix theirFix )
  {
    return TinyGPSPlus::distanceBetween(lat, lng, theirFix.lat, theirFix.lng);
  };

  double headingTo( Fix theirFix )
  {
    return TinyGPSPlus::courseTo(lat, lng, theirFix.lat, theirFix.lng);
  };

  bool decode(uint8_t*buf, uint32_t len)
  {

  if( len != sizeof( Fix ))
  {
    Serial.println("decode - wrong size");
    return false;
  }

  if( 0 != strncmp( (char*) buf, "Techno!", strlen(techno)))
  {
    Serial.println("not techno");
    return false;
  }




  memcpy(this, buf, len);

  if( id != 0x67CDF1D2 && id != 0x52A2DF5C )  // my two t-echo units
  {
    Serial.printf("Wrong id %X, not for us\n", id);
    return false;
  }

  if( id == getMacAddress())
  {
    Serial.printf("id %X is ours!\n", id);
    return false;
  }

  timeWhenReceived = millis(); // so we can calculate age in our own timebase
  
  return true;
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