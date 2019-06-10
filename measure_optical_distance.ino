// This function measures the optical distance as returned by the VL53L0X
// optical time of flight distance sensor.
int measure_optical_distance()
{

   // This creates an object to store the distance measurement from the
   // optical distance sensor
   VL53L0X_RangingMeasurementData_t optical_distance_object;


  // This takes a distance measurement from the optical distance
  // sensor
  optical_sensor.rangingTest( &optical_distance_object, false);

  // This initializes the output distance from the optical sensor
  int tof_distance = -1; // use negative number to indicate out-of-range

  // If the sensor isn't out of range, this returns the measured distance
  if ( optical_distance_object.RangeStatus != 4 )
  {
    // This extracts the measured distance from the optical sensor
    tof_distance = optical_distance_object.RangeMilliMeter;
  }

  // This returns the measured optical distance (or -1 if the sensor is
  // out of range)
  return tof_distance;
  
}
