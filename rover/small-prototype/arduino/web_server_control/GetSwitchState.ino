// GET REAL-TIME STATE
// Send the rover's sensor and state data to the web browser
void GetSwitchState(WiFiClient cl, float timeX, float x, float y, float z, float vx, float vy, float vz, float anglex, float angley, float anglez, float temp)
{
  // Format and send timestamp
  String strtime = "<text style='font-size:35px;'>Time ";
  strtime.concat(timeX);
  strtime += " <br>";

  // Format and send acceleration data
  String stracc = "Acceleration x ";
  stracc.concat(x);
  stracc += " Acceleration y ";
  stracc.concat(y);
  stracc += " Acceleration z ";
  stracc.concat(z);
  stracc += "<br>";

  // Format and send velocity data
  String strvel = "Velocity vx ";
  strvel.concat(vx);
  strvel += " Velocity vy ";
  strvel.concat(vy);
  strvel += " Velocity vz ";
  strvel.concat(vz);
  strvel += "<br>";

  // Format and send orientation angles
  String strangle = "Angle X ";
  strangle.concat(anglex);
  strangle += " Angle Y ";
  strangle.concat(angley);
  strangle += " Angle Z ";
  strangle.concat(anglez);
  strangle += "<br>";

  // Format and send temperature (optional, currently disabled)
  String strtemp = "Temperature °C ";
  strtemp.concat(temp);

  // Send all composed strings to the client
  cl.print(strtime);
  cl.print(stracc);
  cl.print(strvel);
  cl.print(strangle);
  //cl.print(strtemp); // Uncomment to show temperature
}
