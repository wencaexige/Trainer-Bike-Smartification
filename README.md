 A BLE Proform Trainer conversion  for Zwift

   Program receives environment date from the Zwift game (via BLE) and adjusts the resistance of 
   the Proform inddor bike.
   Usable with any inddor bike if calibrated, mine is a revamped version of Proform TDF 1.0 where the original console and mainboard were faulty and discarded.
   
   A rather non-standard H-Bridge (MX1508) is used to control the resistance motor, adapt the changeResistance function to your needs and calibrate PID variables.

   I'll be adding code to control incline when I get a suitable SSR and relays.

   I'll also be uploading a schematic (ESP32-C3 Supermini chinese board pins were tricky).

   Reuses code from previous similar projects: 

   for the BLE code https://www.instructables.com/Connecting-Old-Proform-TDF-Bike-to-Zwift-Part-2/
   
   for general architecture of the code (voids, loops,...) https://github.com/kevinmott09/proformBLE

   for PID that makes the integrated DC motor into a servo https://dronebotworkshop.com/custom-servo-motor/
