package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {

  AnalogInput analog = new AnalogInput(4);
  // static int AVERAGE_OF = 50;
  // static double MCU_VOLTAGE = 5.0;
  
  

  public double readDistance() {
    double voltageReading = analog.getVoltage();
    // Robojax.com code for sharp IR sensor
    //double voltage_temp_average = 0;
    // System.out.println("Voltage: " + analog.getValue());
    /*for (int i = 0; i < AVERAGE_OF; i++) {
      double sensorValue = analog.getValue();
      // delay(1);
      voltage_temp_average += analog.getVoltage();

    }
    voltage_temp_average /= AVERAGE_OF;
    */
    //analog.getAverageVoltage();

    // eqution of the fitting curve
    //// 33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
    //return 33.9 + (-69.5 * (voltage_temp_average)) + (62.3 * Math.pow(voltage_temp_average, 2)) + (-25.4 * Math.pow(voltage_temp_average, 3)) + (3.83 * Math.pow(voltage_temp_average, 4));
    // return (6762/(analog.getValue()-9))+7;

    //return 15.9019*Math.pow(voltageReading, 2.0)+(-65.6834)*voltageReading+77.1776;
    return 83.0137*Math.pow(Math.E, voltageReading*-1.08658);
    // Robojax.com code for sharp IR sensor
  }// readDistance
}
