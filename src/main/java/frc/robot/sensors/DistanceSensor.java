package frc.robot.sensors;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {
  private AnalogInput analog;

  public DistanceSensor (int channel){
    analog = new AnalogInput(channel);
  }

  static int maxAverage = 25;
  double iterations = 0;
  List<Double> list = new ArrayList<>();
  // static double MCU_VOLTAGE = 5.0;
  
  

  public boolean isOff() {
    double voltageReading = analog.getVoltage();
    // Robojax.com code for sharp IR sensor
    double voltageAverage = 0;
    // System.out.println("Voltage: " + analog.getValue());
    if (iterations < maxAverage){
      iterations += 1;
      list.add(voltageReading);
    }
    else {
      list.add(voltageReading);
      list.remove(0);
    }
    for (int i = 0; i < list.size(); i++){
      voltageAverage += list.get(i);
    }
    voltageAverage /= iterations;
    
    //analog.getAverageVoltage();

    // eqution of the fitting curve
    //// 33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
    //return 33.9 + (-69.5 * (voltage_temp_average)) + (62.3 * Math.pow(voltage_temp_average, 2)) + (-25.4 * Math.pow(voltage_temp_average, 3)) + (3.83 * Math.pow(voltage_temp_average, 4));
    // return (6762/(analog.getValue()-9))+7;

    //return 15.9019*Math.pow(voltageReading, 2.0)+(-65.6834)*voltageReading+77.1776;
    //return 83.0137*Math.pow(Math.E, voltageReading*-1.08658);
    return (2.28211 * Math.pow(voltageAverage, 3)) + (7.09806 * Math.pow(voltageAverage, 2)) + (-54.4178 * voltageAverage) + 71.5654 > 15;
    // Robojax.com code for sharp IR sensor
  }// readDistance
}
