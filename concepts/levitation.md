# Concept for "Levitation"
## Description
Our "levitation" design is to help the robot get off of the edge of the charging station to create more space for more robots. We will have 2 sensors that will detect a change in elevation while driving off the ramp. Once it senses the change in elevation, it will stop and deploy a rubber foot. THe robot will then drive off the ramp and the rubber foot will keep it stable from falling. The rubber foot will be a cylinder with a limit, so we will know when to stop.
 
## Cases
1. Perfectly aligned: The driver will balance the robot in the center of the platform. Then, the driver will press a button and it will turn on levitation. Once the button is pressed, the robot will start driving off to the right slowly. The flight sensor will be constantly detecting the elevation. Once there is a dramatic change in elevation, the robot will stop moving. The robot will stop and then deploy a rubber foot. The robot will drive off until the limit and then stop until it reaches the limit of the cylinder. 

2. Slightly rotated: If the robot's rotation is slightly off and the driver presses the levitation button, the robot will slowly drive off to the side and then stop once one of the sensors detects a change in elevation. If the other sensor has not detected a change, we will slowly rotate the robot around the detected pivot. Once both pivots have a change in elevation, the robot will stop, deploy the rubber foot, and drive off.

3. Case 3:....

## Pseudo - code
```
function levitate () {
  while (leftSensor < 14 || rightSensor < 14){ 
    driveRightSlowly();
  }
  
  if (leftSensorChange){
    while(rightSensor < 14){
      pivotOnLeftWheel();
    }
  } else(rightSensorChange){
    while(leftSensor < 14){
      pivotOnRightWheel();
    }
  }
  rampDriveOff();//deployes rubber foot and drives off in one function
}
```
