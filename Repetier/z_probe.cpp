#include "z_probe.h"
#if defined(PROBE_PIN) && (PROBE_PIN > -1)
#include "Reptier.h"

/*      Z Probe Calibration: Place sea-saw under probe and hot end. Level see-saw and record probe reading in configuration.h (Z_PROBE_STOP_POINT)
        Preliminary instructions here: http://reprap.org/wiki/CrashProbe

      2Do:
          Output probe values to text file at each step to determine place where gauss readings are most accurate
          Send current coordinates back to host software
          Code G30 Z5.1  Z is probe offset to zero first probe
          Setup for eeprom variable use
          Quick Calibration M code - Sends sensor value to eeprom
          Sanity check for north or south pole of magnet - maybe auto-figure it out
          allow mm/inch units
          interactive bed leveling. - Set probe to 5mm and scroll readings until user hits exit key.
          Create Bed map
          Leveling Transform Function
          Fix Screw turn leveling output to use proper math.
*/
void X2Steps(float X)     //mostly copied from repetier.pde -> get_coordinates
                                                 //Calculates steps to X, Y position
{
  register long p, steps;

  // X
    if(unit_inches)
      p = X*25.4*axis_steps_per_unit[0];
    else
      p = X*axis_steps_per_unit[0];
    if(relative_mode)
      printer_state.destinationSteps[0] = printer_state.currentPositionSteps[0]+p;
    else
      printer_state.destinationSteps[0] = p+printer_state.offsetX;
}

void Y2Steps(float Y)
{
  register long p, steps;
  //Y
    if(unit_inches)
      p = Y*25.4*axis_steps_per_unit[1];
    else
      p = Y*axis_steps_per_unit[1];
    if(relative_mode)
    {
      printer_state.destinationSteps[1] = printer_state.currentPositionSteps[1]+p;
    }
    else
      printer_state.destinationSteps[1] = p+printer_state.offsetY;
}

void Z2Steps(float Z)
{
  register long p, steps;
 //Z
    if(unit_inches)
      p = Z*25.4*axis_steps_per_unit[2];
    else
      p = Z*axis_steps_per_unit[2];
    if(relative_mode)
      printer_state.destinationSteps[2] = printer_state.currentPositionSteps[2]+p;
    else
        printer_state.destinationSteps[2] = p;
}
//Experimental - calibrate probe and optput data for graphing - manually lower nozzle to bed, Run G92 Z0 and M114 . Then manually raise 10mm. Drop probe and reset retraction bar.
//G34 will lower to .3mm and output readings while collecting data.
//We should be able to use a method like this for automated calibration to find best resolution distance, magnet pole finding, etc
void probe_calibrate()
  {
  int BedProbeValue;                                                                         //Current Hall reading
  int ZStopPoint = -9999;                                                                    //hall reading at desired height
  int BedProbeDeployedValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));  //hall reading when probe deployed
  int BedProbeRetractedValue;                                                                //hall reading when probe retracted
  printPosition(); //update UI

  //2do ??? Force user to drop probe in case command was typed accidentally

  while ( printer_state.currentPositionSteps[2]   > axis_steps_per_unit[2] * .3 )  //move down to .3mm height while gathering data
    {
       move_steps(0,0,1 * Z_HOME_DIR*16,0,homing_feedrate[2],true,false);  //16 to single step
       BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));
       OUT_P_F("Height : ", printer_state.currentPositionSteps[2] / axis_steps_per_unit[2]);
       OUT_P(" : ");
       OUT_P_F_LN("Probe : ", BedProbeValue);

       if ((ZStopPoint < 0) && (printer_state.currentPositionSteps[2]/axis_steps_per_unit[2] <=5)) ZStopPoint = BedProbeValue; //get reading at 5mm height
    }
  BedProbeRetractedValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS)); //this could be moved to a better place if we also want height of retraction

  OUT_P_F_LN("Z_PROBE_STOP_POINT = " , ZStopPoint);
  printPosition();
  }

float Probe_Bed(float x_pos, float y_pos,  int n) //returns Probed Z height. x_pos and y_pos are movement locations.  n is a counting number
//2do: Use -1 for in place probe.
{
    //*** the following commented code is just here for examples ***
    // void move_steps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop)
    // move_steps(0,0,axis_steps_per_unit[2]*2*ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homing_feedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,true);

    //  *** below was pasted from pro level Z Home code ***
    //  if ((MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1 && Z_HOME_DIR==1))
    //  UI_STATUS_UPD(UI_TEXT_HOME_Z);
    //  steps = (printer_state.zMaxSteps-printer_state.zMinSteps) * Z_HOME_DIR;
    //  printer_state.currentPositionSteps[2] = -steps;
    //  move_steps(0,0,2*steps,0,homing_feedrate[2],true,true);
    //  printer_state.currentPositionSteps[2] = 0;
    //  move_steps(0,0,axis_steps_per_unit[2]*-ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homing_feedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,false);  
    //  move_steps(0,0,axis_steps_per_unit[2]*2*ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homing_feedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,true);
    //  printer_state.currentPositionSteps[2] = (Z_HOME_DIR == -1) ? printer_state.zMinSteps : printer_state.zMaxSteps;

   float ProbedHeight;

    //Determine current machine state
    //check for dropped sensor pin
    int BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));  //not sure if the reduce bits is needed. 
    int OldBedProbeValue = -9999;
    int OldBedProbeValue1 = -9999;
    int OldBedProbeValue2 = -9999;
    int OldBedProbeValue3 = -9999;

    //2DO Check that probe is working -  what is the value when probe is unpowered, or unhooked or lacks ground


    //Check that probe is dropped so we don't crash into bed
    while (BedProbeValue > 2300) //this is near the number of my probe when retracted. need to make it variable
      {
      OUT_P_F_LN("Drop Probe to continue - ", BedProbeValue);
      BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));
      }

    //Move head into XY position
    if (x_pos >= 0  || y_pos >=0)
     {
      X2Steps(x_pos);
      Y2Steps(y_pos);
      printer_state.feedrate = 200;
      queue_move(ALWAYS_CHECK_ENDSTOPS,true);
     }

    //set probe to known state - probably not necessary because probe drop movement would have already crashed probe - UNTESTED
    /*
    z_pos = printer_state.currentPositionSteps[2]*inv_axis_steps_per_unit[2]*(unit_inches?0.03937:1);
           BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));
           while (abs(OldBedProbeValue - BedProbeValue) >3)
             {
             OldBedProbeValue = BedProbeValue;
             move_steps(0,0,-1 * Z_HOME_DIR*axis_steps_per_unit[2],0,homing_feedrate[2],true,false);  //move 1 unit up  
             BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));
             OUT_P_F_LN("Zero Probe = ", BedProbeValue);
             OUT_P_F_LN("Old Zero Probe = ", OldBedProbeValue);
             z_pos = printer_state.currentPositionSteps[2]*inv_axis_steps_per_unit[2]*(unit_inches?0.03937:1);           
            }    //the probe should now be off the table but we move 1 more unit to account for table warp.
            move_steps(0,0,-1 * Z_HOME_DIR*axis_steps_per_unit[2],0,homing_feedrate[2],true,false); //move a little more just in case
      */

   int ZStopPoint = Z_PROBE_STOP_POINT;  // hall value for 5mm above tip calibration

   //begin stepped downward probe in 1mm or 1 unit steps. Probe WILL overshoot and then retract in next sequence
   long steps;
   steps = (printer_state.zMaxSteps-printer_state.zMinSteps) * Z_HOME_DIR; //max steps away from z that is posible
   if (n == 1)   //probe count so that first probe sets height to Z_PROBE_HEIGHT_OFFSET
   {
     printer_state.currentPositionSteps[2] = -steps;   //temporarily make printer think it is as far away from home as possible - ** this will cause probing from below to fail. **
   }

    BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));  //not sure if the reduce bits is needed. 
    while (BedProbeValue > ZStopPoint)  // direction will change direction depending on which pole of the magnet is up
    {
       move_steps(0,0,1 * Z_HOME_DIR*axis_steps_per_unit[2],0,homing_feedrate[2],true,false);    //false to allow travel below Z=0
       BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));
       //OUT_P_F_LN("Probe = ", BedProbeValue);
    }

   //full step back up to target
   int Count =0;
   while (BedProbeValue < ZStopPoint)
   {
      OldBedProbeValue3 = OldBedProbeValue2;  //ugly but functional - checks for hall sensor change after 3 steps
      OldBedProbeValue2 = OldBedProbeValue1;
      OldBedProbeValue1 = OldBedProbeValue;
      OldBedProbeValue = BedProbeValue;

      move_steps(0,0,-1 * Z_HOME_DIR*16,0,homing_feedrate[2],true,false);  //16 to single step
      BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));
      OUT_P_F_LN("Probe = ", BedProbeValue); //uncomment to debug probe

      //check that probe is actually changing values. If it doesn't change, it's stuck. :(
 /*    while (BedProbeValue == OldBedProbeValue3)
        {
          OUT_P_F_LN("Probe Stuck = ", BedProbeValue);
          BedProbeValue = (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS));
        }
 */
    } // end full step back up to target

    if (n == 1) //probe count so that only first probe sets height to Z_PROBE_HEIGHT_OFFSET
    {
     printer_state.currentPositionSteps[2] = axis_steps_per_unit[2] * Z_PROBE_HEIGHT_OFFSET;    //sets current height above table 
    }
     ProbedHeight = printer_state.currentPositionSteps[2]*inv_axis_steps_per_unit[2]*(unit_inches?0.03937:1);
    // OUT_P_F_LN("ProbedHeight = ", ProbedHeight);

     //raise up so we don't drag the probe. External code should later force storage retraction.  
     steps = -1L * Z_HOME_DIR * axis_steps_per_unit[2] * 5.0L;   //5 units above target. Fix to also use inch.
     move_steps(0,0,steps,0,homing_feedrate[2],true,false);
     //  Z2Steps(11);
     //  printer_state.feedrate = homing_feedrate[2];
     //  queue_move(ALWAYS_CHECK_ENDSTOPS,true);         /this method doesn't work on the 2nd and 3rd probe for some reason so use move_steps 
    return ProbedHeight;
}

void probe_4points()
{
    float Probe_Avg, Point1, Point2, Point3, Point4;

    Point1 = Probe_Bed(15 - Z_PROBE_X_OFFSET, 15 + Z_PROBE_Y_OFFSET, 1); //PROBE_N replaced with 1 - may have something to do with transform??
    Point2 = Probe_Bed(15 - Z_PROBE_X_OFFSET, Y_MAX_LENGTH -15 + Z_PROBE_Y_OFFSET, 2) ;
    Point3 = Probe_Bed(X_MAX_LENGTH - 15 - Z_PROBE_X_OFFSET, Y_MAX_LENGTH -15 + Z_PROBE_Y_OFFSET, 3);
    Point4 = Probe_Bed(X_MAX_LENGTH - 15 - Z_PROBE_X_OFFSET, 15 + Z_PROBE_Y_OFFSET, 4);

    OUT_P_F_LN("Point 1 = ",Point1);

    OUT_P_F("Point 2 = ",Point2);
    float Point2Multiplier = 0.7; //to account for fact that probe points are not directly above leveling bolts
    OUT_P_F_LN("   Level bed point 2 by turning 3mm bolt clockwise degrees = ", (Point1-Point2)*720*Point2Multiplier);  // 720 degrees per mm

    OUT_P_F("Point 3 = ",Point3);
    float Point3Multiplier = .9;
    OUT_P_F_LN("   Level bed point 3 by turning 3mm bolt clockwise degrees = ", (Point1-Point3)*720*Point3Multiplier);  // 720 degrees per mm

    OUT_P_F("Point 4 = ",Point4);
    float Point4Multiplier = .8;
    OUT_P_F_LN("   Level bed point 4 by turning 3mm bolt clockwise degrees = ", (Point1-Point4)*720*Point4Multiplier);  // 720 degrees per mm

    float zStepMultiplier = 1.4; //to account for fact that z rods are not directly above Y rods
    OUT_P_F_LN("_________________Right motor clockwise steps to level X = ", (((Point3+Point4)/2) - ((Point1+Point2)/2))*(ZAXIS_STEPS_PER_MM/16)*zStepMultiplier);   //16 = don't microstep

    Probe_Avg = (Point1 + Point2 + Point3 + Point4) / 4;
    //2do Calc Point deviation
    OUT_P_F_LN("Probed Average= ",Probe_Avg);

    //At this point, the probe should be 5 units + Z_PROBE_HEIGHT_OFFSET above the last recorded bed height  (Point 3).
    //set new height to include the average calculated from probing
    //Probe 1 is the previously zeroed basepoint so we add on the difference from the average
    printer_state.currentPositionSteps[2] = printer_state.currentPositionSteps[2] + (axis_steps_per_unit[2] * (Point1 - Probe_Avg));
    printPosition();
}

void probe_3points()
{
    float Probe_Avg, Point1, Point2, Point3;

    Point1 = Probe_Bed(15 - Z_PROBE_X_OFFSET, 15 + Z_PROBE_Y_OFFSET, 1); //PROBE_N replaced with 1 - may have something to do with transform??
    OUT_P_F_LN("Point 1 = ",Point1);

    Point2 = Probe_Bed(15 - Z_PROBE_X_OFFSET, Y_MAX_LENGTH -15 + Z_PROBE_Y_OFFSET, 2) ;
    OUT_P_F("Point 2 = ",Point2);
    float Point2Multiplier = 0.2; //to account for fact that probe points are not directly above leveling bolts
    OUT_P_F_LN("   Level bed point 2 by turning 3mm bolt clockwise degrees = ", (Point2-Point1)*720*Point2Multiplier);  // 720 degrees per mm

    Point3 = Probe_Bed(X_MAX_LENGTH - 15 - Z_PROBE_X_OFFSET, Y_MAX_LENGTH/2 + Z_PROBE_Y_OFFSET, 3);
    OUT_P_F("Point 3 = ",Point3);
    float Point3Multiplier = .9;
    OUT_P_F_LN("   Level bed point 3 by turning 3mm bolt clockwise degrees = ", (Point3-Point1)*720*Point3Multiplier);  // 720 degrees per mm
    float zStepMultiplier = 1.4; //to account for fact that z rods are not directly above Y rods
    OUT_P_F_LN("_________________Right motor clockwise steps to level X = ", (Point3-Point1)*(ZAXIS_STEPS_PER_MM/16)*zStepMultiplier);   //16 = don't microstep

    Probe_Avg = (Point1 + Point2 + Point3) / 3;
    OUT_P_F_LN("Probed Average= ",Probe_Avg);

    //At this point, the probe should be 5 units + Z_PROBE_HEIGHT_OFFSET above the last recorded bed height  (Point 3).
    //set new height to include the average calculated from probing
    //Probe 1 is the previously zeroed basepoint so we add on the difference from the average
    printer_state.currentPositionSteps[2] = printer_state.currentPositionSteps[2] + (axis_steps_per_unit[2] * (Point1 - Probe_Avg));
    printPosition();
}

void probe_2points()   //used for setting Z motor heights. Place block on Y rods for probe target. Height doesn't matter but don't crash hotend into bed.
{
    float Probe_Avg, Point1, Point2;
    Point1 = Probe_Bed(15,18,1);
    OUT_P_F_LN("Point1 = ",Point1);
    Point2 = Probe_Bed(125,18,2) ;
    OUT_P_F("Point2 = ",Point2);
    float zStepMultiplier = 1.4; //to account for fact that z rods are not directly above Y rods
    OUT_P_F_LN("     Right Motor Clockwise Steps to level X = ", ((Point2-Point1)*(ZAXIS_STEPS_PER_MM/16))*zStepMultiplier);
    printPosition();
}

void probe_1point()
{
    float  ProbedHeight;
    ProbedHeight = Probe_Bed(-1,-1,1);   //don't move X or Y, probe count number is 1
    OUT_P_F_LN("Probed Z= ",ProbedHeight);
    printPosition();
}

void probe_status()
{
    OUT_P_F_LN("Probe = ", (osAnalogInputValues[BED_PROBE_INDEX]>>(ANALOG_REDUCE_BITS))); 
}

#endif //defined(PROBE_PIN) > -1


