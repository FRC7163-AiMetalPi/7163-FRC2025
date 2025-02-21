package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.LEDs.LEDZone;
import frc.robot.Subsystem.CoralSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  //public static final DriveSub drive = new DriveSub();
  public static final CoralSub coral = new CoralSub();

  /*public static final List<LEDZone> ledZones = new ArrayList<LEDZone>(Arrays.asList(
      new LEDZone(new short[] { 60, 92 }, new short[] { 69, 101 }, 0),
      new LEDZone(70, 91, 1),
      new LEDZone(0, 59, 2)));*/
}
