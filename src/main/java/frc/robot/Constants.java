// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DeviceID {
    public static final int ClawOpen = 0;
    public static final int ClawClosed = 1;
    public static final int Elbow = 11;
    public static final int Extension = 12;
  } 

  public static class Arm {
    public static class ElbowPosition {
      public static final int elbowHome = 500;
      public static final int elbowLow = 750;
      public static final int elbowMedium = 1000;
      public static final int elbowHigh = 1250;
    }
  }
  public static class Extension{
    public static class ExtensionPosition {
      public static final int extensionHome = 0;
      public static final int extensionLow = 0;
      public static final int extensionMedium = 0;
      public static final int extensionHigh = 0;
    }
  }
}
