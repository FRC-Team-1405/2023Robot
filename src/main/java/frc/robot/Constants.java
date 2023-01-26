// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

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
  } 

  public static class Arm {
      static {
        Preferences.initInt("Arm/Home/Elbow", 750);
        Home_Elbow = Preferences.getInt("Arm/Home/Elbow", 750);
        
        Preferences.initInt("Arm/Low/Elbow", 750);
        Low_Elbow = Preferences.getInt("Arm/Low/Elbow", 750);
        
        Preferences.initInt("Arm/Home/Elbow", 1000);
        Medium_Elbow = Preferences.getInt("Arm/Medium/Elbow", 1000);

        Preferences.initInt("Arm/Home/Elbow", 1250);
        High_Elbow = Preferences.getInt("Arm/High/Elbow", 1250);

        Preferences.initDouble("Arm/Threshold/Elbow", 50.0);
        Threshold_Elbow = Preferences.getDouble("Arm/Threshold/Elbow", 50.0);

        Preferences.initInt("Arm/Settle/Elbow", 3);
        Settle_Elbow = Preferences.getInt("Arm/Settle/Elbow", 3); 
      }
      public static final int Home_Elbow;
      public static final int Low_Elbow;
      public static final int Medium_Elbow;
      public static final int High_Elbow;
      public static final double Threshold_Elbow;
      public static final int Settle_Elbow;
  }
}
