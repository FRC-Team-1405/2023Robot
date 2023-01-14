// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final int DriverControllerPort = 1; 
} 

public final static class SwerveBase {
  public final static int DRIVEFRONTLEFT = 1;
  public final static int DRIVEFRONTRIGHT = 2;
  public final static int DRIVEBACKLEFT = 3;
  public final static int DRIVEBACKRIGHT = 4; 

  public final static int ROTATIONFRONTLEFT = 21;
  public final static int ROTATIONFRONTRIGHT = 22;
  public final static int ROTATIONBACKLEFT = 23;
  public final static int ROTATIONBACKRIGHT = 24; 

  public final static int ENCODERFRONTLEFT = 31;
  public final static int ENCODERFRONTRIGHT = 32;
  public final static int ENCODERBACKLEFT = 33;
  public final static int ENCODERBACKRIGHT = 34; 

  public final static double WHEELRADIUS = 0.0508; 
  // public final static double WHEELCIRCUMFERENCE = WHEELRADIUS * 2 * Math.PI;
  public final static double WHEELCIRCUMFERENCE = Units.inchesToMeters(12.875);
  public final static double DRIVEMOTORENCODERRESOLUTION = 2048;  
  //For converting 100 milleseconds (heretofore referred to as 'ms') to seconds 
  public static final double TIMECONSTANTFORCONVERSION = 10; 
  public static final double GEARATIO = 6;

  //A simple conversion formula to turn encoder velocity (sensor units/100ms) to meters per second 
  public static final double VELOCITYMETERS = 1 / DRIVEMOTORENCODERRESOLUTION * WHEELCIRCUMFERENCE * 1 / GEARATIO * TIMECONSTANTFORCONVERSION;

  // A simple conversion formula to turn meters per second to encoder velocity
  public static final double VELOCITYSENSOR = DRIVEMOTORENCODERRESOLUTION * 1 / WHEELCIRCUMFERENCE * GEARATIO * 1 / TIMECONSTANTFORCONVERSION;

  public static final double MAXANGULARSPEED = 1; 
  public static final double MAXANGULARACCELERARTION = 1; 

  public static final SwerveDriveKinematics KINEMATICS = 
          new SwerveDriveKinematics(
              new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(-13)),    // Front Left
              new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(13)),     // Front Right
              new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13)),    // Back Left
              new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13)));    // Back Right
} 

}
