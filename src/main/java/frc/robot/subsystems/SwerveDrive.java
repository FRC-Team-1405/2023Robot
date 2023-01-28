// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase implements SwerveSubsystem {
  //I think we can use these values as our speedlimit, if we make them configureable on Shuffleboard 
  public double maxVelocity; 
  public double maxAngularSpeed; 
   
  //Our swerve modules 
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveBase.DRIVEFRONTLEFT, Constants.SwerveBase.ROTATIONFRONTLEFT, Constants.SwerveBase.ENCODERFRONTLEFT, 45); 
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveBase.DRIVEFRONTRIGHT, Constants.SwerveBase.ROTATIONFRONTRIGHT, Constants.SwerveBase.ENCODERFRONTRIGHT, -45); 
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveBase.DRIVEBACKLEFT, Constants.SwerveBase.ROTATIONBACKLEFT, Constants.SwerveBase.ENCODERBACKLEFT, -45); 
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveBase.DRIVEBACKRIGHT, Constants.SwerveBase.ROTATIONBACKRIGHT, Constants.SwerveBase.ENCODERBACKRIGHT, 45); 
  //Our gyro (used to determine robot heading)
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); // = new AHRS(SPI.Port.kMXP); Causing exception

  private final SwerveDriveOdometry odometry = 
          new SwerveDriveOdometry(Constants.SwerveBase.KINEMATICS, gyro.getRotation2d(), getSwerveModulePositions()); 

  public SwerveDrive() {
    //I am making the maxVelocity configurable so we can ajdust our "speedlimit"
    Preferences.initDouble("SwerveDrive/Speed Limit", 6); 
    maxVelocity = Preferences.getDouble("SwerveDrive/Speed Limit", 6) ;
    Preferences.initDouble("SwerveDrive/Rotation Speed Limit", 6.5); 
    maxAngularSpeed = Preferences.getDouble("SwerveDrive/Rotation Speed Limit", 6.5) ;
    //It may be useful to reset the gyro like this every boot-up. I believe we did this our old code
    if (gyro != null)
      gyro.reset();

    enableFieldOriented(isFieldOrientedEnabled);
  }


  @Override
  public void periodic() {
    updateOdometry(); 
    SmartDashboard.putNumber("angle", gyro.getAngle()); 
    SmartDashboard.putNumber("pitch", gyro.getPitch());
    SmartDashboard.putNumber("distance x",odometry.getPoseMeters().getX()); 
    SmartDashboard.putNumber("distance y",odometry.getPoseMeters().getY()); 
  }

  public void drive(double xPercent, double yPercent, double rotationPercent){ 
    driveSpeed(xPercent * maxVelocity, yPercent * maxVelocity, rotationPercent * maxAngularSpeed, fieldOriented());
  } 

  public void driveSpeed(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented){
     SmartDashboard.putNumber("DriveTo/Speed/x", xSpeed);
    // SmartDashboard.putNumber("DriveTo/Speed/y", ySpeed);
    // SmartDashboard.putNumber("DriveTo/Speed/z", rotationSpeed);

    SwerveModuleState[] swerveModuleStates = Constants.SwerveBase.KINEMATICS.toSwerveModuleStates(
      fieldOriented
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 
                                              ySpeed, 
                                              rotationSpeed, 
                                              Rotation2d.fromDegrees(getGyroAngle())) 
      : new ChassisSpeeds(xSpeed, 
                          ySpeed, 
                          rotationSpeed)); 
    //This function should limit our speed to the value we set (maxVelocity)

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity); 

    // SmartDashboard.putNumber("angle", getGyroAngle()); 
    
    setModuleStates(swerveModuleStates);
  }

  public void updateOdometry(){ 
    odometry.update( Rotation2d.fromDegrees(-getGyroAngle()), getSwerveModulePositions());
    // SmartDashboard.putNumber("SwerveDrive/Pose/X", Units.metersToInches(odometry.getPoseMeters().getX()));
    // SmartDashboard.putNumber("SwerveDrive/Pose/Y", Units.metersToInches(odometry.getPoseMeters().getY()));
    // SmartDashboard.putNumber("SwerveDrive/Pose/Z", odometry.getPoseMeters().getRotation().getDegrees());
  }

  public boolean fieldOriented(){ 
    return (gyro != null && isFieldOrientedEnabled) ? true : false;
  }

  protected boolean isFieldOrientedEnabled = true;
  public void enableFieldOriented(boolean value){
    isFieldOrientedEnabled = value;
    SmartDashboard.putBoolean("Drive by Field Oriented", isFieldOrientedEnabled);
  }
  // FieldOriented and Gyro control mapped to control stick button on a true/false boolean

  public void setStartLocation(Pose2d pose) {
    gyro.setAngleAdjustment(pose.getRotation().getDegrees() - getGyroAngle());
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), getPose());
  } 

  public Pose2d getPose(){ 
    Pose2d pose = odometry.getPoseMeters();
    return new Pose2d( pose.getX(), pose.getY(), Rotation2d.fromDegrees(getGyroAngle()) ); 
  } 

  public void setModuleStates(SwerveModuleState[] states){ 
    frontLeft.setDesiredState(states[0], parkingBrake);
    frontRight.setDesiredState(states[1], parkingBrake); 
    backLeft.setDesiredState(states[2], parkingBrake);
    backRight.setDesiredState(states[3], parkingBrake);
  }
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getSwerveModulePosition(),
      frontRight.getSwerveModulePosition(),
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()
    };
  }
  

  @Override
  public void setPose(Pose2d pose) {
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), getPose());
  }

  public SwerveDriveKinematics getKinematics() {
    return Constants.SwerveBase.KINEMATICS ;
  }

  public double getMaxSpeed() {
    return 3.0;
  }

  public double getMaxAcceleration() {
      return 1.0;
  }

  public double getMaxAngularSpeed() {
      return Math.PI*2;
  }

  public double getMaxAngularAcceleration() {
      return Math.PI;
  } 

  private double distance = 0.0;
  public void resetDistance(){
    frontLeft.getDistance();
    frontRight.getDistance();
    backLeft.getDistance();
    backRight.getDistance();
    distance = 0.0;
  }
  public double getDistance(){
    distance += (frontLeft.getDistance() + frontRight.getDistance() + backLeft.getDistance() + backRight.getDistance()) / 4.0;
    return distance / Constants.SwerveBase.DRIVEMOTORENCODERRESOLUTION / Constants.SwerveBase.GEARATIO * Constants.SwerveBase.WHEELCIRCUMFERENCE;
  } 

 public double getGyroAngle(){
  if (gyro == null)
    return 0.0 ;

  return gyro.getAngle();
 } 

 public double getPitch(){
  return gyro.getPitch();
 } 

 public void brakeMode(){ 
  frontLeft.brakeMode();
  backLeft.brakeMode(); 
  frontRight.brakeMode(); 
  backRight.brakeMode();
 } 
 private boolean parkingBrake = true; 
 public void parkingBrake(boolean enabled){ 
  parkingBrake = enabled; 
 }

}
