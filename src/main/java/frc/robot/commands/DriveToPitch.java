// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPitch extends CommandBase {
  /** Creates a new DriveToPitch. */ 
 // private ProfiledPIDController pitchController;
  private SwerveSubsystem swerve; 
  private double pitchFlag = 0; 
  private boolean forward;
  
      //loadConfigs();

  public DriveToPitch(SwerveDrive swerve, boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies. 
    addRequirements(swerve);
    configPIDs(swerve); 

   // pitchController.setGoal(Units.degreesToRadians(angle));
        this.swerve = swerve; 
        this.forward = forward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    pitchFlag = Math.abs(swerve.getPitch());
    swerve.brakeMode();
   // pitchController.reset(Units.degreesToRadians(swerve.getPitch())); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          swerve.drive((forward ? 0.2 : -0.2), 0.0, 0.0); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    swerve.drive(0.0, 0.0, 0.0);
    System.out.println("stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    //  return pitchController.atGoal();
    double pitch = Math.abs(swerve.getPitch()); 
   // SmartDashboard.putNumber("pitchdifference", pitchFlag - pitch);
     // System.out.println("not flipped");
    if( (pitch - pitchFlag) >= -1.0){ 
      pitchFlag = pitch; 
      return false;
    } 
    return true; 
  }
   
   

  private void setPosition(double speed) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
    SwerveModuleState[] moduleStates = swerve.getKinematics().toSwerveModuleStates(chassisSpeeds) ;
    swerve.setModuleStates(moduleStates);
} 
  private void configPIDs(SwerveSubsystem swerve){
   // pitchController =  new ProfiledPIDController(pitch_P, pitch_I, pitch_D, new TrapezoidProfile.Constraints(1, 0.33));
    //pitchController.enableContinuousInput(-Math.PI, Math.PI); 
    //pitchController.setTolerance((Math.PI*2)/100.0);}
}}

