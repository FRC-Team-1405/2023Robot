// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionAlignment extends CommandBase {
  /** Creates a new VisionAlignment. */ 
  private static double x_P = 0.1 ;
    private static double x_I = 0.0 ;
    private static double x_D = 0.0 ;
    public Limelight limelight = new Limelight();
    static{
       // loadConfigs();
    }
    private PIDController xController;
    private SwerveSubsystem swerve; 
    private DoubleSupplier forwardSpeed; 

    public VisionAlignment(DoubleSupplier forwardSpeed, double angle, SwerveSubsystem swerve) {
        addRequirements(swerve);
        
        configPIDs(swerve);
        xController.setSetpoint(angle); 
        this.forwardSpeed = forwardSpeed; 
        this.swerve = swerve;
    }
    
    public void initialize() { 
        xController.reset();
    }

    public void execute() { 
      if(limelight.hasTarget()){
        double angle = limelight.getTX();
        double speed = xController.calculate( angle ); 
        SmartDashboard.putNumber("Error", xController.getPositionError()); 
        SmartDashboard.putNumber("Speed", speed); 
        setRotation(speed);}
    }

    public boolean isFinished() {
        return false;
      }    

    public void end(boolean interrupted) {
        setRotation(0.0);
    }

    private void setRotation(double speed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardSpeed.getAsDouble(), -speed, 0);
        SwerveModuleState[] moduleStates = swerve.getKinematics().toSwerveModuleStates(chassisSpeeds) ;
        swerve.setModuleStates( moduleStates);
    }

    private void configPIDs(SwerveSubsystem swerve){
        xController =  new PIDController(x_P, x_I, x_D);
       // xController.enableContinuousInput(-Math.PI, Math.PI); 
       xController.setTolerance(1);
    }

    private static void loadConfigs(){
             Preferences.initDouble("VisionAlignment/X/P", x_P);
             Preferences.initDouble("VisionAlignment/X/I", x_I);
             Preferences.initDouble("VisionAlignment/X/D", x_D);
        x_P = Preferences.getDouble("VisionAlignment/X/P", x_P);
        x_I = Preferences.getDouble("VisionAlignment/X/I", x_I);
        x_D = Preferences.getDouble("VisionAlignment/X/D",  x_D);
    }

}
