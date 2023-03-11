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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LED;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionAlignment extends CommandBase {
    /** Creates a new VisionAlignment. */
    private static double x_P = 0.1;
    private static double x_I = 0.0;
    private static double x_D = 0.0;
    private static double z_P = 10.0;
    private static double z_I = 0.0;
    private static double z_D = 0.0;
    public Limelight limelight = new Limelight();
    
    private static byte visionPipeline = Constants.Limelight.Pipeline_Score;
    public static void setVisionPipeline(byte pipeline) {
        visionPipeline = pipeline;
        SmartDashboard.putBoolean("VisionAlignment/TargetIsScore", visionPipeline == Constants.Limelight.Pipeline_Score);
        SmartDashboard.putBoolean("VisionAlignment/TargetIsCone", visionPipeline == Constants.Limelight.Pipeline_Cone);
        SmartDashboard.putBoolean("VisionAlignment/TargetIsCube", visionPipeline == Constants.Limelight.Pipeline_Cube);
    }
    public static byte getVisionPipelien(){
        return visionPipeline;
    }
    
    static{
       // loadConfigs();
    } 
    private SwerveSubsystem swerve; 
    private PIDController xController; 
    private ProfiledPIDController zController;  
    private DoubleSupplier forwardSpeed; 

    public VisionAlignment(DoubleSupplier forwardSpeed, double angle, SwerveSubsystem swerve) {
        addRequirements(swerve);
        configPIDs(swerve);
        xController.setSetpoint(0);
        zController.setGoal(Units.degreesToRadians(angle));
        this.forwardSpeed = forwardSpeed; 
        this.swerve = swerve; 
        zController.enableContinuousInput(-Math.PI, Math.PI); 
        setVisionPipeline(visionPipeline);
    }

    public void initialize() {
        xController.reset();
        zController.reset(swerve.getPose().getRotation().getRadians());

        limelight.setPipeline(visionPipeline);
        limelight.setLED(visionPipeline == Constants.Limelight.Pipeline_Score ? LED.On : LED.Off);
        limelight.setCameraMode(true);
    }

    public void execute() {
        if (limelight.hasTarget()) {
            double angle = limelight.getTX();
            double theta = swerve.getPose().getRotation().getRadians();
            double speed = xController.calculate(-angle);
            double zSpeed = zController.calculate(theta);
            SmartDashboard.putNumber("Error", xController.getPositionError());
            SmartDashboard.putNumber("Speed", speed);
            setPosition(speed, zSpeed);
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        setPosition(0.0, 0.0); 
        limelight.setPipeline(Constants.Limelight.Pipeline_Drive);
        limelight.setLED(LED.Off);
        limelight.setCameraMode(false);
    }

    private void setPosition(double speed, double zSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardSpeed.getAsDouble(), speed, zSpeed);
        SwerveModuleState[] moduleStates = swerve.getKinematics().toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(moduleStates);
    }

    private void configPIDs(SwerveSubsystem swerve) {
        xController = new PIDController(x_P, x_I, x_D);
        zController = new ProfiledPIDController(z_P, z_I, z_D,
                new TrapezoidProfile.Constraints(swerve.getMaxAngularSpeed(), swerve.getMaxAngularAcceleration()));
        xController.setTolerance(1);
    }

}
