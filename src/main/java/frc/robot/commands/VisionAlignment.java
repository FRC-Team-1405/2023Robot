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
import frc.robot.subsystems.SwerveDrive;
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
    public static double fov_left = -27.0;
    public static double fov_right = 27.0;

    private static byte visionPipeline = Constants.Limelight.Pipeline_Score;

    public static void setVisionPipeline(byte pipeline) {
        visionPipeline = pipeline;
        SmartDashboard.putBoolean("VisionAlignment/TargetIsScore",
                visionPipeline == Constants.Limelight.Pipeline_Score);
        SmartDashboard.putBoolean("VisionAlignment/TargetIsCone", visionPipeline == Constants.Limelight.Pipeline_Cone);
        SmartDashboard.putBoolean("VisionAlignment/TargetIsCube", visionPipeline == Constants.Limelight.Pipeline_Cube);        
    }

    public static byte getVisionPipelien() {
        return visionPipeline;
    }

    static {
        Preferences.initDouble("VisionAlignment/fov/left", -27.0);
        fov_left = Preferences.getDouble("VisionAlignment/fov/left", -27.0);

        Preferences.initDouble("VisionAlignment/fov/right", 27.0);
        fov_right = Preferences.getDouble("VisionAlignment/fov/left", 27.0);
        // loadConfigs();
    }
    private SwerveSubsystem swerve;
    private PIDController xController;
    private ProfiledPIDController zController;
    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;

    public VisionAlignment(DoubleSupplier xSpeed, DoubleSupplier ySpeed, double angle, SwerveDrive swerve) {
        addRequirements(swerve);
        configPIDs(swerve);
        xController.setSetpoint(0);
        zController.setGoal(Units.degreesToRadians(angle));
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.swerve = swerve;
        zController.enableContinuousInput(-Math.PI, Math.PI);
        setVisionPipeline(visionPipeline);
    }

    public void initialize() {
        xController.reset();
        zController.reset(swerve.getPose().getRotation().getRadians());

        limelight.setPipeline(visionPipeline);
        // limelight.setLED(visionPipeline == Constants.Limelight.Pipeline_Score ?
        // LED.On : LED.Off);
        // limelight.setCameraMode(false);
    }

    public void execute() {
        double angle = limelight.getTX();
        if (limelight.hasTarget() && angle >= fov_left && angle <= fov_right) {
            double theta = swerve.getPose().getRotation().getRadians();
            double ySpeed = xController.calculate(-angle);
            double zSpeed = zController.calculate(theta);
            SmartDashboard.putNumber("Error", xController.getPositionError());
            SmartDashboard.putNumber("Speed", ySpeed);
            setPosition(xSpeed.getAsDouble(), ySpeed, zSpeed);
        } else {
            setPosition(xSpeed.getAsDouble(), ySpeed.getAsDouble(), 0);
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        setPosition(0.0, 0.0, 0.0);
        limelight.setPipeline(Constants.Limelight.Pipeline_Drive);
        // limelight.setLED(LED.Off);
        // limelight.setCameraMode(true);
    }

    private void setPosition(double xSpeed, double ySpeed, double zSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, -ySpeed, 0.0);
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
