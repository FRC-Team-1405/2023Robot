// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoDrive extends CommandBase {
  private SwerveDrive swerve;
  private double xSpeed;
  private double ySpeed;
  private double drivenDistance;
  private double targetDistance;

  /** Creates a new BackUp. */
  public AutoDrive(SwerveDrive swerve, double xSpeed, double ySpeed, double distanceMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.targetDistance = distanceMeters;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivenDistance = 0.0;
    swerve.driveSpeed(xSpeed, ySpeed, 0.0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.driveSpeed(0.0, 0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    drivenDistance += swerve.getDistance();
    return drivenDistance >= targetDistance;
  }
}
