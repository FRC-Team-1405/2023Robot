// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class BackUp extends CommandBase {
  private SwerveDrive swerve;
  private double totalDistance;
  private static double BackUpDistance;

  static {
    Preferences.initDouble("BackUp/Distance", 6.0);
    BackUpDistance = Preferences.getDouble ("BackUp/Distance", 6.0);
  }

  /** Creates a new BackUp. */
  public BackUp(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    totalDistance = 0.0;
    swerve.driveSpeed(0.1, 0.0, 0.0, false);
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
    totalDistance += swerve.getDistance();
    return totalDistance >= Units.inchesToMeters(BackUpDistance);
  }
}
