// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class NewBalance extends CommandBase {
  /** Creates a new NewBalance. */
  private SwerveDrive swerve;
  private double pitchError;
  public NewBalance(SwerveDrive swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchError = Math.abs(swerve.getPitch());
  }

  private int tick = 0;
  private boolean drive = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tick += 1;
    if (tick % 10 == 0){
      if (drive)
        swerve.driveSpeed(0, 0, 0, true);
      } else {
        swerve.driveSpeed(0.4, 0, 0, true);
      }
      drive = !drive;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getPitch()) < pitchError; 
  }
}
