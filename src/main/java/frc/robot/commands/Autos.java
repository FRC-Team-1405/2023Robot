// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  public static CommandBase AutoBalance(SwerveDrive swerve){
    return Commands.sequence(new DriveToPitch(swerve), new WaitCommand(3.0), new Balance(swerve));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
