// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive; 
import frc.robot.commands.AutoBalance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {

  public static CommandBase RampDriveAuto(SwerveDrive swerve, boolean forward){
    return Commands.sequence(new DriveToPitch(swerve, forward), 
                            new RunCommand(()-> swerve.drive((forward ? 0.2 : -0.2), 0, 0), swerve).withTimeout(2.5),
                            new RunCommand(()-> swerve.drive(0, 0, 0), swerve).withTimeout(2),
                            new AutoBalance(swerve, !forward));
  }

  public static CommandBase AutoFieldDrive(SwerveDrive swerve, double x, double y, double z){
    return new SwerveDriveCommand(() -> { return x; },
                                  () -> { return y; },
                                  () -> { return z; },
                                  swerve);

  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
