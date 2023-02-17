// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class ResetGyro extends CommandBase {
  private SwerveDrive swerve = new SwerveDrive();
  private DigitalInput toggle = new DigitalInput(0);
  private DigitalOutput led = new DigitalOutput(1);


  /** Creates a new ResetGyro. */
  public ResetGyro(SwerveDrive swerve) {
    this.swerve = swerve;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(toggle.get()){
      System.out.println("Toggle on");
    }
    else{
      System.out.println("Toggle off");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
