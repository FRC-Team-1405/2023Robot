// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmExtension extends CommandBase {
  /** Creates a new SetArmExtension. */
  private Arm arm;
  private Arm.ArmLength length;

  public SetArmExtension(Arm arm, Arm.ArmLength length) {
    this.arm = arm;
    this.length = length;
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setExtensionPosition(length);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atExtensionPosition();
  }
}
