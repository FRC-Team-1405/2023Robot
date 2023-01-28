// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm arm;
  private Arm.Position position;
  public MoveArm(Arm arm, Arm.Position position) {
    this.arm= arm;
    this.position = position;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setElbowPosition(position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atElbowPosition();
  }
}
