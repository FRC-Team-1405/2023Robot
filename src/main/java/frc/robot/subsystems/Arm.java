// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.MagicMotionHelper;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {
  }

  @Override
  public void periodic() {
    elbowAtPosition = elbow.atPosition();
    if (elbowAtPosition && !breakClosed) {
      armBreak.set(Value.kForward);
      elbow.stop();
      breakClosed = true;
    }
    // This method will be called once per scheduler run
  }

  public enum Position {
    Home,
    Low,
    Medium,
    High,
  }

  private boolean elbowAtPosition = false;
  private boolean breakClosed = false;

  private DoubleSolenoid claw = new DoubleSolenoid( PneumaticsModuleType.CTREPCM, 
                                                    Constants.PnuematicID.ClawOpen, 
                                                    Constants.PnuematicID.ClawClosed);

  private MagicMotionHelper elbow = new MagicMotionHelper(new TalonSRX(Constants.DeviceID.Elbow), 50.0, 3);
  private MagicMotionHelper extension = new MagicMotionHelper(new TalonSRX(Constants.DeviceID.Extension), 50.0, 3);

  private DoubleSolenoid armBreak = new DoubleSolenoid( PneumaticsModuleType.CTREPCM,
                                                       Constants.PnuematicID.ArmBreakClose,
                                                       Constants.PnuematicID.ArmBreakOpen);
  public void openClaw() {
    claw.set(Value.kReverse);
  }

 
  public void closedClaw() {
    claw.set(Value.kForward);
  }

  public void setElbowPosition(Position position){
    elbowAtPosition = false;
    armBreak.set(Value.kReverse);
    breakClosed = false;
    switch (position){
      case Home: 
        elbow.setPosition(Constants.Arm.ElbowPosition.elbowHome);
        break;
      case Low: 
        elbow.setPosition(Constants.Arm.ElbowPosition.elbowLow);
        break;
      case Medium:
        elbow.setPosition(Constants.Arm.ElbowPosition.elbowMedium);
        break;
      case High:
        elbow.setPosition(Constants.Arm.ElbowPosition.elbowHigh);
        break;
    }
  }

  public void setExtensionPosition(Position position){
    switch (position){
      case Home: 
        extension.setPosition(Constants.Arm.ExtensionPosition.extensionHome);
        break;
      case Low: 
        extension.setPosition(Constants.Arm.ExtensionPosition.extensionLow);
        break;
      case Medium:
        extension.setPosition(Constants.Arm.ExtensionPosition.extensionMedium);
        break;
      case High:
        extension.setPosition(Constants.Arm.ExtensionPosition.extensionHigh);
        break;
    }
  }

  public boolean atElbowPosition(){
    return elbowAtPosition;
  }

  public boolean atExtensionPosition(){
    return extension.atPosition();
  }
}