// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
    // This method will be called once per scheduler run
  }

  public enum Position {
    Home,
    Low,
    Medium,
    High,
  }

  private DoubleSolenoid claw = new DoubleSolenoid( PneumaticsModuleType.CTREPCM, 
                                                    Constants.DeviceID.ClawOpen, 
                                                    Constants.DeviceID.ClawClosed);

  private MagicMotionHelper elbow = new MagicMotionHelper(new TalonSRX(Constants.DeviceID.Elbow), 50.0, 3);
  public void openClaw() {
    claw.set(Value.kReverse);
  }

  public void closedClaw() {
    claw.set(Value.kForward);
  }

  public void setPosition(Position position){
    switch (position){
      case Home: 
        elbow.setPosition(Constants.Arm.ElbowPosition.Home);
        break;
      case Low: 
        elbow.setPosition(Constants.Arm.ElbowPosition.Low);
        break;
      case Medium:
        elbow.setPosition(Constants.Arm.ElbowPosition.Medium);
        break;
      case High:
        elbow.setPosition(Constants.Arm.ElbowPosition.High);
        break;
    }
  }

  public boolean atPosition(){
    return elbow.atPosition();
  }
}
