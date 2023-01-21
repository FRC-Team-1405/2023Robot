// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {
   armSensorPosition =(int) elbow.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    if (armIsMoving){
      int pos = (int)elbow.getSelectedSensorPosition();
      
    }
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

  private WPI_TalonSRX elbow = new WPI_TalonSRX(Constants.DeviceID.Elbow);
  private Position position = Position.Home;
  private int armSensorPosition;
  private boolean armIsMoving = false;

  public void openClaw() {
    claw.set(Value.kReverse);
  }

  public void closedClaw() {
    claw.set(Value.kForward);
  }

  public void setPosition(Position position){
    switch (position){
      case Home: 
        elbow.set(ControlMode.MotionMagic, Constants.Arm.ElbowPosition.Home);
        break;
      case Low: 
        elbow.set(ControlMode.MotionMagic, Constants.Arm.ElbowPosition.Low);
        break;
      case Medium:
        elbow.set(ControlMode.MotionMagic, Constants.Arm.ElbowPosition.Medium);
        break;
      case High:
        elbow.set(ControlMode.MotionMagic, Constants.Arm.ElbowPosition.High);
        break;
    }
    armIsMoving = true;
  }
}
