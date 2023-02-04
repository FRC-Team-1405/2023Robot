// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private WPI_TalonSRX belt = new WPI_TalonSRX(15);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(16);
  
  private DoubleSolenoid gate = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                              Constants.PnuematicID.GateOpen, 
                                              Constants.PnuematicID.GateClose);
  
  public Indexer() {}
  public void beltTake(){
    belt.set(Constants.Indexer.BeltSpeed);
  }
  public void beltReverse(){
    belt.set(Constants.Indexer.BeltSpeedReverse);
  }
  public void beltOff(){
    belt.set(0.0);
  }
  public void gateRaise(){
    gate.set(Value.kForward);
  }
  public void gateLower(){
    gate.set(Value.kReverse);
  }
  public void feederIntake(){
    feeder.set(Constants.Indexer.FeederForward);
  }
  public void feederReverse(){
    feeder.set(Constants.Indexer.FeederReverse);
  }
  public void feederOff(){
    feeder.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
