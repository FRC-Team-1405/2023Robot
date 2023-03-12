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
import frc.robot.sensors.FusionTimeofFlight;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonSRX upper = new WPI_TalonSRX(Constants.DeviceID.Intake);
  private WPI_TalonSRX conveyerBelt = new WPI_TalonSRX(Constants.DeviceID.ConveyerBelt);
  private WPI_TalonSRX twister = new WPI_TalonSRX(Constants.DeviceID.Twister); 
  private boolean intakeIsDeployed = false;

  private FusionTimeofFlight gamePieceSensor = new FusionTimeofFlight(17); 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    gamePieceSensor.measure();
  }

  private DoubleSolenoid intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                    Constants.PnuematicID.IntakeDeploy, 
                                                    Constants.PnuematicID.IntakeRetract);

  private DoubleSolenoid gate = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                                                   Constants.PnuematicID.GateRaise,
                                                   Constants.PnuematicID.GateLower);       

  public Intake() {}

  public void intakeSuck(){
    upper.set(Constants.Intake.UpperSpeed);
  }
  public void intakePush(){
    upper.set(-Constants.Intake.UpperSpeed);
  }
  public void intakeOff(){
    upper.set(0.0);
  }
  public void intakeDeploy(){
    intake.set(Value.kForward);
    intakeIsDeployed = true;
  }
  public void intakeRetract(){
    intake.set(Value.kReverse);
    intakeIsDeployed = false;
  }
  public void gateRaise(){
    gate.set(Value.kForward);
  }
  public void gateLower(){
    gate.set(Value.kReverse);
  }
  public boolean intakeIsDeployed() {
    return intakeIsDeployed;
  }

  public void conveyerBeltForward(){
    conveyerBelt.set(Constants.Intake.ConveyerBeltSpeed);
  }
  public void conveyerBeltBackward(){
    conveyerBelt.set(-Constants.Intake.ConveyerBeltSpeed);
  }
  public void conveyerBeltEject(){
    conveyerBelt.set(1.0); // max speed for ejecting
  }
  public void conveyerBeltOff(){
    conveyerBelt.set(0.0);
  }
  public void twisterForward(){
    twister.set(Constants.Intake.Twister);
  }
  public void twisterBackward(){
    twister.set(-Constants.Intake.Twister);
  }
  public void twisterOff(){
    twister.set(0.0);
  }
  public void onDisable(){
    upper.set(0.0);
    twister.set(0.0);
    conveyerBelt.set(0.0);
  }
}
