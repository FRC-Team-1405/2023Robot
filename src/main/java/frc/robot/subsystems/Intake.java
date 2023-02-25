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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonSRX upper = new WPI_TalonSRX(13);
  private WPI_TalonSRX lower = new WPI_TalonSRX(14);
  @Override
  public void periodic() {
    // This method will be called once ;per scheduler run
  }

  private DoubleSolenoid intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                    Constants.PnuematicID.IntakeDeploy, 
                                                    Constants.PnuematicID.IntakeRetract);

  public Intake() {}
  public void intakeSuck(){
    upper.set(Constants.Intake.UpperSpeed);
    lower.set(Constants.Intake.LowerSpeed);
  }
  public void intakePush(){
    upper.set(-Constants.Intake.UpperSpeed);
    lower.set(-Constants.Intake.LowerSpeed);
  }
  public void intakeOff(){
    upper.set(0.0);
    lower.set(0.0);
  }
  public void intakeDeploy(){
    intake.set(Value.kForward);
  }
  public void intakeRetract(){
    intake.set(Value.kReverse);
  }
}
