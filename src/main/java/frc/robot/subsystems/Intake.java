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
  private WPI_TalonSRX left = new WPI_TalonSRX(13);
  private WPI_TalonSRX right = new WPI_TalonSRX(14);
  @Override
  public void periodic() {
    // This method will be called once ;per scheduler run
  }

  private DoubleSolenoid intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                    Constants.PnuematicID.IntakeDeploy, 
                                                    Constants.PnuematicID.IntakeRetract);

  public Intake() {}
  public void intakeSuck(){
    left.set(Constants.Intake.LeftSpeed);
    right.set(Constants.Intake.RightSpeed);
  }
  public void intakePush(){
    left.set(-Constants.Intake.LeftSpeed);
    right.set(-Constants.Intake.RightSpeed);
  }
  public void intakeOff(){
    left.set(0.0);
    right.set(0.0);
  }
  public void IntakeDeploy(){
    intake.set(Value.kForward);
  }
  public void IntakeRetract(){
    intake.set(Value.kReverse);
  }
}
