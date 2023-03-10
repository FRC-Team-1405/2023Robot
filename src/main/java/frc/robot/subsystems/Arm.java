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
  public boolean isZeroized = false; 
  public Arm() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void onDisable(){
    elbow.stop();
    extension.stop();
  }
  public enum Position {
    Home,
    Low,
    ConeMiddle,
    ConeHigh, 
    CubeMiddle, 
    CubeHigh,
    Custom,
    Grab,
    FeederStation, 
    FeederStationStore
  }

  private DoubleSolenoid claw = new DoubleSolenoid( PneumaticsModuleType.CTREPCM, 
                                                    Constants.PnuematicID.ClawOpen, 
                                                    Constants.PnuematicID.ClawClosed);

  private MagicMotionHelper elbow = new MagicMotionHelper(new TalonSRX(Constants.DeviceID.Elbow), 3);
  private MagicMotionHelper extension = new MagicMotionHelper(new TalonSRX(Constants.DeviceID.Extension), 3);

  public void openClaw() {
    claw.set(Value.kReverse);
  }

 
  public void closedClaw() {
    claw.set(Value.kForward);
  }

  private double customArmAngle = Constants.Arm.ElbowPosition.ElbowHome;
  public void adjustElbowPosition(int positionAdjust){
    double currentPosition = elbow.getPosition();
    customArmAngle = currentPosition + positionAdjust;
    elbow.setPosition(customArmAngle);
  }

  public void setElbowPosition(Position position){
     switch (position){
      case Home: 
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowHome);
        break;
      case Low: 
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowLow);
        break;
      case ConeMiddle:
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowConeMedium);
        break;
      case ConeHigh:
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowConeHigh);
        break;
      case Custom:
        elbow.setPosition(customArmAngle);
        break; 
      case CubeMiddle: 
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowCubeMedium); 
        break; 
      case CubeHigh: 
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowCubeHigh);
        break;
      case FeederStation: 
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowFeederStation); 
        break;
      case FeederStationStore: 
        elbow.setPosition(Constants.Arm.ElbowPosition.ElbowFeederStationStorage);
        break; 
    }
  }
  private double customArmExtension = Constants.Arm.ExtensionPosition.ExtensionHome;
  public void adjustExtensionPosition(int positionAdjust){
    double currentPosition = extension.getPosition();
    customArmAngle = currentPosition + positionAdjust;
    extension.setPosition(customArmAngle);
  }

  public void setExtensionPosition(Position position){
    switch (position){
      case Home: 
      extension.setPosition(Constants.Arm.ExtensionPosition.ExtensionHome);
        break;
      case Low: 
        extension.setPosition(Constants.Arm.ExtensionPosition.ExtensionLow);
        break;
      case ConeMiddle:
      case CubeMiddle: 
        extension.setPosition(Constants.Arm.ExtensionPosition.ExtensionMedium);
        break;
      case ConeHigh:
      case CubeHigh: 
        extension.setPosition(Constants.Arm.ExtensionPosition.ExtensionHigh);
        break; 
      case Custom:
        extension.setPosition(customArmExtension);
        break;
      case FeederStation:
      case FeederStationStore: 
        extension.setPosition(Constants.Arm.ExtensionPosition.ExtensionFeederPosition); 
        break;

    }
  }

  public boolean atElbowPosition(){
    return elbow.atPosition();
  }

  public boolean atExtensionPosition(){
    return extension.atPosition();
  } 

//   public void zeroElbow(){ 
//     if(!isZeroized){
//     elbow.percent(0.05); }
//     isZeroized = false;
//  }
}
