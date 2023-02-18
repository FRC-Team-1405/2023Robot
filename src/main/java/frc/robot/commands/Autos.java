// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;


public final class Autos {

  private enum Position {
    Left,
    Center,
    Right,
  };

  private enum AutoCommand {
    Score_High,
    Score_Middle,
    Score_Low,
    Leave_Community,
    Drive_Over_Ramp,
    Balance_Forward,
    Balance_Backwards,
    Do_Nothing,
  };

  private static final SendableChooser<Position> position = new SendableChooser<>();  
  private static final SendableChooser<AutoCommand> step_1 = new SendableChooser<>();
  private static final SendableChooser<AutoCommand> step_2 = new SendableChooser<>();
  private static final SendableChooser<AutoCommand> step_3 = new SendableChooser<>();

  public static void initAutoSelector(){
    position.setDefaultOption("Center", Position.Center);
    position.addOption("Left", Position.Left);
    position.addOption("Right", Position.Right);
    SmartDashboard.putData("Auto/Position", position);
    
    step_1.setDefaultOption("High", AutoCommand.Score_High);
    step_1.addOption("Middle", AutoCommand.Score_Middle);
    step_1.addOption("Low", AutoCommand.Score_Low);
    step_1.addOption("Do nothing!", AutoCommand.Do_Nothing);
    
    SmartDashboard.putData("Auto/Step 1", step_1);
        
    step_2.setDefaultOption("Drive over ramp", AutoCommand.Drive_Over_Ramp);
    step_2.addOption("Backwards Balance", AutoCommand.Balance_Backwards);
    step_2.addOption("Leave Community", AutoCommand.Leave_Community);
    step_2.addOption("Do nothing!", AutoCommand.Do_Nothing);
    SmartDashboard.putData("Auto/Step 2", step_2);

    step_3.setDefaultOption("Backwards Balance", AutoCommand.Balance_Backwards);
    step_3.addOption("Forwards Balance", AutoCommand.Balance_Forward);
    step_3.addOption("Do Nothing!", AutoCommand.Do_Nothing);
    SmartDashboard.putData("Auto/Step 3", step_3);
  }

  public static CommandBase getSelectedAuto(SwerveDrive swerve){
    Position pos = position.getSelected();
    CommandBase autoCommand = Commands.print("Auto Command");

    switch (step_1.getSelected()){
      case Score_High:    autoCommand.andThen( scoreHigh() ) ;    
                          break;
      case Score_Middle:  autoCommand.andThen( scoreMiddle() ) ;  
                          break;
      case Score_Low:     autoCommand.andThen( scoreLow() ) ;     
                          break;
      default:            autoCommand.andThen( Commands.print("Skipping Step 1") ) ; 
                          break;
      }

      switch (step_2.getSelected()){
        case Drive_Over_Ramp: autoCommand.andThen( DriveOverRamp(swerve, false) ) ;    
                              break;
        case Balance_Backwards: autoCommand.andThen( BalanceAuto(swerve, false) ) ;  
                              break;
        case Leave_Community: autoCommand.andThen(leaveCommunity(swerve, false));
                              break;
        default:              autoCommand.andThen( Commands.print("Skipping Step 2") ) ; 
                              break;
      }

      switch (step_3.getSelected()){
        case Balance_Backwards: autoCommand.andThen( BalanceAuto(swerve, false) );
                                break;
        case Balance_Forward: autoCommand.andThen(BalanceAuto(swerve, true));
                                break;                        
        default:                autoCommand.andThen( Commands.print("Skipping Step 3") );
                                break;
      }
  
      autoCommand.andThen( Commands.print("Auto command stopped"));
 
      return autoCommand;
  }
  

  public static CommandBase RampDriveAuto(SwerveDrive swerve, boolean forward){
    return Commands.sequence(new DriveToPitch(swerve, forward), 
                            new RunCommand(()-> swerve.drive((forward ? 0.2 : -0.2), 0, 0), swerve).withTimeout(2.5),
                            new RunCommand(()-> swerve.drive(0, 0, 0), swerve).withTimeout(2),
                            AutoBalance.Command(swerve, !forward));
  }

  public static CommandBase BalanceAuto(SwerveDrive swerve, boolean forward){
    return AutoBalance.Command(swerve, forward);
  }

  public static CommandBase DriveOverRamp(SwerveDrive swerve, boolean forward){
    return Commands.sequence(new DriveToPitch(swerve, forward), 
                             new RunCommand(()-> swerve.drive((forward ? 0.2 : -0.2), 0, 0), swerve).withTimeout(2.5),
                             new RunCommand(()-> swerve.drive(0, 0, 0), swerve).withTimeout(2));
  }

  private static CommandBase leaveCommunity(SwerveDrive swerve, boolean forward){
    return Commands.sequence(new RunCommand(()-> swerve.drive((forward ? 0.2 : -0.2), 0, 0), swerve).withTimeout(1));
  }

  private static CommandBase scoreLow(){
    return Commands.print("Score in low goal");
  }

  private static CommandBase scoreMiddle(){
    return Commands.print("score in middle goal");
  }
  
  private static CommandBase scoreHigh(){
    return Commands.print("score in high goal");
  }

  public static CommandBase AutoFieldDrive(SwerveDrive swerve, double x, double y, double z){
    return new SwerveDriveCommand(() -> { return x; },
                                  () -> { return y; },
                                  () -> { return z; },
                                  swerve);

  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
