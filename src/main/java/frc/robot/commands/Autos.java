// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.AutoDrive;

import java.security.AuthProvider;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


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
    Long_Exit,
    Short_Exit,
    Drive_Over_Ramp, 
    U_Turn,
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
    step_2.addOption("Short Exit", AutoCommand.Short_Exit);
    step_2.addOption("Long Exit", AutoCommand.Long_Exit);
    step_2.addOption("Do nothing!", AutoCommand.Do_Nothing); 
    step_2.addOption("U-Turn", AutoCommand.U_Turn); 
    SmartDashboard.putData("Auto/Step 2", step_2);

    step_3.setDefaultOption("Backwards Balance", AutoCommand.Balance_Backwards);
    step_3.addOption("Forwards Balance", AutoCommand.Balance_Forward);
    step_3.addOption("Do Nothing!", AutoCommand.Do_Nothing);
    SmartDashboard.putData("Auto/Step 3", step_3);
  }

  public static CommandBase getSelectedAuto(SwerveDrive swerve, Arm arm, Intake intake){
    Position pos = position.getSelected();
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();

    Command cmd_1;
    switch (step_1.getSelected()){
      case Score_High:    cmd_1 = scoreHigh(arm, swerve, intake) ;    
                          break;
      case Score_Middle:  cmd_1 = scoreMiddle(arm, swerve, intake) ;  
                          break;
      case Score_Low:     cmd_1 = scoreLow(intake)  ;     
                          break;
      default:            cmd_1 = Commands.print("Skipping Step 1") ; 
                          break;
      }

      Command cmd_2;
      switch (step_2.getSelected()){
        case Drive_Over_Ramp:   cmd_2 = DriveOverRamp(swerve, false) ;    
                                break;
        case Balance_Backwards: cmd_2 = BalanceAuto(swerve, false)  ;  
                                break;
        case Short_Exit:   cmd_2 = shortExit(swerve, false);
                                break;
        case Long_Exit:   cmd_2 = longExit(swerve);
                                break; 
        case U_Turn:      cmd_2 = uTurnRamp(swerve, false); 
                                break; 
        default:                cmd_2 = Commands.print("Skipping Step 2") ; 
                                break;
      }

      Command cmd_3;
      switch (step_3.getSelected()){
        case Balance_Backwards: cmd_3 = BalanceAuto(swerve, false) ;
                                break;
        case Balance_Forward:   cmd_3 = BalanceAuto(swerve, true);
                                break;                        
        default:                cmd_3 = Commands.print("Skipping Step 3") ;
                                break;
      }
  
      autoCommand.addCommands( Commands.print("Auto command starting"),
                               Commands.runOnce(swerve::resetGyro),
                               Commands.waitSeconds(1),
                               cmd_1,
                               cmd_2,
                               cmd_3,
                               Commands.print("Auto command finished") );
 
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
                             new AutoDrive(swerve, 0.5, 0, Units.inchesToMeters(78)),
                             swerve.runOnce( () -> { swerve.drive(0, 0, 0); } )
                            );
  }

  private static CommandBase shortExit(SwerveDrive swerve, boolean forward){
    return Commands.sequence(new RunCommand(()-> swerve.drive((forward ? 0.2 : -0.2), 0, 0), swerve).withTimeout(1));
  }

  private static CommandBase longExit(SwerveDrive swerve){
    return Commands.sequence(new AutoDrive(swerve, 0.5, 0.0, Units.inchesToMeters(180.0)), 
    new TurnToAngle(180, swerve)); 

  }

  private static CommandBase scoreLow(Intake intake){
    return new SequentialCommandGroup(
                intake.runOnce( intake:: gateLower),
                intake.run( intake::conveyerBeltEject).withTimeout(1),
                intake.runOnce( intake::conveyerBeltOff),
                intake.runOnce( intake::gateRaise)
    );
  }

  private static CommandBase scoreMiddle(Arm arm, SwerveDrive swerve, Intake intake){
    return new SequentialCommandGroup(
      intake.runOnce( intake::gateLower ),
      new ScoreConeCommand(arm, Arm.Position.ConeMiddle),
      new AutoDrive( swerve, -.2, 0, Units.inchesToMeters(4)),
      new InstantCommand(()->{arm.openClaw();}, arm),
      new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Home);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
      new FunctionalCommand( () -> { arm.setElbowPosition(Arm.Position.Home);}, () -> {}, interupted -> {}, arm::atElbowPosition, arm),
      intake.runOnce( intake::gateRaise )
    );
  }
  
  private static CommandBase scoreHigh(Arm arm, SwerveDrive swerve, Intake intake){
    return new SequentialCommandGroup(
      intake.runOnce( intake::gateLower),
      new ScoreConeCommand(arm, Arm.Position.ConeHigh), 
      new AutoDrive(swerve, -.2, 0, Units.inchesToMeters(6)),
      new InstantCommand(()->{arm.openClaw();}, arm),
      new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Home);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
      new FunctionalCommand( () -> { arm.setElbowPosition(Arm.Position.Home);}, () -> {}, interupted -> {}, arm::atElbowPosition, arm),
      intake.runOnce( intake::gateRaise )
    );
  }

  public static CommandBase AutoFieldDrive(SwerveDrive swerve, double x, double y, double z){
    return new SwerveDriveCommand(() -> { return x; },
                                  () -> { return y; },
                                  () -> { return z; },
                                  swerve);

  } 

  public static CommandBase uTurnRamp(SwerveDrive swerve, boolean forward){ 
    return Commands.sequence(new AutoDrive(swerve, 0.5, 0, Units.feetToMeters(16)), 
    new AutoDrive(swerve, 0, 0.5, Units.feetToMeters(9.0)));
  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
