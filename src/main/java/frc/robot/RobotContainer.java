// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveToPitch;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.NewBalance;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(); 
  private final SwerveDrive driveBase = new SwerveDrive();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                       this::getYSpeed, 
                                                       this::getRotationSpeed, driveBase));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem)); 

    driver.start().whileTrue(new InstantCommand( () -> { driveBase.enableFieldOriented(true); })); 
    
    driver.back().whileTrue(new InstantCommand(() -> { driveBase.enableFieldOriented(false);}));

    driver.a().whileTrue(new TurnToAngle(0.0, driveBase)); 

    // driver.y().onTrue(new DriveToPitch(driveBase)); 
    // driver.b().onTrue(new Balance(driveBase)); 
    driver.y().onTrue(new SequentialCommandGroup(new DriveToPitch(driveBase), new WaitCommand(1.925),
    new Balance(driveBase)));
    //driver.b().onTrue(new NewBalance(driveBase));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driver.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.AutoBalance(driveBase);
  }
  double getXSpeed(){ 
    double finalX;
    if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY() * 0.5 * (1.0 + driver.getLeftTriggerAxis());
    
    return -finalX;
  }

  public double getYSpeed(){ 
    double finalY;
    if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX() * 0.5 * (1.0 + driver.getLeftTriggerAxis());
    
    return finalY;
  } 
  
  public double getRotationSpeed(){ 
    double finalRotation;

    // if (Math.abs(driver.getRightX()) <= 0.1)
    //   finalRotation = Math.abs(operator.getRightX()) <= 0.1 ? 0.0 : operator.getRightX() * .5 / (1.0 + operator.getRightTriggerAxis());
    // else
      finalRotation = driver.getRightX() * .5 / (1.0 + driver.getRightTriggerAxis());

      if (Math.abs(finalRotation) < 0.1)
        finalRotation = 0.0;
    
    return finalRotation;
  }

}


