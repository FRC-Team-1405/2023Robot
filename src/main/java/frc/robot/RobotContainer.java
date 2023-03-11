// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Autos;
import frc.robot.commands.BackUp;
import frc.robot.commands.LEDManager;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.VisionAlignment;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.tools.SwerveType;
import frc.robot.tools.LEDs.BalanceLED;
import frc.robot.tools.LEDs.BatteryLED;
import frc.robot.tools.LEDs.IAddressableLEDHelper;
import frc.robot.tools.LEDs.MultiFunctionLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here... 
  private final SwerveDrive driveBase = new SwerveDrive();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.OperatorControllerPort);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    ConfigShuffleboard();
    Autos.initAutoSelector();

    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                       this::getYSpeed, 
                                                       this::getRotationSpeed, driveBase));
  }

  public void disabledInit() {
    arm.onDisable();
    intake.onDisable();
  }

  private IAddressableLEDHelper[] leds;
  private MultiFunctionLED multifucntion ;
  private LEDManager ledManager;

  public void configureLEDs(){
    //ledManager can run during disabled
    multifucntion = new MultiFunctionLED( 
                        new BatteryLED(Constants.BatteryMonitor.LEDCOUNT), 
                        new BalanceLED(Constants.BatteryMonitor.LEDCOUNT, () -> { return driveBase.getPitch(); }) );
    leds = new IAddressableLEDHelper[] {multifucntion};
    

    ledManager = new LEDManager(Constants.PWMPort.LEDPORT, leds);
    ledManager.schedule();
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
    ScoreCommand scoreCommand = new ScoreCommand(arm);
    operator.y().onTrue(scoreCommand.setHighPostition);
    operator.b().onTrue(scoreCommand.setMiddlePosition);
    operator.a().onTrue(scoreCommand.setLowPostition);
    operator.x().onTrue(scoreCommand.setCustomPosition);

    operator.back().onTrue( new InstantCommand( driveBase::resetGyro ) {
      public boolean runsWhenDisabled() {
        return true;
      }    
    });


    operator.leftBumper().whileTrue( Commands.run(() -> { arm.adjustElbowPosition( (int)(operator.getLeftY() * 1250));}, arm) );
    operator.rightBumper().whileTrue(Commands.run(() -> { arm.adjustExtensionPosition((int)(operator.getRightY() * 1250));}, arm));
    
    CommandBase visionAlignment = new VisionAlignment(this::getXSpeed, 0, driveBase); 
    driver.x().onTrue(new BackUp(driveBase));
    driver.a().whileTrue( new ParallelCommandGroup( visionAlignment, scoreCommand ));
    driver.b().whileTrue( new SequentialCommandGroup( 
                              new AutoBalance.Balance(driveBase, this::getYSpeed),
                              new AutoBalance.DropTrigger(driveBase, this::getYSpeed),
                              new AutoBalance.BalanceTrigger(driveBase, this::getYSpeed) ).repeatedly()
                          );
    driver.start().whileTrue(new InstantCommand( () -> { driveBase.enableFieldOriented(true); }));
    driver.back().whileTrue(new InstantCommand(() -> { driveBase.enableFieldOriented(false);}));
                      
    driver.rightBumper()
      .onTrue( new ConditionalCommand(
                  Commands.parallel(
                    Commands.run( ()-> {
                      intake.intakeRetract();
                      intake.intakeOff();
                      intake.conveyerBeltOff();
                      intake.twisterOff();},
                      intake),
                    Commands.sequence(
                      new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Grab);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
                      new InstantCommand(arm::closedClaw)
                    )),
                    Commands.parallel(
                      Commands.run( ()-> {
                        intake.intakeDeploy();
                        intake.intakeSuck();
                        intake.conveyerBeltForward();
                        intake.twisterForward();},
                        intake),
                      Commands.sequence(
                        new InstantCommand(arm::openClaw),
                        new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Home);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
                        new FunctionalCommand( () -> { arm.setElbowPosition(Arm.Position.Home);}, () -> {}, interupted -> {}, arm::atElbowPosition, arm)
                      )
                    ),
                    intake::intakeIsDeployed)
      );

    driver.leftBumper().onTrue( 
      new SequentialCommandGroup(
          new InstantCommand(arm::openClaw),
          new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Home);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
          new FunctionalCommand( () -> { arm.setElbowPosition(Arm.Position.Home);}, () -> {}, interupted -> {}, arm::atElbowPosition, arm)
        )
    );
    driver.x().whileTrue( Commands.startEnd( () -> { driveBase.parkingBrake(true);},
                                             () -> { driveBase.parkingBrake(false);}));

  }
  
  private void ConfigShuffleboard(){
    SmartDashboard.putNumber("Swerve/X", 0.0);
    SmartDashboard.putNumber("Swerve/Y", 0.0);
    SmartDashboard.putNumber("Swerve/Z", 0.0);

    SwerveDriveCommand swerveDrive = new SwerveDriveCommand(() -> { return SmartDashboard.getNumber("Swerve/X", 0.0); },
                                                            () -> { return SmartDashboard.getNumber("Swerve/Y", 0.0); },
                                                            () -> { return SmartDashboard.getNumber("Swerve/Z", 0.0); },
                                                            driveBase);

    SmartDashboard.putData("Swerve/Start", swerveDrive);

    SmartDashboard.putData("Intake/Deploy", intake.run( intake::intakeDeploy ));
    SmartDashboard.putData("Intake/Retract", intake.run( intake::intakeRetract ));
    SmartDashboard.putData("Claw/Open", arm.run( arm::openClaw ));
    SmartDashboard.putData("Claw/Close", arm.run( arm::closedClaw ));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  return Autos.BalanceAuto(driveBase, false);
  }
  double getXSpeed(){ 
    double finalX;
    if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY() * 0.75 * (1.0 + driver.getLeftTriggerAxis());
    
    return finalX;
  }

  public double getYSpeed(){ 
    int pov = driver.getHID().getPOV();

    double finalY;
    if ( pov == 270 || pov == 315 || pov == 225)
      finalY = -0.05;
    else if(pov == 90 || pov == 45 || pov == 135)
      finalY = 0.05;
    else if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX() * 0.75 * (1.0 + driver.getLeftTriggerAxis());
    
    if (SwerveType.isStandard())
      finalY = -finalY;
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


