// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Autos;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.LEDManager;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.VisionAlignment;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystems.SwerveModule;
import frc.robot.tools.DigitalToggle;
import frc.robot.tools.MathTools;
import frc.robot.tools.SwerveType;
import edu.wpi.first.math.util.Units;
import frc.robot.tools.LEDs.BalanceLED;
import frc.robot.tools.LEDs.BatteryLED;
import frc.robot.tools.LEDs.BounceDisplay;
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
  private final PowerDistribution powerDistribution = new PowerDistribution();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.OperatorControllerPort);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    BadLog.createTopic("PDU/Total Current", "A", () -> powerDistribution.getTotalCurrent(), "hide", "join:PDU/Totals");
    BadLog.createTopic("PDU/Total Energy", "J", () -> powerDistribution.getTotalEnergy(), "hide", "join:PDU/Totals");
    BadLog.createTopic("PDU/Total Power", "W", () -> powerDistribution.getTotalPower(), "hide", "join:PDU/Totals");
    
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
                        new BalanceLED(Constants.BatteryMonitor.LEDCOUNT, () -> { return driveBase.getPitch(); }),
                        new BounceDisplay(Constants.BatteryMonitor.LEDCOUNT)
                        );
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
  private enum InputType { Cube, Cone };
  private InputType inputType = InputType.Cone;
  public void setInputType(InputType type) {
    inputType = type;
    SmartDashboard.putBoolean("IntakeType/Cone", inputType == InputType.Cone);
    SmartDashboard.putBoolean("IntakeType/Cube", inputType == InputType.Cube);
  }

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
    operator.start().onTrue( Commands.runOnce( () -> { System.out.println("Emergency Cancel"); }, arm, driveBase, intake));


    operator.leftBumper().whileTrue( Commands.run(() -> { arm.adjustElbowPosition( (int)(operator.getLeftY() * 1250));}, arm) );
    operator.rightBumper().whileTrue(Commands.run(() -> { arm.adjustExtensionPosition((int)(operator.getRightY() * 1250));}, arm));
    operator.rightTrigger().onTrue(Commands.run(() -> {intake.gateLower();}, intake));
    operator.rightTrigger().onFalse(Commands.run(() -> {intake.gateRaise();}, intake));
    operator.povUp().onTrue( 
        new SequentialCommandGroup( 
          new InstantCommand(() -> { arm.openClaw();}),
          new ScoreCommand(arm, Arm.Position.FeederStation)
          ))
        .onFalse( new InstantCommand(() -> { arm.closedClaw();}) );
    operator.povDown().onTrue( 
      new SequentialCommandGroup(
          new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Home);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
          new FunctionalCommand( () -> { arm.setElbowPosition(Arm.Position.Home);}, () -> {}, interupted -> {}, arm::atElbowPosition, arm)
        )
    );


    
    Limelight limelight = new Limelight();
    operator.povDown().whileTrue(Commands.runOnce(() ->{
      byte pipeline = (byte) MathTools.clamp(VisionAlignment.getVisionPipelien() -1, Constants.Limelight.Pipeline_MIN, Constants.Limelight.Pipeline_MAX);
      VisionAlignment.setVisionPipeline(pipeline);
    }));
    operator.povUp().whileTrue(Commands.runOnce(() -> {
      byte pipeline = (byte) MathTools.clamp(VisionAlignment.getVisionPipelien() +1, Constants.Limelight.Pipeline_MIN, Constants.Limelight.Pipeline_MAX);
      VisionAlignment.setVisionPipeline(pipeline);
    }));
    CommandBase visionAlignment = new VisionAlignment(this::getXSpeed, 0, driveBase); 

    setInputType(inputType);
    driver.x().onTrue( new InstantCommand(() -> { setInputType(InputType.Cube);}));
    driver.y().onTrue( new InstantCommand(() -> { setInputType(InputType.Cone);}));

    driver.a().whileTrue( new ParallelCommandGroup( visionAlignment, scoreCommand ));
    driver.b().whileTrue( new SequentialCommandGroup( 
                              new AutoBalance.Balance(driveBase, this::getYSpeed),
                              new AutoBalance.DropTrigger(driveBase, this::getYSpeed),
                              new AutoBalance.BalanceTrigger(driveBase, this::getYSpeed) ).repeatedly()
                          );
    driver.start().whileTrue(new InstantCommand( () -> { driveBase.enableFieldOriented(true); }));
    driver.back().whileTrue(new InstantCommand(() -> { driveBase.enableFieldOriented(false);}));
                      
    driver.rightBumper()
      .onTrue(
        Commands.parallel(
          Commands.runOnce( ()-> {
            intake.intakeDeploy();
            intake.intakeSuck();
            if (inputType == InputType.Cone) { intake.conveyerBeltForward(); }
            intake.twisterForward();},
            intake),
          Commands.sequence(
            new InstantCommand(arm::openClaw),
            new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Home);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
            new FunctionalCommand( () -> { arm.setElbowPosition(Arm.Position.Home);}, () -> {}, interupted -> {}, arm::atElbowPosition, arm)
      )))
      .onFalse(
        Commands.parallel(
          Commands.runOnce( ()-> {
            intake.intakeRetract();
            intake.intakeOff();
            intake.conveyerBeltOff();
            intake.twisterOff();},
            intake),
          Commands.sequence(
            new FunctionalCommand( () -> { arm.setExtensionPosition(Arm.Position.Grab);}, () -> {}, intrupted -> {}, arm::atExtensionPosition, arm),
            new InstantCommand(arm::closedClaw)
      )));

    driver.leftBumper().onTrue( 
      new SequentialCommandGroup(
//          new AutoDrive(driveBase, 0.1, 0.0, Units.inchesToMeters(6)),
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
    SmartDashboard.putData("ConveyerBelt/Forward", intake.run( intake::conveyerBeltForward));
    SmartDashboard.putData("ConveyerBelt/Off", intake.run( intake::conveyerBeltOff));
    SmartDashboard.putData("Intake/On", intake.run( intake::intakeSuck));
    SmartDashboard.putData("Intake/Off", intake.run( intake::intakeOff));
    SmartDashboard.putData("Gate/Raise", intake.run( intake::gateRaise ));
    SmartDashboard.putData("Gate/Lower", intake.run( intake::gateLower));

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


