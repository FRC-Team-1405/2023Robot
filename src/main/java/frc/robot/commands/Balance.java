package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class Balance extends CommandBase {
  /** Creates a new DriveToPitch. */ 
  private static double pitch_P = 2.5;
  private static double pitch_I = 0.0;
  private static double pitch_D = 0.75 ;
  
  private double pitch = 0.0; 
  private SwerveDrive swerveDrive;
 // private ProfiledPIDController pitchController;
 // private SwerveSubsystem swerve;
      //loadConfigs();

  public Balance(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies. 
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive; 

  //   pitchController.setGoal(Units.degreesToRadians(angle));
         //this.swerve = swerve;
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    pitch = (swerveDrive.getPitch() - 0.0125); 
    swerveDrive.brakeMode();
    //pitchController.reset(Units.degreesToRadians(-swerve.getPitch())); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = Units.degreesToRadians(-swerveDrive.getPitch());
        //double speed = pitchController.calculate( pitch );
        //setPosition(speed); 
        swerveDrive.driveSpeed(0.4, 0, 0, true);
        //SmartDashboard.putNumber("Balance Error", pitchController.getPositionError()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    System.out.println("stop");
    swerveDrive.driveSpeed(0.0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return swerveDrive.getPitch() < pitch; 
   // return false; 
    //return pitchController.atGoal();
  } 

  private void setPosition(double speed) {
    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
    // SwerveModuleState[] moduleStates = swerve.getKinematics().toSwerveModuleStates(chassisSpeeds) ;
    // swerve.setModuleStates(moduleStates); 
    swerveDrive.driveSpeed(0.4, 0, 0, true);
} 
  private void configPIDs(SwerveSubsystem swerve){
    //pitchController =  new ProfiledPIDController(pitch_P, pitch_I, pitch_D, new TrapezoidProfile.Constraints(0.005, 1));
    //pitchController.enableContinuousInput(-Math.PI, Math.PI); 
    //pitchController.setTolerance((Units.degreesToRadians(5)));}
}}