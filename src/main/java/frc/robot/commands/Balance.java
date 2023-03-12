package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class Balance extends CommandBase {
  /** Creates a new DriveToPitch. */ 
  private boolean forward;

  private double pitch = 0.0; 
  private SwerveDrive swerveDrive;
  private double speed;

  public Balance(SwerveDrive swerveDrive, double speed) {
    // Use addRequirements() here to declare subsystem dependencies. 
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive; 
    this.speed = speed;
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    if(swerveDrive.getPitch() > 0){
      forward = true;
    }
    else if(swerveDrive.getPitch() < 0){
      forward = false; 
    }
    pitch = (Math.abs(swerveDrive.getPitch()) - 1); 
    swerveDrive.brakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive((forward ? speed : -speed), 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    swerveDrive.drive(0.0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 

    return Math.abs(swerveDrive.getPitch()) < pitch; 
  
  } 
}