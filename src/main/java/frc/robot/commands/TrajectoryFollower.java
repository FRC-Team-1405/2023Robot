// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.tools.Trajectories;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;

public class TrajectoryFollower extends SequentialCommandGroup {
  /** Creates a new RamseteController. */ 
  public Trajectories trajectories; 
  public SwerveDrive driveBase; 
  public SwerveControllerCommand swerveControllerCommand;

  private PIDController xController = new PIDController(0, 0, 0);
  private PIDController yController = new PIDController(0, 0, 0);
  private ProfiledPIDController zController = new ProfiledPIDController(0, 0, 0, new Constraints(Math.PI, 2*Math.PI)); 

  public TrajectoryFollower(Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
     swerveControllerCommand =  new SwerveControllerCommand(trajectory,  driveBase::getPose, driveBase.getKinematics(),
     xController,
     yController, 
     zController,  
     driveBase::setModuleStates, 
     driveBase);
  addCommands(swerveControllerCommand);
  }

}
