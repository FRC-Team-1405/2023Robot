// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Trajectories { 
    
    public Trajectory testTrajectory() { 

    var startPosition = new Pose2d(0, 0, new Rotation2d(0) );
    var endPosition = new Pose2d(Units.feetToMeters(3.0), 0, new Rotation2d(0) );

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(1), Units.feetToMeters(0)));
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(2), Units.feetToMeters(0)));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(false);

    var testTrajectory = TrajectoryGenerator.generateTrajectory(
        startPosition,
        interiorWaypoints,
        endPosition,
        config); 

        return testTrajectory; 
  }
}
