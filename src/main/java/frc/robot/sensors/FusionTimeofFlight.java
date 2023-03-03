// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class FusionTimeofFlight {

    public TimeOfFlight lidar; 
    
    public FusionTimeofFlight(int sensorID){ 
        lidar = new TimeOfFlight(sensorID);
    }

    public void measure(){ 
        SmartDashboard.putNumber("lidar distance", lidar.getRange()); 
    }

}
