// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class FusionTimeofFlight {

    private TimeOfFlight lidar; 
    private MedianFilter filter = new MedianFilter(5);
    
    public FusionTimeofFlight(int sensorID){ 
        lidar = new TimeOfFlight(sensorID);
        lidar.setRangingMode(RangingMode.Medium, 20);
    }

    public double measure(){
        double value = filter.calculate(lidar.getRange()); 
        SmartDashboard.putNumber("lidar distance", value); 
        return value;
    }

}
