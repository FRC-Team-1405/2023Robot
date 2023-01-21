// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;

/** Add your docs here. */
public class MagicMotionHelper {
    private IMotorController motor;
    private double threshold;
    private double pos;
    private int settleCount;
    private int settleThreshold;
    public MagicMotionHelper(IMotorController motor, double threshold, int settle){
        this.motor=motor;
        this.threshold=threshold;
        this.settleThreshold=settle;
    }
    public void setPosition(double pos){
        this.pos=pos;
        settleCount=0;
        motor.set(ControlMode.MotionMagic, pos);
    }
    public boolean atPosition(){
        if ( (Math.abs(motor.getActiveTrajectoryPosition() - pos) < threshold) 
            && (Math.abs(motor.getClosedLoopError(0)) < threshold) ){
            settleCount+=1; 
        } else {
            settleCount=0;
        }

        return settleCount >= settleThreshold;
    }

}
