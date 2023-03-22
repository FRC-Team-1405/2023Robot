// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

/** Add your docs here. */
public class MagicMotionHelper {
    private IMotorController motor;
    private double threshold;
    private double targetPos;
    private double startPos;
    private int settleCount;
    private int settleThreshold;
    public MagicMotionHelper(IMotorController motor, double threshold, int settle){
        this.motor=motor;
        this.threshold=threshold;
        this.settleThreshold=settle;
    }

    public MagicMotionHelper(BaseMotorController motor, int settle){
        SlotConfiguration slot = new SlotConfiguration();
        motor.getSlotConfigs(slot, 0, settle);
        this.threshold = slot.allowableClosedloopError * 4;

        this.motor=motor;
        this.settleThreshold=settle;
    }

    public void setPosition(double pos){
        this.targetPos=pos;
        settleCount=0;
        motor.set(ControlMode.MotionMagic, pos);
        startPos = motor.getSelectedSensorPosition(0);
    }

    public double getPosition(){
        return targetPos;
    }

    public boolean atPosition(){
        if ( (Math.abs(motor.getActiveTrajectoryPosition() - targetPos) < threshold) 
            && (Math.abs(motor.getClosedLoopError(0)) < threshold) ){
            settleCount+=1; 
        } else {
            settleCount=0;
        }

        return settleCount >= settleThreshold;
    }

    public double getProgress(){
        double pos = motor.getSelectedSensorPosition(0);
        return Math.abs((targetPos - pos) / (targetPos - startPos));
    }
    
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

}
