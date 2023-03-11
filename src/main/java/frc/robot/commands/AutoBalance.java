// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.tools.LEDs.MultiFunctionLED;

/** Add your docs here. */
public class AutoBalance {
    protected static final double PITCH_DELTA = 5.0;
    protected static final double PITCH_DROP_DELTA = 0.1;
    protected static final double PITCH_BALANCED = 1.5;

    public static CommandBase Command(SwerveDrive swerve, boolean forward) {
        return  new SequentialCommandGroup( new DriveToPitch(swerve, forward),
                                            new DropTrigger(swerve, () -> { return 0.0;} ))
        .andThen(   new SequentialCommandGroup( new Balance(swerve, () -> { return 0.0; }),
                                                new DropTrigger(swerve, () -> { return 0.0; }),
                                                new BalanceTrigger(swerve, () -> { return 0.0; }) ).repeatedly()
                );
    }

    public static class Balance extends CommandBase {
        private double targetPitch = 0.0; 
        private SwerveDrive swerveDrive;
        private DoubleSupplier speedSupplier;
        private double speed;
        private DoubleSupplier ySpeed;
      
        public Balance(SwerveDrive swerveDrive, DoubleSupplier ySpeed, DoubleSupplier speedSupplier) {
            addRequirements(swerveDrive);
            this.swerveDrive = swerveDrive; 
            this.speedSupplier = speedSupplier;
            this.ySpeed = ySpeed;
        }

        public Balance(SwerveDrive swerveDrive, DoubleSupplier speedSupplier){
            this(swerveDrive, 
                 speedSupplier,
                 new DoubleSupplier() {
                    private double speed = 0.16;
                    public double getAsDouble(){
                    if (speed <= 0.06) {
                        return speed;
                    }
                    speed -= 0.02;
                    return speed;
                    }
                });
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            MultiFunctionLED.setMode(1); // Balance Display
            
            double angle = Math.IEEEremainder(swerveDrive.getGyroAngle(), 360.0) ;
            double pitch = swerveDrive.getPitch();
            
            boolean isFacingForward = (-90.0 < angle) && (angle < 90.0) ? true : false;
            boolean isPitchedForwared = (pitch > 0) ? true : false;

            speed = speedSupplier.getAsDouble();

            if (   (!isFacingForward &&  isPitchedForwared)
                || ( isFacingForward && !isPitchedForwared))
            {
                speed = -speed;
            }
            
            targetPitch = (Math.abs(swerveDrive.getPitch()) / 3.0); 
            if (targetPitch < PITCH_BALANCED / 3.0) {
                targetPitch = PITCH_BALANCED / 3.0;
            }
            swerveDrive.brakeMode();
        }
        
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            swerveDrive.drive(speed, ySpeed.getAsDouble(), 0);
        }
        
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) { 
            swerveDrive.drive(0.0, 0, 0);
        }
        
        // Returns true when the command should end.
        @Override
        public boolean isFinished() { 
            double currentPitch = Math.abs(swerveDrive.getPitch());
            return (currentPitch < targetPitch || currentPitch < PITCH_BALANCED) ;
        
        } 
    }

    public static class DropTrigger extends CommandBase {
        private SwerveDrive swerveDrive;
        private DoubleSupplier ySpeed;
        private double pitch;
        public DropTrigger(SwerveDrive swerveDrive, DoubleSupplier speedSupplier){
            this.swerveDrive = swerveDrive;
            this.ySpeed = speedSupplier;
         }

         @Override
         public void initialize() { 
            pitch = swerveDrive.getPitch();
         }
              
         @Override
         public void execute() {
            swerveDrive.drive(0, ySpeed.getAsDouble(), 0);
         }
         @Override
         public void end(boolean interrupted) { 
             swerveDrive.drive(0.0, 0, 0);
         }
         // Returns true when the command should end.
         @Override
         public boolean isFinished() { 
            double new_pitch = swerveDrive.getPitch(); 
            if ( Math.abs(new_pitch - pitch) < PITCH_DROP_DELTA) {
                return true;
            }
            pitch = new_pitch;
            return false; 
         } 
     }

    public static class BalanceTrigger extends CommandBase {
        private SwerveDrive swerveDrive;
        private DoubleSupplier ySpeed;
        public BalanceTrigger(SwerveDrive swerveDrive, DoubleSupplier speedSupplier){
            this.swerveDrive = swerveDrive;
            this.ySpeed = speedSupplier;
         }
              
         // Returns true when the command should end.
         @Override
         public boolean isFinished() { 
            double pitch = Math.abs(swerveDrive.getPitch());
            return pitch > PITCH_BALANCED;
         } 
         @Override
         public void execute() {
            swerveDrive.drive(0, ySpeed.getAsDouble(), 0);
         }
         @Override
         public void end(boolean interrupted) { 
             swerveDrive.drive(0.0, 0, 0);
         }
    }
}
