// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class AutoBalance {
    protected static final double PITCH_DELTA = 1.0;
    protected static final double PITCH_DROP_DELTA = 0.1;
    protected static final double PITCH_BALANCED = 1.5;
    
    public static CommandBase Command(SwerveDrive swerve, boolean forward) {
        DoubleSupplier speedSupplier = new DoubleSupplier() {
            private double speed = 0.12;
            public double getAsDouble(){
              if (speed <= 0.04) {
                return speed;
              }
              speed -= 0.02;
              return speed;
            }
          };
        return  new SequentialCommandGroup( new DriveToPitch(swerve, forward),
                                            new DropTrigger(swerve))
        .andThen(   new SequentialCommandGroup( new Balance(swerve, speedSupplier),
                                                new DropTrigger(swerve),
                                                new BalanceTrigger(swerve) ).repeatedly()
                );
    }

    private static class Balance extends CommandBase {
        private boolean forward;

        private double pitch = 0.0; 
        private SwerveDrive swerveDrive;
        private DoubleSupplier speedSupplier;
        private double speed;
      
        public Balance(SwerveDrive swerveDrive, DoubleSupplier speedSupplier) {
            addRequirements(swerveDrive);
            this.swerveDrive = swerveDrive; 
            this.speedSupplier = speedSupplier;
        }
    
        // Called when the command is initially scheduled.
        @Override
        public void initialize() { 
            speed = speedSupplier.getAsDouble();
            if(swerveDrive.getPitch() > 0){
            forward = true;
            }
            else if(swerveDrive.getPitch() < 0){
            forward = false; 
            }
            pitch = (Math.abs(swerveDrive.getPitch()) - PITCH_DELTA); 
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
            System.out.println("stop");
            swerveDrive.drive(0.0, 0, 0);
        }
        
        // Returns true when the command should end.
        @Override
        public boolean isFinished() { 
        
            return Math.abs(swerveDrive.getPitch()) < pitch; 
        
        } 
    }

    private static class DropTrigger extends CommandBase {
        private SwerveDrive swerveDrive;
        private double pitch;
        public DropTrigger(SwerveDrive swerveDrive){
            this.swerveDrive = swerveDrive;
         }

         @Override
         public void initialize() { 
            pitch = swerveDrive.getPitch();
         }
              
         // Returns true when the command should end.
         @Override
         public boolean isFinished() { 
            double new_pitch = swerveDrive.getPitch(); 
            if (Math.abs(new_pitch - pitch) < PITCH_DROP_DELTA) {
                return true;
            }
            pitch = new_pitch;
            return false; 
         } 
    }

    private static class BalanceTrigger extends CommandBase {
        private SwerveDrive swerveDrive;
        public BalanceTrigger(SwerveDrive swerveDrive){
            this.swerveDrive = swerveDrive;
         }
              
         // Returns true when the command should end.
         @Override
         public boolean isFinished() { 
            return Math.abs(swerveDrive.getPitch()) > PITCH_BALANCED;
         } 
    }
}
