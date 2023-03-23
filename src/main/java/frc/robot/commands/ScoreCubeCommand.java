
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.cert.Extension;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;

/** Add your docs here. */
public class ScoreCubeCommand extends SequentialCommandGroup{


    public CommandBase setCubeHighPostition = new InstantCommand( () -> { setTarget(Arm.Position.CubeHigh);} );
    public CommandBase setCubeMiddlePosition = new InstantCommand( () -> { setTarget(Arm.Position.CubeMiddle);});
    public CommandBase setLowPostition = new InstantCommand( () -> { setTarget(Arm.Position.Low);} );
    public CommandBase setCustomPosition = new InstantCommand( () -> { setTarget(Arm.Position.Custom);});

    private void setTarget(Arm.Position position) {
        this.position = position;
        SmartDashboard.putBoolean("Score/Position/High", position == Arm.Position.CubeHigh);
        SmartDashboard.putBoolean("Score/Position/Middle", position == Arm.Position.CubeMiddle);
        SmartDashboard.putBoolean("Score/Position/Low", position == Arm.Position.Low);
        SmartDashboard.putBoolean("Score/Position/Custom", position == Arm.Position.Custom);
    }


    private Arm arm;
    protected Arm.Position position = Position.Home;
    public Arm.Position getTarget() {
        return position;
    }

    public ScoreCubeCommand(Arm arm){
        this.arm = arm;
        addRequirements(arm);

        addCommands( new InstantCommand(() -> { arm.closedClaw(); }),
                     new ParallelCommandGroup(
                        new ArmAngle(this.arm, () -> { return this.position;}, false ),
                        new ArmExtension(this.arm, () -> { return this.position;}, true ) 
                     ));    
    }
    
    public ScoreCubeCommand(Arm arm, Arm.Position position){ 
        this.arm = arm;
        addRequirements(arm);

        
        setTarget(position);

        addCommands( new InstantCommand(() -> { arm.closedClaw(); }),
                     new ParallelCommandGroup(
                        new ArmAngle(this.arm, () -> { return this.position;}, false ),
                        new ArmExtension(this.arm, () -> { return this.position;}, true ) 
                     ));    
    }

    static public CommandBase ArmHomeCommand(Arm arm){
        return new ParallelCommandGroup(
            new ScoreCubeCommand.ArmExtension(arm, () -> { return Position.Home;}, false ),
            new ScoreCubeCommand.ArmAngle(arm, () -> { return Position.Home;}, true )
         );    
    }
    private static class ArmAngle extends CommandBase{
        static {
            Preferences.initDouble("ArmAngle Command", 0.25);
            delayPercent = Preferences.getDouble("ArmAngle Command", 0.25);
        }

        private Arm arm;
        private Supplier<Position> position;
        private boolean waiting = true;
        private static double delayPercent = 0.25;
        private boolean delayStart;
        public ArmAngle(Arm arm, Supplier<Position> position, boolean delayStart){
            this.arm = arm;
            this.position = position;
            this.delayStart = delayStart;
        }
    
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            if (!delayStart){
                arm.setElbowPosition(position.get());
                waiting = false;
            }
        }

        @Override
        public void execute(){
            if (waiting && arm.elbowPositionProgress() > delayPercent){
                arm.setElbowPosition(position.get());
                waiting = false;
            }
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
        return arm.atElbowPosition();
        }
    }

    private static class ArmExtension extends CommandBase{
        static {
            Preferences.initDouble("ArmExtension Command", 0.25);
            delayPercent = Preferences.getDouble("ArmExtension Command", 0.25);
        }

        private Arm arm;
        private Supplier<Position> position;
        private boolean waiting = true;
        private static double delayPercent = 0.25;
        private boolean delayStart;
        public ArmExtension(Arm arm, Supplier<Position> position, boolean delayStart){
            this.arm = arm;
            this.position = position;
            this.delayStart = delayStart;
        }
            
        @Override
        public void initialize() {
            if (!delayStart){
                arm.setElbowPosition(position.get());
                waiting = false;
            }
        }

        @Override
        public void execute(){
            if (waiting && arm.extensionPositionProgress() > delayPercent){
                arm.setExtensionPosition(position.get());
                waiting = false;
            }
        }
    
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
        return arm.atExtensionPosition();
        }
    }

}