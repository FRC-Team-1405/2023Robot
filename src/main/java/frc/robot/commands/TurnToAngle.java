package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToAngle extends CommandBase{

    private static double z_P = 10.0 ;
    private static double z_I = 0.0 ;
    private static double z_D = 0.0 ;
    static{
        loadConfigs();
    }
    private ProfiledPIDController zController;
    private SwerveSubsystem swerve;

    public TurnToAngle(double angle, SwerveSubsystem swerve) {
        addRequirements(swerve);
        
        configPIDs(swerve);
        zController.setGoal(Units.degreesToRadians(angle));
        this.swerve = swerve;
    }
    
    public void initialize() {
        zController.reset(swerve.getPose().getRotation().getRadians());
    }

    public void execute() {
        double angle = swerve.getPose().getRotation().getRadians();
        double speed = zController.calculate( angle );
        setRotation(speed);
    }

    public boolean isFinished() {
        return zController.atGoal();
      }    

    public void end(boolean interrupted) {
        setRotation(0.0);
    }

    private void setRotation(double speed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, speed);
        SwerveModuleState[] moduleStates = swerve.getKinematics().toSwerveModuleStates(chassisSpeeds) ;
        swerve.setModuleStates( moduleStates);
    }

    private void configPIDs(SwerveSubsystem swerve){
        zController =  new ProfiledPIDController(z_P, z_I, z_D, new TrapezoidProfile.Constraints(swerve.getMaxAngularSpeed(),swerve.getMaxAngularAcceleration()));
        zController.enableContinuousInput(-Math.PI, Math.PI); 
        zController.setTolerance((Math.PI*2)/100.0);
    }

    private static void loadConfigs(){
        Preferences.initDouble("TurnToAngle/Z/P", z_P);
        Preferences.initDouble("TurnToAngle/Z/I", z_I);
        Preferences.initDouble("TurnToAngle/Z/D", z_D);
        z_P = Preferences.getDouble("TurnToAngle/Z/P", z_P);
        z_I = Preferences.getDouble("TurnToAngle/Z/I", z_I);
        z_D = Preferences.getDouble("TurnToAngle/Z/D", z_D);
    }

}
