package frc.robot.commands;

// import com.team2357.frc2023.controls.SwerveDriveControls;
// import com.team2357.frc2023.state.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import swervelib.SwerveController;


public class DefaultDriveCommand extends Command {
    private DriveSubsystem m_swerve;
    private Supplier<Double> x, y, rotation;
    private Supplier<Boolean> slow;


    public DefaultDriveCommand(DriveSubsystem swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rotation) {
        this.m_swerve = swerve;
        this.x = x;
        this.y = y;
        this.rotation = rotation;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double current_x = x.get();
        double current_y = y.get() * -1;
        double current_rotation = rotation.get() * -0.5;
        // if(slow.get()){
        //     current_x *= 0.5;
        //     current_y *= 0.5;
        // }
        ChassisSpeeds targetSpeeds = m_swerve.getTargetSpeeds(current_x, current_y,new Rotation2d(current_rotation * Math.PI));


// 4 below is meant to be MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND wtv that means figure it out probably !!!!
        
        m_swerve.drive(SwerveController.getTranslation2d(targetSpeeds), current_rotation * 4, true, false);
        // m_swerve.driveCommand(current_x, current_y);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new Translation2d(), 0, true, false);
    }
}
