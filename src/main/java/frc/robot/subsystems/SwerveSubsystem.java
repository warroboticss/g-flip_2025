package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase{

  private SwerveDrive swerveDrive;
  double maximumSpeed = Units.feetToMeters(4.5);
  double maximumAngularSpeed = Units.degreesToRadians(720);

  public SwerveSubsystem(){
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
    swerveDrive.setHeadingCorrection(false); 
    swerveDrive.setCosineCompensator(false);
    //try perhaps: swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1); 

    setupPathPlanner();
  }

  double getMaximumVelocity(){
    return maximumSpeed;
  }

  double getMaximumAngularVelocity(){
    return maximumAngularSpeed;
  }

    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }


  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }
 public void setupPathPlanner()
  {
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(// PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(0.01, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.01, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }
    public Command getAutonomousCommand(String pathName)
  {
    return new PathPlannerAuto(pathName);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {swerveDrive.drive(new Translation2d(translationX.getAsDouble() * getMaximumVelocity(), translationY.getAsDouble() * getMaximumVelocity()), angularRotationX.getAsDouble() * getMaximumAngularVelocity(), true, false);
    });
  }


  //limelight stuff
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerveDrive.drive(translation.rotateBy(Rotation2d.fromDegrees(-90)), rotation, fieldRelative, isOpenLoop);
  }
  public ChassisSpeeds getTargetSpeeds(double x, double y, Rotation2d rotation) {
      return swerveDrive.swerveController.getTargetSpeeds(x, y, rotation.getRadians(),
              (getYaw()).getRadians() , maximumSpeed);

  }
}

