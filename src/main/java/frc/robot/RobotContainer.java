package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.LimelightCenter;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  //private final Joystick controller = new Joystick(0);
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick revControl = new Joystick(1);
  private final
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-a");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public RobotContainer() {


    configureBindings();
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem, () -> controller.getLeftX() , () -> controller.getLeftY(), () -> controller.getRightX()));
  }

 
  private void configureBindings() {
    controller.y().onTrue((new InstantCommand(driveSubsystem::zeroGyro)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return driveSubsystem.getAutonomousCommand("Just Shoot");
    //return new ShootSpeakerCommand(shooterSubsystem);
  }
}
