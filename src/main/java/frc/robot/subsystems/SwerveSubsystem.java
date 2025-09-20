import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase{

  double maximumSpeed = Units.feetToMeters(4.5)
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);

}