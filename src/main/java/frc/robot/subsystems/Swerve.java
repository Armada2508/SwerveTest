package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveK;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

    SwerveDrive swerveDrive;
    double maximumSpeed = Units.feetToMeters(4.5);

    /**
     * 
     * @param directory Directory of swerve drive config files.
     */
    public Swerve(File directory) throws IOException {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(SwerveK.steerGearRatio);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(SwerveK.driveGearRatio, SwerveK.wheelDiameter.in(Inches));

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    }

    public void setupPathPlanner() {

    }

    public Command driveCommand() {
        return runOnce(null); //! Fix
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }
}
