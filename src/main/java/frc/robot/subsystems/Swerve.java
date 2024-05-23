package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveK;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

    SwerveDrive swerveDrive;
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");


    public Swerve(File directory) throws IOException {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(SwerveK.steerGearRatio);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(SwerveK.driveGearRatio, SwerveK.driveGearRatio);


        swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    }
}
