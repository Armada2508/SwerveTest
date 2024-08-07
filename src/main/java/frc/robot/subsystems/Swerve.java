package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveK;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    // double maximumSpeed = Units.feetToMeters(4.5);

    /**
     * 
     * @param directory Directory of swerve drive config files.
     */
    public Swerve(File directory) throws IOException {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(SwerveK.steerGearRatio);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(SwerveK.driveGearRatio, SwerveK.wheelDiameter.in(Inches));

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveK.maxModuleSpeed.in(MetersPerSecond), angleConversionFactor, driveConversionFactor);
    }

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotVelocity, 
            this::setChassisSpeeds, 
            new HolonomicPathFollowerConfig(SwerveK.translationConstants, 
                                            SwerveK.rotationConstants, 
                                            SwerveK.maxModuleSpeed.in(MetersPerSecond), 
                                            SwerveK.driveBaseRadius.in(Meters), 
                                            null), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                } return false;
            }
            ,this);
    }

    /**
     * 
     * @param TranslationX Translation in the X direction
     * @param TranslationY Translation in the Y direction
     * @param angularVelocity Angular Velocity to set 
     * @return
     */
    public Command driveCommand(double TranslationX, double TranslationY, double angularVelocity) {
        return runOnce(() -> drive(
                new Translation2d(TranslationX * swerveDrive.getMaximumVelocity(), TranslationY * swerveDrive.getMaximumVelocity()), 
                angularVelocity * swerveDrive.getMaximumAngularVelocity(), 
                true, false));
    }

    public Command turnCommand(Measure<Angle> targetAngle, Measure<Angle> currentAngle, boolean fieldRelative) {
        return runOnce(() -> turn(targetAngle, currentAngle, fieldRelative));
    }

    public void turn(Measure<Angle> targetAngle, Measure<Angle> currentAngle, boolean fieldRelative) {
        drive(getPose().getTranslation(), getTurningAngle(targetAngle, currentAngle).in(Degrees), fieldRelative, false);
    }

    public Measure<Angle> getTurningAngle(Measure<Angle> desiredAngle, Measure<Angle> currentHeading) {
        double angle = (desiredAngle.minus(currentHeading).plus(Degrees.of(540))).in(Degrees);
        angle = (angle % 360) - 180;
        return Degrees.of(angle);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
}
