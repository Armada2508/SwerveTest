package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveK;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * ? TODO: Need to tune current limits, pidfproperties, controllerproperties, maybe find wheel grip coefficient of friction
 */
public class Swerve extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    /**
     * 
     * @param directory Directory of swerve drive config files.
     */
    public Swerve() {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(SwerveK.steerGearRatio, 4096);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(SwerveK.driveGearRatio, SwerveK.wheelDiameter.in(Inches));
        System.out.println("Angle conversion factor: " + angleConversionFactor);
        // if (1 == 1)throw new RuntimeException("");

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SwerveParser parser = null;
        try {
            parser = new SwerveParser(SwerveK.swerveDirectory);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Swerve directory not found.");
        }
        swerveDrive = parser.createSwerveDrive(SwerveK.maxRobotSpeed.in(MetersPerSecond), angleConversionFactor, driveConversionFactor);
        for (var mod : swerveDrive.getModules()) { //^ BANDAID SOLUTION FOR INVERT ISSUE
            var motor = (WPI_TalonSRX) mod.getAngleMotor().getMotor();
            motor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
            // if (motor.getDeviceID() == 4) {
            // }
        }
    }

    @Override
    public void periodic() { // Calls constantly while robot is running
        for (var mod : swerveDrive.getModules()) { //^ BANDAID SOLUTION FOR INVERT ISSUE
            var motor = (WPI_TalonSRX) mod.getAngleMotor().getMotor();
            if (motor.getDeviceID() == 4) {
                                // motor.set(0.05);

                // System.out.println(motor.getSelectedSensorPosition());
                System.out.println("Motor:  " + motor.getDeviceID() + " |  Closed loop error: " + motor.getClosedLoopError() + " | Closed loop target: " + motor.getClosedLoopTarget() + " | Current Position: " + motor.getSelectedSensorPosition());
                SmartDashboard.putNumber("Error", motor.getClosedLoopError());
            }
        }
    }

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotVelocity, 
            this::setChassisSpeeds, 
            new HolonomicPathFollowerConfig(SwerveK.translationConstants, 
                                            SwerveK.rotationConstants, 
                                            SwerveK.maxRobotSpeed.in(MetersPerSecond), 
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
     * @param TranslationX Translation in the X direction (Forwards, Backwards)
     * @param TranslationY Translation in the Y direction (Left, Right)
     * @param angularVelocity Angular Velocity to set 
     * @return
     */
    public Command driveCommand(DoubleSupplier TranslationX, DoubleSupplier TranslationY, DoubleSupplier angularVelocity) {
        return runOnce(() -> drive(
                new Translation2d(TranslationX.getAsDouble() * swerveDrive.getMaximumVelocity(), TranslationY.getAsDouble() * swerveDrive.getMaximumVelocity()), 
                angularVelocity.getAsDouble() * swerveDrive.getMaximumAngularVelocity(), 
                false, true));
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

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    //! TODO: Fill out this info
    /**
     * 
     * @param xInput
     * @param yInput
     * @param angle
     * @return
     */
    public ChassisSpeeds getTargetChassisSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), getPose().getRotation().getRadians(), SwerveK.maxRobotSpeed.in(MetersPerSecond));                                        
    }
}
