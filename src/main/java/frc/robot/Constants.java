package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import java.io.File;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static class SwerveK {
        public static final Distance wheelDiameter = Inches.of(3); 
        public static final Distance driveBaseRadius = Meters.of(0.4579874);

        public static final double steerGearRatio = 1;
        public static final double driveGearRatio = 6;

        public static final LinearVelocity maxRobotSpeed = MetersPerSecond.of(4.24);

        public static final PIDConstants translationConstants = new PIDConstants(1, 1, 1); //! TODO: Tune
        public static final PIDConstants rotationConstants = new PIDConstants(1, 1, 1); //! TODO: Tune
        public static final ModuleConfig moduleConfig = new ModuleConfig(wheelDiameter.div(2), maxRobotSpeed, 1, null, Amps.of(40), 1); //! TODO: Tune
        public static final RobotConfig robotConfig = new RobotConfig(Pounds.of(0), KilogramSquareMeters.of(0), moduleConfig, new Translation2d[4]); //! TODO: Tune

        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/swerve");
    }

    public static class ControllerK {
        public static final int xboxPort = 0;
        public static final double leftJoystickDeadband = 0.05;
        public static final double rightJoystickDeadband = 0.05;
    }

    public static class DriveK {
        public static final DynamicSlewRateLimiter translationalYLimiter = new DynamicSlewRateLimiter(1.25, 2); // Larger number = faster rate of change
        public static final DynamicSlewRateLimiter translationalXLimiter = new DynamicSlewRateLimiter(1.25, 2);
        public static final DynamicSlewRateLimiter rotationalLimiter = new DynamicSlewRateLimiter(1.25, 2);
    }
    
}
