package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static class SwerveK {
        public static final Measure<Distance> wheelDiameter = Inches.of(3); 
        public static final Measure<Distance> driveBaseRadius = Meters.of(0.4579874);

        public static final double steerGearRatio = 1;
        public static final double driveGearRatio = 6;

        public static final Measure<Velocity<Distance>> maxRobotSpeed = MetersPerSecond.of(4.24);

        public static final PIDConstants translationConstants = new PIDConstants(1, 1, 1); //! TODO: Tune
        public static final PIDConstants rotationConstants = new PIDConstants(1, 1, 1); //! TODO: Tune

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
