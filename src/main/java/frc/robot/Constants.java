package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class Constants {
    public static class SwerveK {
        public static final int driveMotorID = 0;
        public static final int turnMotorID = 1;
        public static final int encoderID = 2;

        public static final Measure<Distance> wheelDiameter = Inches.of(3); 
        public static final Measure<Distance> driveBaseRadius = Meters.of(0.4579874);

        public static final double steerGearRatio = 12.8; //! Figure Out;
        public static final double driveGearRatio = 6; //! Figure Out

        public static final Measure<Velocity<Distance>> maxModuleSpeed = MetersPerSecond.of(0); //! Tune

        public static final PIDConstants translationConstants = new PIDConstants(1, 1, 1); //! Tune
        public static final PIDConstants rotationConstants = new PIDConstants(1, 1, 1); //! Tune

    }
}
