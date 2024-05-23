package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class Constants {
    public static class SwerveK {
        public static int driveMotorID = 0;
        public static int turnMotorID = 1;
        public static int encoderID = 2;

        public static Measure<Distance> wheelDiameter = Inches.of(3);

        public static double steerGearRatio = 12.8;
        public static double driveGearRatio = 6;
    }
}
