package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve;

public class AbsoluteDriveAdv {

    private final Swerve swerve; 
    private final DoubleSupplier vXInput, vYInput, headingAdjust; 
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading;

    /**
     * @param swerve Swerve Subsystem
     * @param vXInput Double supplier X component of joystick input after deadband is applied, should be -1 to 1
     * @param vYInput Double supplier Y component of joystick input after deadband is applied, should be -1 to 1
     * @param headingAdjust Double supplier current robot heading to be adjusted after deadband is applied
     * @param lookAway Faces the robot towards the opposind alliance wall from the driver
     * @param lookTowards Faces the robot towards the driver
     * @param lookLeft Faces the robot left
     * @param lookRight Faces the robot right
     * 
     * @return
     */ 
    public AbsoluteDriveAdv(Swerve swerve, DoubleSupplier vXInput, DoubleSupplier vYInput, DoubleSupplier headingAdjust,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight) {

                            this.swerve = swerve;
                            this.vXInput = vXInput;
                            this.vYInput = vYInput;
                            this.headingAdjust = headingAdjust;
                            this.lookAway = lookAway;
                            this.lookTowards = lookTowards;
                            this.lookLeft = lookLeft;
                            this.lookRight = lookRight;
    }

    public void initialize() {
        resetHeading = true;
    }

    public void execute() {
        double headingX = 0; // Resets values
        double headingY = 0;

        if (lookAway.getAsBoolean()) { // Y
            headingY = -1;
        }
        if (lookTowards.getAsBoolean()) { // Y
            headingY = 1;
        }
        if (lookLeft.getAsBoolean()) { // X
            headingX = -1;
        }
        if (lookRight.getAsBoolean()) { // X
            headingX = 1;
        }

        ChassisSpeeds desiredSpeed = swerve.getTargetChassisSpeeds(vXInput.getAsDouble(), vYInput.getAsDouble(), new Rotation2d(headingAdjust.getAsDouble()));

        swerve.drive(new Translation2d(headingX, headingY), headingAdjust.getAsDouble(), true, true); //verify if correct
    }

    public void end() {
        
    }

    public boolean isFinished() {
        return false; //! TODO:
    }
}
