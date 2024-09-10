package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Swerve;

public class AbsoluteDriveAdv {

    private final Swerve swerve; 
    private final DoubleSupplier vX, vY, headingAdjust; 
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading;

    /**
     * @param swerve Swerve Subsystem
     * @param vX Double supplier X component of joystick input after deadband is applied, should be -1 to 1
     * @param vY Double supplier Y component of joystick input after deadband is applied, should be -1 to 1
     * @param headingAdjust Double supplier current robot heading to be adjusted after deadband is applied
     * @param lookAway Faces the robot towards the opposind alliance wall from the driver
     * @param lookTowards Faces the robot towards the driver
     * @param lookLeft Faces the robot left
     * @param lookRight Faces the robot right
     * 
     * @return
     */ 
    public AbsoluteDriveAdv(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight) {

                            this.swerve = swerve;
                            this.vX = vX;
                            this.vY = vY;
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
    }

    public void end() {

    }

    public boolean isFinished() {
        return false; //!
    }
}
