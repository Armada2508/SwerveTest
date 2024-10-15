// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {

    private final Swerve swerve = new Swerve();
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    
    public Robot() {
        addPeriodic(() -> CommandScheduler.getInstance().run(), kDefaultPeriod);
        configureBindings();
        //! Buttons are for turning to directions, can cause conflict with definitions in configureBindings, change?
        // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv( //? Rename?
        //     swerve, 
        //     () -> MathUtil.applyDeadband(xboxController.getLeftX(), DriveK.leftJoystickDeadband), 
        //     () -> MathUtil.applyDeadband(xboxController.getLeftY(), DriveK.leftJoystickDeadband) ,
        //     () -> MathUtil.applyDeadband(xboxController.getRightX(), DriveK.rightJoystickDeadband), 
        //     xboxController.getHID()::getAButton, 
        //     xboxController.getHID()::getXButton, 
        //     xboxController.getHID()::getYButton, 
        //     xboxController.getHID()::getBButton
        // );

       // Fill in parameter info
        Command driveFieldOrientedAngularVelocity = swerve.driveCommand(
            () -> MathUtil.applyDeadband(-xboxController.getLeftY(), DriveK.leftJoystickDeadband), 
            () -> MathUtil.applyDeadband(-xboxController.getLeftX(), DriveK.leftJoystickDeadband), 
            () -> MathUtil.applyDeadband(-xboxController.getRightX(), DriveK.rightJoystickDeadband)
        );

        swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }
    
    private void configureBindings() {
        
    }

    @Override
    public void robotPeriodic() {
    }

}

