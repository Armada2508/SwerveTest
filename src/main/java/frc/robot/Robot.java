// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerK;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {

    private final Swerve swerve = new Swerve();

    final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    
    public Robot() {
        configureController();
    }

    private void configureController() {
        
    }

}

