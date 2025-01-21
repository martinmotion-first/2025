package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverMapping6237MR {

    /* Driver Buttons */
    public static JoystickButton zeroGyro;
    // public static JoystickButton robotCentric;
    public static Trigger robotCentric;
    // public static JoystickButton invertFrontAndBackButton;
    public static Trigger invertFrontAndBackButton;


    /* Drive Controls */
    public static int translationAxis = XboxController.Axis.kLeftY.value;
    public static int strafeAxis = XboxController.Axis.kLeftX.value;
    public static int rotationAxis = XboxController.Axis.kRightX.value;

    public static void mapXboxController(XboxController driverController, SwerveSubsystem swerveDrive) {
        zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
        // robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        // invertFrontAndBackButton = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        //switching for Rachel
        robotCentric = new Trigger(() -> driverController.getLeftTriggerAxis() > Constants.kTriggerButtonThreshold);
        invertFrontAndBackButton = new Trigger(() -> driverController.getRightTriggerAxis() > Constants.kTriggerButtonThreshold);
        

        zeroGyro.onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
    }
    
}
