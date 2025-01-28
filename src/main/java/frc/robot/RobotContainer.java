// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.controllers.DriverMapping6237MR;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


//HERE!!! BLOCK IMPORTED FROM PHOENIX GENERATED SWERVE
import static edu.wpi.first.units.Units.*;

import java.nio.file.FileSystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
 //HERE!!! END BLOCK IMPORTED FROM PHOENIX GENERATED SWERVE


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    //HERE!!! BLOCK IMPORTED FROM PHOENIX GENERATED SWERVE
    //moving to Contants
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity




    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //HERE!!! END BLOCK IMPORTED FROM PHOENIX GENERATED SWERVE

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(Constants.kXboxDriverPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
  
    //HERE!!! - this probably could be or should be moved to the teleop init
    // s_Swerve.setDefaultCommand(
    //   new TeleopSwerve(
    //       s_Swerve, 
    //       () -> -driver.getRawAxis(DriverMapping6237MR.translationAxis), 
    //       () -> -driver.getRawAxis(DriverMapping6237MR.strafeAxis), 
    //       () -> -driver.getRawAxis(DriverMapping6237MR.rotationAxis), 
    //       () -> DriverMapping6237MR.robotCentric.getAsBoolean(),
    //       () -> DriverMapping6237MR.invertFrontAndBackButton.getAsBoolean()
    //   )
    // );
  }

  private void configureButtonBindings() {
    DriverMapping6237MR.mapXboxController(driver, drivetrain);

    //HERE!!! BLOCK IMPORTED FROM PHOENIX GENERATED SWERVE (moving to DriverMapping6237MR)

    // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);

    //HERE!!! END BLOCK IMPORTED FROM PHOENIX GENERATED SWERVE
  }

  public void fromRobotInit(){
    Orchestra theOrchestra = new Orchestra();
    theOrchestra.addInstrument(drivetrain.getAMotorAsParentDevice());
    theOrchestra.loadMusic(Filesystem.getDeployDirectory().getAbsolutePath() + "/output.chrp");
    theOrchestra.play();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
