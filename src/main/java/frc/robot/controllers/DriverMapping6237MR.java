package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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

        /* Setting up bindings for necessary control of the swerve drive platform */
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * Constants.driverStickDeadband).withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.driverStickDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private static final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);

    private static final CommandXboxController joystick = new CommandXboxController(0);

    public static void mapXboxController(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain) {
        // robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        // invertFrontAndBackButton = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        //switching for Rachel
        robotCentric = new Trigger(() -> driverController.getLeftTriggerAxis() > Constants.kTriggerButtonThreshold);
        invertFrontAndBackButton = new Trigger(() -> driverController.getRightTriggerAxis() > Constants.kTriggerButtonThreshold);


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.Swerve.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Constants.Swerve.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.Swerve.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.y().onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
}
