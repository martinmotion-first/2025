package frc.robot.controllers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.commands.AprilTagCommands;
import frc.robot.commands.SimpleDriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.limelightlib.LimelightHelpers;
import frc.robot.limelightlib.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightVisionSubsystem;

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

    private static double invertXNumber = -1.0;
    private static double invertYNumber = -1.0;
    private static NetworkTable limelight;

    public static void mapXboxController(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain, LimelightVisionSubsystem limelight) {
        // robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        // invertFrontAndBackButton = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        //switching for Rachel
        robotCentric = new Trigger(() -> driverController.getLeftTriggerAxis() > Constants.kTriggerButtonThreshold);
        invertFrontAndBackButton = new Trigger(() -> driverController.getRightTriggerAxis() > Constants.kTriggerButtonThreshold);

        Command defaultDrivetrainCommand = drivetrain.applyRequest(() ->
            drive.withVelocityX(invertXNumber * joystick.getLeftY() * Constants.Swerve.MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(invertYNumber * joystick.getLeftX() * Constants.Swerve.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-1 * joystick.getRightX() * Constants.Swerve.MaxAngularRate)); // Drive counterclockwise with negative X (left)

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            defaultDrivetrainCommand
        );

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        
        driverController.y().debounce(1.0).onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));
        driverController.leftBumper().debounce(1.0).onTrue(new InstantCommand(() -> invertXNumber *= -1.0));
        driverController.rightBumper().debounce(1.0).onTrue(new InstantCommand(() -> invertYNumber *= -1.0));
        
        
        // driverController.a().onTrue(new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.kLimelightName, 1)));
        
        // driverController.a().onTrue(new SimpleDriveToPoseCommand(drivetrain, LimelightHelpers.getTargetPose3d_CameraSpace(Constants.kLimelightName).toPose2d()));
        // driverController.b().onTrue(new SimpleDriveToPoseCommand(drivetrain, LimelightHelpers.getBotPose2d(Constants.kLimelightName)));
        // driverController.x().onTrue(new SimpleDriveToPoseCommand(drivetrain, LimelightHelpers.getBotPose2d(Constants.kLimelightName)));

        // SequentialCommandGroup a = new SequentialCommandGroup(
        //     new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.kLimelightName, 1)),
        //     new SimpleDriveToPoseCommand(drivetrain, LimelightHelpers.getBotPose3d_TargetSpace(Constants.kLimelightName).toPose2d())
        // );
        // SequentialCommandGroup x = new SequentialCommandGroup(
        //     new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.kLimelightName, 1)),
        //     new SimpleDriveToPoseCommand(drivetrain, LimelightHelpers.getTargetPose3d_CameraSpace(Constants.kLimelightName).toPose2d())
        // );
        // SequentialCommandGroup b = new SequentialCommandGroup(
        //     new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.kLimelightName, 1)),
        //     new SimpleDriveToPoseCommand(drivetrain, LimelightHelpers.getCameraPose3d_RobotSpace(Constants.kLimelightName).toPose2d())
        // );
        // driverController.a().onTrue(a).onFalse(defaultDrivetrainCommand);
        // driverController.b().onTrue(b).onFalse(defaultDrivetrainCommand);
        // driverController.x().onTrue(x).onFalse(defaultDrivetrainCommand);

        // driverController.x().onTrue(new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.kLimelightName, 2)));
        // driverController.b().onTrue(new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.kLimelightName, 3)));

        // reset the field-centric heading on left bumper press
        // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driverController.a().whileTrue(new AprilTagCommands.ApproachAprilTagCommand(drivetrain, limelight)).onFalse(defaultDrivetrainCommand);
        driverController.b().onTrue(new AprilTagCommands.MoveToTranslationCommand(drivetrain, new Transform2d(new Translation2d(.3, -.5), new Rotation2d()))).onFalse(defaultDrivetrainCommand);
        driverController.x().onTrue(new AprilTagCommands.MoveToTranslationCommand(drivetrain, new Transform2d(new Translation2d(.3, .5), new Rotation2d()))).onFalse(defaultDrivetrainCommand);

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
}
