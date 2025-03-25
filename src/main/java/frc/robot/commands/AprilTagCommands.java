package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightVisionSubsystem;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class AprilTagCommands {
    
    /**
     * Command to approach and align with an AprilTag
     */
    public static class ApproachAprilTagCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final LimelightVisionSubsystem vision;
        
        private static final double MAX_DRIVE_SPEED = 0.5; // m/s
        private static final double MAX_ROTATION_SPEED = 1.0; // rad/s
        
        private final PIDController alignmentController = new PIDController(0.03, 0, 0.001);
        private final PIDController distanceController = new PIDController(1.0, 0, 0.05);
        
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        public ApproachAprilTagCommand(CommandSwerveDrivetrain drivetrain, LimelightVisionSubsystem vision) {
            this.drivetrain = drivetrain;
            this.vision = vision;
            
            alignmentController.setTolerance(2.0); // 2 degrees tolerance
            distanceController.setTolerance(0.05); // 5cm tolerance
            
            addRequirements(drivetrain, vision);
        }
        
        @Override
        public void initialize() {
            // Set the Limelight to AprilTag detection mode
            vision.setPipeline(0); // Assuming pipeline 0 is configured for AprilTags
            SmartDashboard.putString("Current Command", "Approaching AprilTag");
        }
        
        @Override
        public void execute() {
            if (!vision.hasTarget()) {
                // No target visible - rotate to search
                drivetrain.setControl(drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0.5)); // Slow rotation to search
                return;
            }
            
            // Calculate alignment correction
            double horizontalOffset = vision.getHorizontalOffset();
            double rotationSpeed = alignmentController.calculate(horizontalOffset, 0);
            rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
            
            // Calculate approach speed
            double distance = vision.getEstimatedDistance();
            double targetDistance = 0.5; // target distance in meters
            double driveSpeed = distanceController.calculate(distance, targetDistance);
            driveSpeed = MathUtil.clamp(driveSpeed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
            
            // Use field-centric drive with the calculated speeds
            // Convert from robot-centric to field-centric as needed
            Rotation2d robotAngle = drivetrain.getState().Pose.getRotation();
            double xSpeed = driveSpeed * robotAngle.getCos();
            double ySpeed = driveSpeed * robotAngle.getSin();
            
            drivetrain.setControl(drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));
            
            // Debug info
            SmartDashboard.putNumber("Alignment Error", horizontalOffset);
            SmartDashboard.putNumber("Distance Error", distance - targetDistance);
            SmartDashboard.putNumber("Drive Speed", driveSpeed);
            SmartDashboard.putNumber("Rotation Speed", rotationSpeed);
        }
        
        @Override
        public boolean isFinished() {
            return vision.isReadyToGrab();
        }
        
        @Override
        public void end(boolean interrupted) {
            // Stop the drivetrain
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            SmartDashboard.putString("Current Command", "AprilTag Approach " + 
                (interrupted ? "Interrupted" : "Completed"));
        }
    }
    
    /**
     * Command to grab an object using the arm and intake
     */
    // public static class GrabCommand extends SequentialCommandGroup {
        
    //     public GrabCommand(Elevator elevator, Arm arm, Intake intake) {
    //         // Position the arm appropriately for grabbing
    //         // These positions will need to be adjusted based on your robot's geometry
    //         addCommands(
    //             new InstantCommand(() -> {
    //                 SmartDashboard.putString("Current Command", "Positioning Arm for Grab");
    //             }),
    //             // Lower elevator to grabbing position
    //             new InstantCommand(() -> elevator.setPosition(0.1)),
    //             // Move arm to grabbing position 
    //             new InstantCommand(() -> arm.setPosition(0.0)),
    //             // Wait for mechanisms to get in position
    //             new WaitCommand(1.0),
    //             // Activate intake to grab
    //             new InstantCommand(() -> intake.setSpeed(0.5)),
    //             // Wait for intake to secure the object
    //             new WaitCommand(0.5),
    //             // Stop intake after grabbing
    //             new InstantCommand(() -> intake.setSpeed(0.0)),
    //             // Move arm to secure position
    //             new InstantCommand(() -> arm.setPosition(0.3)),
    //             // Slightly raise elevator
    //             new InstantCommand(() -> elevator.setPosition(0.2))
    //         );
    //     }
    // }
    
    /**
     * Command to move to scoring position
     */
    public static class MoveToScoringPositionCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final Pose2d scoringPosition;
        
        private static final double MAX_DRIVE_SPEED = 1.0; // m/s
        private static final double MAX_ROTATION_SPEED = 1.5; // rad/s
        
        private final PIDController xController = new PIDController(1.0, 0, 0.05);
        private final PIDController yController = new PIDController(1.0, 0, 0.05);
        private final PIDController rotationController = new PIDController(0.8, 0, 0.05);
        
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        public MoveToScoringPositionCommand(CommandSwerveDrivetrain drivetrain, Pose2d scoringPosition) {
            this.drivetrain = drivetrain;
            this.scoringPosition = scoringPosition;
            
            xController.setTolerance(0.05); // 5cm tolerance
            yController.setTolerance(0.05); // 5cm tolerance
            rotationController.setTolerance(Math.toRadians(2.0)); // 2 degrees tolerance
            
            addRequirements(drivetrain);
        }
        
        @Override
        public void initialize() {
            SmartDashboard.putString("Current Command", "Moving to Scoring Position");
        }
        
        @Override
        public void execute() {
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Calculate position errors
            double xError = scoringPosition.getX() - currentPose.getX();
            double yError = scoringPosition.getY() - currentPose.getY();
            
            // Calculate heading error
            double targetAngle = scoringPosition.getRotation().getRadians();
            double currentAngle = currentPose.getRotation().getRadians();
            double angleError = MathUtil.angleModulus(targetAngle - currentAngle);
            
            // Calculate control signals
            double xSpeed = xController.calculate(currentPose.getX(), scoringPosition.getX());
            double ySpeed = yController.calculate(currentPose.getY(), scoringPosition.getY());
            double rotationSpeed = rotationController.calculate(currentAngle, targetAngle);
            
            // Apply limits
            xSpeed = MathUtil.clamp(xSpeed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
            ySpeed = MathUtil.clamp(ySpeed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
            rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
            
            // Apply control to the drivetrain
            drivetrain.setControl(drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));
            
            // Debug info
            SmartDashboard.putNumber("X Error", xError);
            SmartDashboard.putNumber("Y Error", yError);
            SmartDashboard.putNumber("Angle Error", Math.toDegrees(angleError));
        }
        
        @Override
        public boolean isFinished() {
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Check if we're at the target position and orientation
            double xError = scoringPosition.getX() - currentPose.getX();
            double yError = scoringPosition.getY() - currentPose.getY();
            double positionError = Math.sqrt(xError * xError + yError * yError);
            
            double angleError = Math.abs(MathUtil.angleModulus(
                scoringPosition.getRotation().getRadians() - currentPose.getRotation().getRadians()));
            
            return positionError < 0.1 && angleError < Math.toRadians(5.0);
        }
        
        @Override
        public void end(boolean interrupted) {
            // Stop the drivetrain
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            SmartDashboard.putString("Current Command", "Move to Scoring Position " + 
                (interrupted ? "Interrupted" : "Completed"));
        }
    }
    
    /**
     * Command to score the game piece
     */
    // public static class ScoreCommand extends SequentialCommandGroup {
        
    //     public ScoreCommand(Elevator elevator, Arm arm, Intake intake) {
    //         // This sequence will need to be adjusted based on your robot's specific mechanisms
    //         addCommands(
    //             new InstantCommand(() -> {
    //                 SmartDashboard.putString("Current Command", "Scoring");
    //             }),
    //             // Position elevator for scoring
    //             new InstantCommand(() -> elevator.setPosition(0.8)),
    //             // Position arm for scoring
    //             new InstantCommand(() -> arm.setPosition(0.5)),
    //             // Wait for mechanisms to get in position
    //             new WaitCommand(1.5),
    //             // Activate intake to release
    //             new InstantCommand(() -> intake.setSpeed(-0.3)),
    //             // Wait for release
    //             new WaitCommand(0.5),
    //             // Stop intake
    //             new InstantCommand(() -> intake.setSpeed(0.0)),
    //             // Return arm to safe position
    //             new InstantCommand(() -> arm.setPosition(0.0)),
    //             // Lower elevator
    //             new InstantCommand(() -> elevator.setPosition(0.1))
    //         );
    //     }
    // }
    
    /**
     * Complete autonomous sequence to find, grab, and score an AprilTag
     */
    public static class AprilTagAutonomousSequence extends SequentialCommandGroup {
        
        public AprilTagAutonomousSequence(
                CommandSwerveDrivetrain drivetrain,
                LimelightVisionSubsystem vision,
                Elevator elevator,
                Arm arm,
                Intake intake,
                Pose2d scoringPosition) {
            
            addCommands(
                // Step 1: Find and approach the AprilTag
                new ApproachAprilTagCommand(drivetrain, vision),
                
                // Step 2: Grab the game piece
                // new GrabCommand(elevator, arm, intake),
                
                // Step 3: Move to scoring position
                new MoveToScoringPositionCommand(drivetrain, scoringPosition)
                
                // Step 4: Score
                // ,new ScoreCommand(elevator, arm, intake)
            );
        }
    }

    public static class MoveToTranslationCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private Pose2d scoringPosition;
        private Transform2d storedTransform;

        
        private static final double MAX_DRIVE_SPEED = 1.0; // m/s
        private static final double MAX_ROTATION_SPEED = 1.5; // rad/s
        
        private final PIDController xController = new PIDController(10, 0, 0.05);
        private final PIDController yController = new PIDController(10, 0, 0.05);
        private final PIDController rotationController = new PIDController(7, 0, 0.05);
        
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        public MoveToTranslationCommand(CommandSwerveDrivetrain drivetrain, Transform2d newTransform) {
            this.drivetrain = drivetrain;
            storedTransform = newTransform;
            
            xController.setTolerance(0.05); // 5cm tolerance
            yController.setTolerance(0.05); // 5cm tolerance
            rotationController.setTolerance(Math.toRadians(2.0)); // 2 degrees tolerance
            
            addRequirements(drivetrain);
        }
        
        @Override
        public void initialize() {
            this.scoringPosition = drivetrain.getState().Pose.transformBy(storedTransform);
            SmartDashboard.putString("Current Command", "Moving to Scoring Position");
            SmartDashboard.putString("Initialized pose from drivetrain:   ", "X=" + drivetrain.getState().Pose.getX() + " Y=" + drivetrain.getState().Pose.getY());
            SmartDashboard.putString("Scoring Position:   ", "X=" + scoringPosition.getX() + " Y=" + scoringPosition.getY());
        }
        
        @Override
        public void execute() {
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Calculate position errors
            double xError = scoringPosition.getX() - currentPose.getX();
            double yError = scoringPosition.getY() - currentPose.getY();
            
            // Calculate heading error
            double targetAngle = scoringPosition.getRotation().getRadians();
            double currentAngle = currentPose.getRotation().getRadians();
            double angleError = MathUtil.angleModulus(targetAngle - currentAngle);
            
            // Calculate control signals
            double xSpeed = xController.calculate(currentPose.getX(), scoringPosition.getX());
            double ySpeed = yController.calculate(currentPose.getY(), scoringPosition.getY());
            double rotationSpeed = rotationController.calculate(currentAngle, targetAngle);
            
            // Apply limits
            xSpeed = MathUtil.clamp(xSpeed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
            ySpeed = MathUtil.clamp(ySpeed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
            rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
            
            // Apply control to the drivetrain
            drivetrain.setControl(drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));
            
            // Debug info
            SmartDashboard.putNumber("X Error", xError);
            SmartDashboard.putNumber("Y Error", yError);
            SmartDashboard.putNumber("Angle Error", Math.toDegrees(angleError));
        }
        
        @Override
        public boolean isFinished() {
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Check if we're at the target position and orientation
            double xError = scoringPosition.getX() - currentPose.getX();
            double yError = scoringPosition.getY() - currentPose.getY();
            double positionError = Math.sqrt(xError * xError + yError * yError);
            
            double angleError = Math.abs(MathUtil.angleModulus(
                scoringPosition.getRotation().getRadians() - currentPose.getRotation().getRadians()));
            
            return positionError < 0.1 && angleError < Math.toRadians(5.0);
        }
        
        @Override
        public void end(boolean interrupted) {
            // Stop the drivetrain
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            SmartDashboard.putString("Current Command", "Move to Scoring Position " + 
                (interrupted ? "Interrupted" : "Completed"));
        }
    }
}
