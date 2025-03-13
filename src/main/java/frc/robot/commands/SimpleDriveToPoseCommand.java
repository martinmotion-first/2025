package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.limelightlib.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class SimpleDriveToPoseCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Pose2d m_targetPose;
    
    // Controllers for X, Y, and rotation
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
    
    // Field-centric swerve request
    private final SwerveRequest.FieldCentric m_request = new SwerveRequest.FieldCentric();
    
    // Constants for maximum speeds
    private final double m_maxSpeed;
    private final double m_maxRotationSpeed;
    
    /**
     * Simple command to drive to a pose using PID controllers
     * 
     * @param drivetrain The swerve drivetrain
     * @param targetPose The target pose to drive to
     * @param positionTolerance Position tolerance in meters
     * @param rotationTolerance Rotation tolerance in radians
     * @param maxSpeed Maximum speed in meters per second
     * @param maxRotationSpeed Maximum rotation speed in radians per second
     */
    public SimpleDriveToPoseCommand(
            CommandSwerveDrivetrain drivetrain,
            Pose2d targetPose,
            double positionTolerance,
            double rotationTolerance,
            double maxSpeed,
            double maxRotationSpeed) {
        m_drivetrain = drivetrain;
        m_targetPose = targetPose;
        m_maxSpeed = maxSpeed;
        m_maxRotationSpeed = maxRotationSpeed;
        
        // Create the PID controllers
        m_xController = new PIDController(1.0, 0.0, 0.0);
        m_yController = new PIDController(1.0, 0.0, 0.0);
        m_rotationController = new PIDController(2.0, 0.0, 0.0);
        
        // Set the tolerance for the controllers
        m_xController.setTolerance(positionTolerance);
        m_yController.setTolerance(positionTolerance);
        m_rotationController.setTolerance(rotationTolerance);
        
        // Make the rotation controller continuous
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivetrain);
    }
    
    /**
     * Simple command to drive to a pose using default parameters
     * 
     * @param drivetrain The swerve drivetrain
     * @param targetPose The target pose to drive to
     */
    public SimpleDriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this(drivetrain, targetPose, 0.1, 0.05, 2, Math.PI / 2);
    }
    
    @Override
    public void initialize() {
        // Reset the PID controllers
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();
        
 
    }
    
    @Override
    public void execute() {       
       // Log the target pose
       SmartDashboard.putString("Target Pose", 
       String.format("X: %.2f, Y: %.2f, Rot: %.2f", 
           m_targetPose.getX(), m_targetPose.getY(), 
           m_targetPose.getRotation().getDegrees()));

            // Get the current robot pose
            Pose2d currentPose = m_drivetrain.getState().Pose;
            
            // Calculate the error
            double xError = m_targetPose.getX() - currentPose.getX();
            double yError = m_targetPose.getY() - currentPose.getY();
            double rotationError = m_targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
            
            // Calculate the PID outputs
            double xOutput = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
            double yOutput = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
            double rotationOutput = m_rotationController.calculate(
                currentPose.getRotation().getRadians(), 
                m_targetPose.getRotation().getRadians()
            );
            
            // Clamp the outputs to the maximum speed
            xOutput = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, xOutput));
            yOutput = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, yOutput));
            rotationOutput = Math.max(-m_maxRotationSpeed, Math.min(m_maxRotationSpeed, rotationOutput));
            
            // if(LimelightHelpers.hasTarget()){ 
            if(m_targetPose.getX() != 0 && m_targetPose.getY() != 0){
                // Apply the outputs to the drivetrain
                m_drivetrain.setControl(
                    m_request
                        .withVelocityX(xOutput)
                        .withVelocityY(yOutput)
                        .withRotationalRate(rotationOutput)
                );
            }
            // }
            
            // // Log the errors
            SmartDashboard.putNumber("Pose X Error", xError);
            SmartDashboard.putNumber("Pose Y Error", yError);
            SmartDashboard.putNumber("Pose Rot Error", rotationError);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        m_drivetrain.setControl(new SwerveRequest.Idle());
        
        SmartDashboard.putBoolean("At Target Pose", !interrupted);
    }
    
    @Override
    public boolean isFinished() {
        // Check if all controllers are at their setpoints
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotationController.atSetpoint();
    }
}
