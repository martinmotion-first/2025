package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.limelightlib.LimelightHelpers;

public class LimelightVisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    
    // Constants for AprilTag alignment
    // public static final double ALIGNMENT_THRESHOLD = 2.0; // degrees
    // public static final double DISTANCE_THRESHOLD = 0.3; // meters
    
    public LimelightVisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    
    /**
     * Sets the Limelight pipeline for AprilTag detection
     * @param pipelineIndex The pipeline index to set
     */
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(Constants.kLimelightName, pipelineIndex);
    }
    
    /**
     * Check if the Limelight has a valid target
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(Constants.kLimelightName);
    }
    
    /**
     * Get the ID of the detected AprilTag
     */
    public int getTargetID() {
        return (int)LimelightHelpers.getFiducialID(Constants.kLimelightName);
    }
    
    /**
     * Get horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(Constants.kLimelightName);
    }
    
    /**
     * Get vertical offset from crosshair to target (-24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(Constants.kLimelightName);
    }
    
    /**
     * Get the target area (0% to 100% of image)
     */
    public double getTargetArea() {
        return LimelightHelpers.getTA(Constants.kLimelightName);
    }
    
    /**
     * Get the estimated distance to the target in meters
     * This is a simplistic calculation based on target area
     */
    public double getEstimatedDistance() {
        // Get the transform to the target in camera space
        Pose3d targetToCamera = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.kLimelightName);
        if (targetToCamera != null) {
            // The Z component is the distance from camera to target
            return targetToCamera.getTranslation().getNorm();
        }
        
        // Fallback calculation using target area if 3D pose isn't available
        double targetArea = getTargetArea();
        if (targetArea < 0.1) return 5.0; // Far away
        return 5.0 / Math.sqrt(targetArea);
    }
    
    /**
     * Get the robot pose relative to the AprilTag in field coordinates
     */
    public Pose2d getRobotPoseRelativeToTag() {
        // return LimelightHelpers.getBotPose2d_TargetSpace(Constants.kLimelightName).toPose2D();
        return LimelightHelpers.getBotPose3d_TargetSpace(Constants.kLimelightName).toPose2d();
    }
    
    /**
     * Check if the robot is aligned with the AprilTag
     */
    public boolean isAligned() {
        return hasTarget() && Math.abs(getHorizontalOffset()) < Constants.AprilTag.ALIGNMENT_THRESHOLD;
    }
    
    /**
     * Check if the robot is at the right distance to grab
     */
    public boolean isAtGrabbingDistance() {
        return hasTarget() && Math.abs(getEstimatedDistance() - Constants.AprilTag.TARGET_DISTANCE) < Constants.AprilTag.DISTANCE_THRESHOLD;
    }

    /**
 * Get the front-to-back distance from the robot to the AprilTag in meters.
 * Positive values indicate the target is in front of the robot.
 * Negative values indicate the target is behind the robot.
 * 
 * @return The front-to-back distance in meters, or -9999 if no target is detected
 */
public double getFrontToBackDistance() {
    if (!hasTarget()) {
        return -9999; // No valid target detected
    }
    
    // Method 1: Direct access to botpose array (most reliable for front-to-back)
    double[] botpose = LimelightHelpers.getBotPose(Constants.kLimelightName);
    if (botpose != null && botpose.length >= 3) {
        return botpose[2]; // Z component is front-to-back distance
    }
    
    // Method 2: Use targetpose_robotspace which gives target position relative to robot
    double[] targetpose_robotspace = LimelightHelpers.getTargetPose_RobotSpace(Constants.kLimelightName);
    if (targetpose_robotspace != null && targetpose_robotspace.length >= 3) {
        return targetpose_robotspace[2]; // Z component is front-to-back distance
    }
    
    // Method 3: Use the camera-relative pose and transform it
    Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.kLimelightName);
    if (targetPose != null) {
        // Note: This is a simplification, and doesn't account for camera mounting position
        return targetPose.getZ(); 
    }
    
    // Fallback method using target area
    // This is very approximate and should only be used if all other methods fail
    double targetArea = getTargetArea();
    if (targetArea > 0.1) {
        return 5.0 / Math.sqrt(targetArea);
    }
    
    return -9999; // Couldn't determine distance
}

public double getYaw(){
    if (!hasTarget()) {
        return -9999; // No valid target detected
    }
    double[] botpose = LimelightHelpers.getBotPose(Constants.kLimelightName);
    if (botpose != null && botpose.length >= 5) {
        return botpose[5]; 
    }
    return -9999;
}

/**
 * Checks if we're at the correct front-to-back distance for interaction
 * with the AprilTag (e.g., picking up a game piece)
 * 
 * @return true if at the proper distance within tolerance
 */
public boolean isAtCorrectFrontToBackDistance() {
    double distance = getFrontToBackDistance();
    if (distance == -9999) {
        return false; // No valid target
    }
    
    return Math.abs(distance - Constants.AprilTag.TARGET_DISTANCE) < Constants.AprilTag.DISTANCE_THRESHOLD;
}

/**
 * Updates the isReadyToGrab method to use front-to-back distance
 */
public boolean isReadyToGrab() {
    return isAligned() && isAtCorrectFrontToBackDistance();
}

    /**
     * Updates the robot's odometry using AprilTag vision data
     */
    public void updateOdometryWithVision() {
        if (hasTarget()) {
            Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.kLimelightName);
            if (visionPose != null) {
                drivetrain.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
            }
        }
    }
    
    @Override
    public void periodic() {
        // Display useful information on the dashboard
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        if (hasTarget()) {
            SmartDashboard.putNumber("Limelight Target ID", getTargetID());
            SmartDashboard.putNumber("Limelight Horizontal Offset", getHorizontalOffset());
            SmartDashboard.putNumber("Limelight Distance", getEstimatedDistance());
            SmartDashboard.putBoolean("Limelight Aligned", isAligned());
            SmartDashboard.putBoolean("Limelight At Grabbing Distance", isAtGrabbingDistance());
        }
    }
}
