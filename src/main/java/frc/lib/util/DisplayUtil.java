package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

public class DisplayUtil {
    
    public static Trajectory offsetTrajectoryCoordinatesForDisplayByXAndY(Trajectory originalTrajectory, double xMeters, double yMeters){
        Transform2d displayAdjustment = new Transform2d(xMeters, yMeters, new Rotation2d(0));
        Trajectory adjustedTrajectory = originalTrajectory.transformBy(displayAdjustment);
        return adjustedTrajectory;
    }

    public static Trajectory invertXValuesForRedStartingCoordinates(Trajectory originalTrajectory){
        Pose2d initialPose = originalTrajectory.getInitialPose();
        double newInitialPoseX = -initialPose.getX();
        double newInitialPoseY = initialPose.getY();
        Rotation2d newPoseRotation = initialPose.getRotation();
        Pose2d modifiedInitialPose2d = new Pose2d(newInitialPoseX, newInitialPoseY, newPoseRotation);


        Pose2d finalPose = originalTrajectory.getStates().get(originalTrajectory.getStates().size() - 1).poseMeters;
        double newFinalPoseX = finalPose.getX();
        double newFinalPoseY = finalPose.getY();
        Rotation2d newFinalPoseRotation = finalPose.getRotation();
        Pose2d modifiedFinalPose2d = new Pose2d(newFinalPoseX, newFinalPoseY, newFinalPoseRotation);
        
        Trajectory adjustedTrajectory = originalTrajectory.transformBy(new Transform2d(modifiedInitialPose2d, modifiedFinalPose2d)); 

        return adjustedTrajectory;
    }

    public static void log(String caller, String message){
        DriverStation.reportError("***************************" + caller + "***************************", new StackTraceElement[0]);
        DriverStation.reportError(">>>" + message, new StackTraceElement[0]);
    }
}