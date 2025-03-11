import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlignToAprilTag extends Command {
    private final SwerveDrivetrain swerve;
    private Pose2d targetPose;

    public AlignToAprilTag(SwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (!LimelightHelper.hasTarget()) {
            System.out.println("No AprilTag detected.");
            return;
        }

        Pose3d tagPose = LimelightHelper.getTagPose();

        // Convert AprilTag's 3D pose to a 2D target position (on the ground)
        double tagX = tagPose.getTranslation().getX(); // Forward/backward
        double tagY = tagPose.getTranslation().getY(); // Left/right
        double tagZ = tagPose.getTranslation().getZ(); // Depth (distance from robot to tag)

        // Set target position 1 foot (0.3048 meters) away from the tag along its depth axis
        double desiredDistance = 0.3048;
        double approachX = tagX - desiredDistance * Math.cos(tagPose.getRotation().getZ());
        double approachY = tagY - desiredDistance * Math.sin(tagPose.getRotation().getZ());

        targetPose = new Pose2d(approachX, approachY, new Rotation2d(tagPose.getRotation().getZ()));

        System.out.println("Aligning to: " + targetPose);
    }

    @Override
    public void execute() {
        if (targetPose != null) {
            swerve.getAutoBuilder().driveToPose(targetPose);
        }
    }

    @Override
    public boolean isFinished() {
        return swerve.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.05; // Stop when within 5 cm
    }
}