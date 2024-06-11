package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerveSubsystem;

    private final PhotonCamera apriltagCamera = new PhotonCamera("Camera_Module_v1");
    // private final PhotonCamera objectCamera = new PhotonCamera("ObjectDetection");

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void periodic() {
        estimateLocalPose();

        // Returns the current pose (position and rotation) of the robot, as reported by odometry
        // Results should be relative of the blue alliance driverstation wall
        Pose2d robotPose = swerveSubsystem.getPose();

        double swerveXPosition = robotPose.getX();
        double swerveYPosition = robotPose.getY();
        double swerveRotation = robotPose.getRotation().getDegrees();

        System.out.println("<--------------->");
        System.out.println("Swerve X Position: " + swerveXPosition);
        System.out.println("Swerve Y Position: " + swerveYPosition);
        System.out.println("Swerve Rotation: " + swerveRotation);
        System.out.println("<--------------->");

        SmartDashboard.putNumber("FieldXPosition", swerveXPosition);
        SmartDashboard.putNumber("FieldYPosition", swerveYPosition);
        SmartDashboard.putNumber("FieldRotation", swerveRotation);
    }

    private void estimateLocalPose() {
        var result = apriltagCamera.getLatestResult();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d fieldToCameraPose = result.getMultiTagResult().estimatedPose.best;

            double xPosition = fieldToCameraPose.getX();
            double yPosition = fieldToCameraPose.getY();
            Rotation2d rotation = new Rotation2d(fieldToCameraPose.getRotation().getZ());

            swerveSubsystem.swerveDrive.addVisionMeasurement(new Pose2d(xPosition, yPosition, rotation), Timer.getFPGATimestamp());
        }
    }

    public PhotonCamera returnCamera() {
        return apriltagCamera;
    }
}
