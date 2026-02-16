package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pose Test")
public class PoseTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize mecanum drive with a starting pose of (0, 0, 0)
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Ready! Press play to start pose test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update the localizer's pose estimate
            PoseVelocity2d vel = drive.updatePoseEstimate();

            // Retrieve current estimated pose
            Pose2d pose = drive.localizer.getPose();

            // Display pose data on telemetry
            telemetry.addData("x (in)", pose.position.x);
            telemetry.addData("y (in)", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
