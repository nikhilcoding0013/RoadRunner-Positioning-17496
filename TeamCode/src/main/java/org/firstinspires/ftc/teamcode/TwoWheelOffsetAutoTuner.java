package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "TwoWheel Offset Auto Tuner (Half Spin)")
public class TwoWheelOffsetAutoTuner extends LinearOpMode {

    // ===== TUNING PARAMETERS =====
    public static double K = 0.075;
    public static int ITERATIONS = 8;
    public static int SPINS_PER_ITERATION = 3;
    public static double SPIN_POWER = 0.375;

    // ===== INITIAL OFFSETS =====
    public static double forwardOffset = -5.8;
    public static double lateralOffset = 2.4;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize drive and localizer
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        if (drive.localizer instanceof TwoDeadWheelLocalizer) {
            ((TwoDeadWheelLocalizer) drive.localizer).setOffsets(forwardOffset, lateralOffset);
        }

        telemetry.addLine("Two-Wheel Offset Auto Tuner Ready");
        telemetry.addLine("Robot will spin automatically, half spin at a time");
        telemetry.update();

        waitForStart();

        for (int iter = 0; iter < ITERATIONS && opModeIsActive(); iter++) {

            double sumDeltaX = 0.0;
            double sumDeltaY = 0.0;

            for (int spinNum = 0; spinNum < SPINS_PER_ITERATION && opModeIsActive(); spinNum++) {

                // Stop robot
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightBack.setPower(0);
                sleep(200);

                // Reset pose
                drive.localizer.setPose(new Pose2d(0, 0, 0));

                // Start half spin
                double accumulatedHeading = 0.0;
                double lastHeading = drive.localizer.getPose().heading.toDouble();

                while (opModeIsActive() && Math.abs(accumulatedHeading) < Math.PI) {

                    // Spin motors
                    drive.leftFront.setPower(-SPIN_POWER);
                    drive.leftBack.setPower(-SPIN_POWER);
                    drive.rightFront.setPower(SPIN_POWER);
                    drive.rightBack.setPower(SPIN_POWER);

                    // Update pose
                    drive.updatePoseEstimate();
                    Pose2d pose = drive.localizer.getPose();
                    double heading = pose.heading.toDouble();

                    // Handle wraparound
                    double delta = heading - lastHeading;
                    if (delta > Math.PI) delta -= 2 * Math.PI;
                    if (delta < -Math.PI) delta += 2 * Math.PI;

                    accumulatedHeading += delta;
                    lastHeading = heading;

                    // Telemetry
                    telemetry.addData("Iteration", iter + 1);
                    telemetry.addData("Spin", spinNum + 1);
                    telemetry.addData("Accum Heading (deg)", Math.toDegrees(accumulatedHeading));
                    telemetry.addData("ΔX (in)", pose.position.x);
                    telemetry.addData("ΔY (in)", pose.position.y);
                    telemetry.update();

                    sleep(10);
                }

                // Stop robot after half spin
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightBack.setPower(0);
                sleep(150);

                // Record drift for this half-spin
                Pose2d finalPose = drive.localizer.getPose();
                sumDeltaX += finalPose.position.x;
                sumDeltaY += finalPose.position.y;
            }

            // Average over half spins
            double avgDeltaX = sumDeltaX / SPINS_PER_ITERATION;
            double avgDeltaY = sumDeltaY / SPINS_PER_ITERATION;

            // Update offsets (still using K, can adjust later)
            forwardOffset -= K * avgDeltaY;
            lateralOffset -= K * avgDeltaX;

            // Apply new offsets
            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                ((TwoDeadWheelLocalizer) drive.localizer).setOffsets(forwardOffset, lateralOffset);
            }

            telemetry.addLine("---- Iteration Complete ----");
            telemetry.addData("Avg ΔX", avgDeltaX);
            telemetry.addData("Avg ΔY", avgDeltaY);
            telemetry.addData("forwardOffset", forwardOffset);
            telemetry.addData("lateralOffset", lateralOffset);
            telemetry.update();

            sleep(500);
        }

        telemetry.addLine("TUNING COMPLETE");
        telemetry.addData("FINAL forwardOffset", forwardOffset);
        telemetry.addData("FINAL lateralOffset", lateralOffset);
        telemetry.update();

        while (opModeIsActive()) sleep(50);
    }
}
