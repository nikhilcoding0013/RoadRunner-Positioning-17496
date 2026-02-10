package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "TwoWheel Offset Auto Tuner")
public class TwoWheelOffsetAutoTuner extends LinearOpMode {

    // Tuning parameters
    public static double K = 0.075; // adjustment multiplier
    public static int ITERATIONS = 10;
    public static int SPINS_PER_ITERATION = 3;
    public static double SPIN_POWER = 0.1; // slow constant rotation power

    // Starting offsets (load from last run or approximate)
    public static double forwardOffset = -5.8;
    public static double lateralOffset = -2.4;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize drive
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap);

        telemetry.addLine("Starting Two-Wheel Auto Tuning...");
        telemetry.update();

        waitForStart();

        for (int iter = 0; iter < ITERATIONS && opModeIsActive(); iter++) {

            double sumDeltaX = 0;
            double sumDeltaY = 0;

            for (int spin = 0; spin < SPINS_PER_ITERATION && opModeIsActive(); spin++) {

                // Reset pose to zero for accurate delta measurement
                localizer.setPose(new Pose2d());

                // Spin in place slowly
                double spinTime = 0.0;
                double spinDuration = 4000; // milliseconds per spin (adjust for your robot)
                long startTime = System.currentTimeMillis();

                while (System.currentTimeMillis() - startTime < spinDuration && opModeIsActive()) {
                    // Apply slow turn
                    localizer.setDrivePower(new Pose2d(0, 0, SPIN_POWER));

                    // Update localizer
                    localizer.update();

                    // Small sleep to avoid busy loop
                    sleep(10);
                }

                // Stop robot
                localizer.setDrivePower(new Pose2d());

                // Measure drift
                Pose2d pose = localizer.getPose();
                sumDeltaX += pose.getX();
                sumDeltaY += pose.getY();

                telemetry.addData("Iteration", iter + 1);
                telemetry.addData("Spin", spin + 1);
                telemetry.addData("ΔX this spin", pose.getX());
                telemetry.addData("ΔY this spin", pose.getY());
                telemetry.update();

                sleep(250); // brief pause between spins
            }

            // Average drift over spins
            double avgDeltaX = sumDeltaX / SPINS_PER_ITERATION;
            double avgDeltaY = sumDeltaY / SPINS_PER_ITERATION;

            // Update offsets
            forwardOffset -= K * avgDeltaX;
            lateralOffset -= K * avgDeltaY;

            telemetry.addData("Iteration", iter + 1);
            telemetry.addData("Avg ΔX", avgDeltaX);
            telemetry.addData("Avg ΔY", avgDeltaY);
            telemetry.addData("Updated forwardOffset", forwardOffset);
            telemetry.addData("Updated lateralOffset", lateralOffset);
            telemetry.update();
        }

        // Final results
        telemetry.addLine("Tuning Complete");
        telemetry.addData("Final forwardOffset", forwardOffset);
        telemetry.addData("Final lateralOffset", lateralOffset);
        telemetry.update();

        // Keep program running so you can read dashboard
        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
