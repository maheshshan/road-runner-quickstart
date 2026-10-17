package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "BioBuzz Auto Sample (Actions)", group = "Test")
public class BioBuzzAutoSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Road Runner Mecanum Drive at start pose (0, 0, heading 0)
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Initialize Subsystems
        BioBuzzSubsystems.Gate gate = new BioBuzzSubsystems.Gate(hardwareMap);
        BioBuzzSubsystems.Intake intake = new BioBuzzSubsystems.Intake(hardwareMap);
        BioBuzzSubsystems.Shooter shooter = new BioBuzzSubsystems.Shooter(hardwareMap);

        telemetry.addData("Status", "Initialized Auto with Subsystem Actions");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Build Road Runner Trajectory Actions with Gate, Intake, & Shooter Actions
        Actions.runBlocking(
            new SequentialAction(
                // 1. Spin up shooter flywheel
                shooter.spinFlywheel(),

                // 2. Drive forward to shooting position
                drive.actionBuilder(startPose)
                    .lineToX(24.0)
                    .build(),

                // 3. Open Gate to shoot
                gate.openGate(),

                // 4. Drive while intaking new element
                drive.actionBuilder(new Pose2d(24.0, 0, 0))
                    .stopAndAdd(intake.forward())
                    .strafeTo(new Vector2d(24.0, 12.0))
                    .stopAndAdd(intake.stopIntake())
                    .build(),

                // 5. Close Gate
                gate.closeGate(),

                // 6. Stop Flywheels at end of Auto
                shooter.stopFlywheel()
            )
        );
    }
}
