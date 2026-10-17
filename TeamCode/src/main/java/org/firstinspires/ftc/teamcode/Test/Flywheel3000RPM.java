package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FlywheelConstantVelocity")
public class Flywheel3000RPM extends LinearOpMode {

    DcMotorEx flywheel;

    @Override
    public void runOpMode() throws InterruptedException {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // GoBilda 5203 motor → 28 ticks per revolution
        double ticksPerRev = 28.0;

        // Target RPM
        double targetRPM = 3000;

        // Convert RPM → ticks per second (TPS)
        double targetTPS = (targetRPM * ticksPerRev) / 60.0;

        waitForStart();

        while (opModeIsActive()) {

            // Set constant velocity using encoder feedback
            flywheel.setVelocity(targetTPS);

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Current Velocity (TPS)", flywheel.getVelocity());
            telemetry.update();
        }
    }
}
