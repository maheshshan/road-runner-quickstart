package org.firstinspires.ftc.teamcode.Test;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem classes and Road Runner 1.0 Action builders for Gate, Intake, and Shooter Flywheels.
 * Can be used cleanly in TeleOp and Autonomous trajectory action builders.
 */
public class BioBuzzSubsystems {

    // ==========================================
    // AXON GATE SERVO SUBSYSTEM
    // ==========================================
    public static class Gate {
        private final Servo axonGateServo;
        public static double OPEN_POSITION = 0.4;
        public static double CLOSED_POSITION = 0.95;

        public Gate(HardwareMap hardwareMap) {
            axonGateServo = hardwareMap.get(Servo.class, BioBuzzRoboCentric.GATE_SERVO);
            close(); // Default to closed on init
        }

        public void open() {
            axonGateServo.setPosition(OPEN_POSITION);
        }

        public void close() {
            axonGateServo.setPosition(CLOSED_POSITION);
        }

        public double getPosition() {
            return axonGateServo.getPosition();
        }

        // --- Road Runner 1.0 Actions for Autonomous ---
        public class OpenGateAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                open();
                packet.put("gateState", "OPEN");
                return false; // Action complete
            }
        }

        public class CloseGateAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                close();
                packet.put("gateState", "CLOSED");
                return false; // Action complete
            }
        }

        public Action openGate() {
            return new OpenGateAction();
        }

        public Action closeGate() {
            return new CloseGateAction();
        }
    }

    // ==========================================
    // INTAKE MOTOR SUBSYSTEM
    // ==========================================
    public static class Intake {
        private final DcMotorEx intake;
        public static double INTAKE_POWER = 1.0;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentric.INTAKE_NAME);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            stop();
        }

        public void setForward() {
            intake.setPower(INTAKE_POWER);
        }

        public void setReverse() {
            intake.setPower(-INTAKE_POWER);
        }

        public void stop() {
            intake.setPower(0.0);
        }

        public void setPower(double power) {
            intake.setPower(power);
        }

        // --- Road Runner 1.0 Actions for Autonomous ---
        public class IntakeForwardAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setForward();
                packet.put("intakeState", "FORWARD");
                return false;
            }
        }

        public class IntakeReverseAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setReverse();
                packet.put("intakeState", "REVERSE");
                return false;
            }
        }

        public class IntakeStopAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stop();
                packet.put("intakeState", "STOPPED");
                return false;
            }
        }

        public Action forward() {
            return new IntakeForwardAction();
        }

        public Action reverse() {
            return new IntakeReverseAction();
        }

        public Action stopIntake() {
            return new IntakeStopAction();
        }
    }

    // ==========================================
    // SHOOTER FLYWHEEL SUBSYSTEM
    // ==========================================
    public static class Shooter {
        private final DcMotorEx shooter1;
        private final DcMotorEx shooter2;

        public static double TARGET_RPM = 2700.0;
        public static double TICKS_PER_REV = 28.0;

        public Shooter(HardwareMap hardwareMap) {
            shooter1 = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentric.MOTOR_1_NAME);
            shooter2 = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentric.MOTOR_2_NAME);

            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter2.setDirection(BioBuzzRoboCentric.MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        public void spin() {
            double targetTPS = (TARGET_RPM * TICKS_PER_REV) / 60.0;
            shooter1.setVelocity(targetTPS);
            shooter2.setPower(shooter1.getPower());
        }

        public void stop() {
            shooter1.setVelocity(0.0);
            shooter2.setPower(0.0);
        }

        public double getRPM1() {
            return (shooter1.getVelocity() * 60.0) / TICKS_PER_REV;
        }

        public double getRPM2() {
            return (shooter2.getVelocity() * 60.0) / TICKS_PER_REV;
        }

        public class SpinFlywheelAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spin();
                packet.put("shooterState", "SPINNING");
                return false;
            }
        }

        public class StopFlywheelAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stop();
                packet.put("shooterState", "STOPPED");
                return false;
            }
        }

        public Action spinFlywheel() {
            return new SpinFlywheelAction();
        }

        public Action stopFlywheel() {
            return new StopFlywheelAction();
        }
    }
}
