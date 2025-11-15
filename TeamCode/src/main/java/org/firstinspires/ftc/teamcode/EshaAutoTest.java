package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "EshaAutoTest")
public class EshaAutoTest extends LinearOpMode {

    // Motor power constants
    private static final double FORWARD_POWER = 0.5;
    private static final double STRAFE_POWER = 0.5;

    // Timing constants
    private static final long FORWARD_TIME_MS = 1800;  // 1.8 seconds
    private static final long STRAFE_LEFT_TIME_MS = 200;  // 200 milliseconds


    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Set motor directions (same as teleop)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU (same configuration as teleop)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // ========== INITIALIZATION TELEMETRY ==========
        telemetry.addLine("========================================");
        telemetry.addLine("ESHATEST AUTONOMOUS");
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Sequence:");
        telemetry.addLine("1. Move forward for 1.8 seconds");
        telemetry.addLine("2. Move left for 200 milliseconds");
        telemetry.addLine("========================================");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ========== STEP 1: MOVE FORWARD FOR 1.8 SECONDS ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Moving forward...");
        telemetry.addData("Duration", "1.8 seconds");
        telemetry.addLine("========================================");
        telemetry.update();

        // Set motor powers for forward movement
        // Forward: BR+, BL-, FR-, FL+
        backRightMotor.setPower(FORWARD_POWER);
        backLeftMotor.setPower(-FORWARD_POWER);
        frontRightMotor.setPower(-FORWARD_POWER);
        frontLeftMotor.setPower(FORWARD_POWER);

        // Move forward for 1.8 seconds
        sleep(FORWARD_TIME_MS);

        // Stop all motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        // Brief pause before strafing
        sleep(500);

        // ========== STEP 2: STRAFE LEFT FOR 200 MILLISECONDS ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Strafing left...");
        telemetry.addData("Duration", "200 milliseconds");
        telemetry.addLine("========================================");
        telemetry.update();

        // Record starting angle for telemetry
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Set motor powers for left strafe
        // Based on teleop: right joystick left (negative) maps to y axis
        // For left strafe: y is negative in the mecanum formula
        // Left strafe motor pattern: FL-, BL+, FR+, BR-
        frontLeftMotor.setPower(-STRAFE_POWER);
        backLeftMotor.setPower(STRAFE_POWER);
        frontRightMotor.setPower(STRAFE_POWER);
        backRightMotor.setPower(-STRAFE_POWER);

        // Strafe left for 200 milliseconds
        sleep(STRAFE_LEFT_TIME_MS);

        // Stop all motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        // ========== COMPLETION ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "COMPLETE ✓");
        telemetry.addLine("========================================");
        double finalAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Final Angle", "%.1f°", finalAngle);
        telemetry.addData("Total Rotation", "%.1f°", finalAngle - startAngle);
        telemetry.addLine("========================================");
        telemetry.addData("Forward Time", "%d ms", FORWARD_TIME_MS);
        telemetry.addData("Strafe Left Time", "%d ms", STRAFE_LEFT_TIME_MS);
        telemetry.update();
    }

    private void stopAllMotors(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor) {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
