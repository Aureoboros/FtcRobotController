package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "EshaAutoTest")
public class AutoRedAndBlueFrontStart extends LinearOpMode {

    // Motor power constants
    private static final double FORWARD_POWER = 0.5;
    private static final double STRAFE_POWER = 0.5;
    private static final double ROTATION_POWER = 0.5;  // Rotation power for left/right rotation (LT/RT pattern)
    private static final double LAUNCH_MOTOR_POWER = 0.75;  // Launch motor power for shooting
    private static final double INTAKE_POWER = 1.0;  // Intake motor power during launch
    private static final double RAMP_POWER = -0.4;  // Ramp motor power during launch (reverse, 40% speed)

    // Timing constants
    private static final long FORWARD_TIME_MS = 1700;  // 1.7 seconds
    private static final long STRAFE_LEFT_TIME_MS = 255;  // 255 milliseconds
    private static final long LAUNCH_SPINUP_TIME_MS = 1000;  // 1 second for launch motor to spin up
    private static final long LAUNCH_FEED_TIME_MS = 2500;  // 2.5 seconds for feeding during launch
    private static final long ROTATE_LEFT_TIME_MS = 100;  // 100 milliseconds for left rotation after launch
    private static final long BACKWARD_TIME_MS = 1000;  // 1 second for backward movement


    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launchMotor = hardwareMap.dcMotor.get("launchMotor");
        DcMotor rampMotor = hardwareMap.dcMotor.get("rampMotor");

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
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor modes for mechanisms
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        telemetry.addLine("1. Move forward for 1.7 seconds");
        telemetry.addLine("2. Move left for 255 milliseconds");
        telemetry.addLine("3. Launch sequence (X button equivalent)");
        telemetry.addLine("4. Rotate left for 100 milliseconds (LT pattern)");
        telemetry.addLine("5. Move backward for 1 second (joystick down)");
        telemetry.addLine("========================================");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ========== STEP 1: MOVE FORWARD FOR 1.7 SECONDS ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Moving forward...");
        telemetry.addData("Duration", "1.7 seconds");
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

        // ========== STEP 2: STRAFE LEFT FOR 255 MILLISECONDS ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Strafing left...");
        telemetry.addData("Duration", "255 milliseconds");
        telemetry.addLine("========================================");
        telemetry.update();

        // Record starting angle for telemetry
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Set motor powers for left strafe
        // Based on teleop: right joystick left (negative) maps to y axis
        // For left strafe: y is negative in the mecanum formula
        // Formula: frontLeftPower = -y + x + rx, backLeftPower = -y - x + rx,
        //          frontRightPower = -y - x - rx, backRightPower = -y + x - rx
        // When y is negative (left), -y is positive, so all motors get positive power
        // Left strafe motor pattern: FL+, BL+, FR+, BR+
        frontLeftMotor.setPower(STRAFE_POWER);
        backLeftMotor.setPower(STRAFE_POWER);
        frontRightMotor.setPower(STRAFE_POWER);
        backRightMotor.setPower(STRAFE_POWER);

        // Strafe left for 255 milliseconds
        sleep(STRAFE_LEFT_TIME_MS);

        // Stop all motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        // ========== STEP 3: LAUNCH SEQUENCE (X BUTTON EQUIVALENT) ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Starting launch sequence...");
        telemetry.addLine("========================================");
        telemetry.update();

        // Stop intake and ramp (ensure they're off)
        intakeMotor.setPower(0);
        rampMotor.setPower(0);

        // Lock wheels during launch to prevent movement
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        // Start launch motor
        launchMotor.setPower(LAUNCH_MOTOR_POWER);
        telemetry.addData("Launch Motor", "Spinning up...");
        telemetry.update();
        sleep(LAUNCH_SPINUP_TIME_MS);  // Wait for launch motor to spin up

        // Start feeding
        intakeMotor.setPower(INTAKE_POWER);
        rampMotor.setPower(RAMP_POWER);  // Reverse power during launch (40% speed)
        telemetry.addData("Launch Motor", "Running at %.2f", LAUNCH_MOTOR_POWER);
        telemetry.addData("Intake", "Feeding at %.2f", INTAKE_POWER);
        telemetry.addData("Ramp", "Running at %.2f", RAMP_POWER);
        telemetry.update();
        sleep(LAUNCH_FEED_TIME_MS);  // Feed for 2.5 seconds

        // Stop launch motors
        launchMotor.setPower(0);
        intakeMotor.setPower(0);
        rampMotor.setPower(0);
        sleep(200);

        // ========== STEP 4: ROTATE LEFT (LT PATTERN) ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Rotating left...");
        telemetry.addData("Duration", "100 milliseconds");
        telemetry.addLine("========================================");
        telemetry.update();

        // Use LT (Left Trigger) rotation pattern from SVTestTeleop12nov.java
        // Pattern: Front left: backward, Back left: backward, Front right: forward, Back right: forward
        // frontLeftMotor: +power (backward, positive = backward)
        // backLeftMotor: -power (backward, negative = backward)
        // frontRightMotor: +power (forward, positive = forward)
        // backRightMotor: -power (forward, negative = forward)
        frontLeftMotor.setPower(ROTATION_POWER);  // Backward (positive = backward)
        backLeftMotor.setPower(-ROTATION_POWER);  // Backward (negative = backward)
        frontRightMotor.setPower(ROTATION_POWER);  // Forward (positive = forward)
        backRightMotor.setPower(-ROTATION_POWER);  // Forward (negative = forward)
        sleep(ROTATE_LEFT_TIME_MS);  // Rotate left for 100 milliseconds

        // Stop all drive motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        sleep(200);

        // ========== STEP 5: MOVE BACKWARD (JOYSTICK DOWN) ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Moving backward...");
        telemetry.addData("Duration", "1 second");
        telemetry.addLine("========================================");
        telemetry.update();

        // Use mecanum drive formula from SVTestTeleop12nov.java for backward movement
        // For backward: x = +power (positive), y = 0 (no strafe), rx = 0 (no rotation)
        // Formula: frontLeftPower = -y + x + rx, backLeftPower = -y - x + rx,
        //          frontRightPower = -y - x - rx, backRightPower = -y + x - rx
        double backwardPower = FORWARD_POWER;
        double xBack = backwardPower;  // Positive for backward (joystick down = positive Y = positive x)
        double yBack = 0.0;            // No strafe
        double rxBack = 0.0;           // No rotation
        
        double frontLeftPowerBack = -yBack + xBack + rxBack;
        double backLeftPowerBack = -yBack - xBack + rxBack;
        double frontRightPowerBack = -yBack - xBack - rxBack;
        double backRightPowerBack = -yBack + xBack - rxBack;
        
        frontLeftMotor.setPower(frontLeftPowerBack);
        backLeftMotor.setPower(backLeftPowerBack);
        frontRightMotor.setPower(frontRightPowerBack);
        backRightMotor.setPower(backRightPowerBack);
        sleep(BACKWARD_TIME_MS);  // Move backward for 1 second

        // Stop all drive motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        sleep(200);

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
