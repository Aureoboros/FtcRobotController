package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutoRedBackStart", group = "FTC2025")
public class AutoRedBackStart  extends LinearOpMode {

    // Launch motor power constant
    private static final double LAUNCH_MOTOR_POWER = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== HARDWARE SETUP =====
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        // Launch motors
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launchMotor = hardwareMap.dcMotor.get("launchMotor");
        DcMotor rampMotor = hardwareMap.dcMotor.get("rampMotor");

        // Set motor directions (matching SVTestTeleop12nov.java - all FORWARD)
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

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // ===== WAIT FOR START =====
        waitForStart();

        if (opModeIsActive()) {

            // === MOVE FORWARD 6 FT ===
            telemetry.addLine("Moving forward 6 ft...");
            telemetry.update();

            // Use mecanum drive formula from SVTestTeleop12nov.java
            // For forward movement: x = -power (negative), y = 0 (no strafe), rx = 0 (no rotation)
            // Formula: frontLeftPower = -y + x + rx
            //          backLeftPower = -y - x + rx
            //          frontRightPower = -y - x - rx
            //          backRightPower = -y + x - rx
            double forwardPower = 0.5;
            double x = -forwardPower;  // Negative for forward (matching joystick up = negative)
            double y = 0.0;            // No strafe
            double rx = 0.0;           // No rotation
            
            double frontLeftPower = -y + x + rx;
            double backLeftPower = -y - x + rx;
            double frontRightPower = -y - x - rx;
            double backRightPower = -y + x - rx;
            
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            sleep(1900);   // ~6 ft forward (estimated from 1500ms for 2 ft)
            
            // Stop all drive motors
            stopDriveMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
            sleep(200);

            // === ROTATE RIGHT 45 DEGREES (using RT rotation pattern) ===
            telemetry.addLine("Rotating right 45 degrees...");
            telemetry.update();

            // Use RT rotation pattern from SVTestTeleop12nov.java (lines 968-976)
            // Pattern from SVTestTeleop12nov.java lines 968-976:
            // Front left: forward, Back left: forward, Front right: backward, Back right: backward
            // frontLeftMotor: -rtPower (forward, negative)
            // backLeftMotor: rtPower (forward, positive)
            // frontRightMotor: -rtPower (backward, negative)
            // backRightMotor: rtPower (backward, positive)
            // For right rotation (clockwise), we use RT pattern
            double rtPower = 0.5;  // Rotation power (limited to max 0.5 like in teleop)
            
            frontLeftMotor.setPower(-rtPower);  // Forward (negative = forward)
            backLeftMotor.setPower(rtPower);  // Forward (positive = forward)
            frontRightMotor.setPower(-rtPower);  // Backward (negative = backward)
            backRightMotor.setPower(rtPower);  // Backward (positive = backward)
            sleep(300);   // Rotate for 0.3 seconds (adjust if needed for 45 degrees)
            
            // Stop all drive motors
            stopDriveMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
            sleep(200);

            // === MOVE BACKWARD FOR 0.5 SECONDS ===
            telemetry.addLine("Moving backward for 0.5 seconds...");
            telemetry.update();

            // Use mecanum drive formula from SVTestTeleop12nov.java
            // For backward movement: x = +power (positive), y = 0 (no strafe), rx = 0 (no rotation)
            double backwardPower1 = 0.5;
            double xBack1 = backwardPower1;  // Positive for backward
            double yBack1 = 0.0;             // No strafe
            double rxBack1 = 0.0;            // No rotation
            
            double frontLeftPowerBack1 = -yBack1 + xBack1 + rxBack1;
            double backLeftPowerBack1 = -yBack1 - xBack1 + rxBack1;
            double frontRightPowerBack1 = -yBack1 - xBack1 - rxBack1;
            double backRightPowerBack1 = -yBack1 + xBack1 - rxBack1;
            
            frontLeftMotor.setPower(frontLeftPowerBack1);
            backLeftMotor.setPower(backLeftPowerBack1);
            frontRightMotor.setPower(frontRightPowerBack1);
            backRightMotor.setPower(backRightPowerBack1);
            sleep(500);   // 0.5 seconds
            
            // Stop all drive motors
            stopDriveMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
            sleep(200);

            // === LAUNCH (X BUTTON SEQUENCE) ===
            telemetry.addLine("Launching...");
            telemetry.update();

            // Stop intake and ramp
            intakeMotor.setPower(0);
            rampMotor.setPower(0);

            // Start launch motor
            launchMotor.setPower(LAUNCH_MOTOR_POWER);
            sleep(1000);  // Wait for launch motor to spin up

            // Start feeding
            intakeMotor.setPower(1.0);
            rampMotor.setPower(-0.4);  // Reverse power during launch (40% speed)
            sleep(2500);  // Feed for 2.5 seconds

            // Stop launch motors
            launchMotor.setPower(0);
            intakeMotor.setPower(0);
            rampMotor.setPower(0);
            sleep(200);

            // === MOVE BACKWARD FOR 1 SECOND ===
            telemetry.addLine("Moving backward for 1 second...");
            telemetry.update();

            // Use mecanum drive formula from SVTestTeleop12nov.java
            // For backward movement: x = +power (positive), y = 0 (no strafe), rx = 0 (no rotation)
            double backwardPower2 = 0.5;
            double xBack2 = backwardPower2;  // Positive for backward
            double yBack2 = 0.0;             // No strafe
            double rxBack2 = 0.0;            // No rotation
            
            double frontLeftPowerBack2 = -yBack2 + xBack2 + rxBack2;
            double backLeftPowerBack2 = -yBack2 - xBack2 + rxBack2;
            double frontRightPowerBack2 = -yBack2 - xBack2 - rxBack2;
            double backRightPowerBack2 = -yBack2 + xBack2 - rxBack2;
            
            frontLeftMotor.setPower(frontLeftPowerBack2);
            backLeftMotor.setPower(backLeftPowerBack2);
            frontRightMotor.setPower(frontRightPowerBack2);
            backRightMotor.setPower(backRightPowerBack2);
            sleep(1000);   // 1 second
            
            // Stop all motors explicitly
            stopDriveMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
            sleep(200);

            // === END ===
            telemetry.addLine("Done!");
            telemetry.update();
            sleep(1000);
        }
    }

    private void stopDriveMotors(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
