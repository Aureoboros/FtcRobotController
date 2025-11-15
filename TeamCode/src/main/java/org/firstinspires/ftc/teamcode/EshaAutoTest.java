package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "EshaTest")
public class EshaTest extends LinearOpMode {
    
    // Motor power constants
    private static final double FORWARD_POWER = 0.5;
    private static final double TURN_POWER = 0.4;
    private static final double ANGLE_TOLERANCE = 5.0;  // degrees
    
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
        telemetry.addLine("1. Move forward for 2 seconds");
        telemetry.addLine("2. Turn 180 degrees");
        telemetry.addLine("========================================");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ========== STEP 1: MOVE FORWARD FOR 2 SECONDS ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Moving forward...");
        telemetry.addLine("========================================");
        telemetry.update();
        
        // Set motor powers for forward movement
        // Forward: BR+, BL-, FR-, FL+
        backRightMotor.setPower(FORWARD_POWER);
        backLeftMotor.setPower(-FORWARD_POWER);
        frontRightMotor.setPower(-FORWARD_POWER);
        frontLeftMotor.setPower(FORWARD_POWER);
        
        // Move forward for 2 seconds
        sleep(2000);
        
        // Stop all motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        
        // Brief pause before turning
        sleep(500);

        // ========== STEP 2: TURN 180 DEGREES USING GYRO ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "Turning 180 degrees...");
        telemetry.addLine("========================================");
        telemetry.update();
        
        // Reset gyro to establish new zero point
        imu.resetYaw();
        sleep(100);  // Brief delay to ensure reset completes
        
        // Target is 180 degrees from current position (which is now 0)
        double targetAngle = 180.0;
        
        // Gyro-controlled turn with proportional control
        while (opModeIsActive()) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double angleError = targetAngle - currentAngle;
            
            // Normalize angle error to [-180, 180]
            while (angleError > 180) angleError -= 360;
            while (angleError < -180) angleError += 360;
            
            // Update telemetry
            telemetry.addData("Current Angle", "%.1f°", currentAngle);
            telemetry.addData("Target Angle", "%.1f°", targetAngle);
            telemetry.addData("Error", "%.1f°", angleError);
            telemetry.update();
            
            // Check if we're close enough to target
            if (Math.abs(angleError) < ANGLE_TOLERANCE) {
                break;
            }
            
            // Proportional control: slow down as we approach target
            double turnSpeed = TURN_POWER;
            if (Math.abs(angleError) < 30) {
                // Reduce speed when within 30 degrees
                turnSpeed = TURN_POWER * (Math.abs(angleError) / 30.0);
                // Ensure minimum speed to overcome friction
                turnSpeed = Math.max(turnSpeed, 0.15);
            }
            
            // Turn direction based on error (positive error = turn clockwise)
            if (angleError > 0) {
                // Turn clockwise: left side forward, right side backward
                frontLeftMotor.setPower(-turnSpeed);   // Forward
                backLeftMotor.setPower(turnSpeed);     // Forward
                frontRightMotor.setPower(-turnSpeed);  // Backward
                backRightMotor.setPower(turnSpeed);    // Backward
            } else {
                // Turn counter-clockwise: left side backward, right side forward
                frontLeftMotor.setPower(turnSpeed);    // Backward
                backLeftMotor.setPower(-turnSpeed);    // Backward
                frontRightMotor.setPower(turnSpeed);   // Forward
                backRightMotor.setPower(-turnSpeed);   // Forward
            }
        }
        
        // Stop all motors
        stopAllMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        // ========== COMPLETION ==========
        telemetry.addLine("========================================");
        telemetry.addData("Status", "COMPLETE ✓");
        telemetry.addLine("========================================");
        double finalAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Final Angle", "%.1f°", finalAngle);
        telemetry.addData("Turn Accuracy", "%.1f° from target", Math.abs(180.0 - finalAngle));
        telemetry.update();
    }
    
    private void stopAllMotors(DcMotor fl, DcMotor bl, DcMotor fr, DcMotor br) {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
}
