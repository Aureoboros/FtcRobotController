package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name="AutoFR-Blue", group="Autonomous")
public class AutoTwoSpikeMarksBlue extends LinearOpMode {
    
    // Motor power constants
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCH_MOTOR_POWER = 0.6;  // Reduced for better accuracy
    private static final double RAMP_MOTOR_POWER = 1.0;
    
    // Navigation constants
    private static final double AUTO_MAX_SPEED = 0.7;
    private static final double AUTO_MIN_SPEED = 0.15;
    private static final double SLOWDOWN_DISTANCE_FEET = 2.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.5;
    
    // AprilTag alignment constants
    private static final double APRILTAG_ALIGNMENT_TOLERANCE = 3.0;  // degrees
    private static final double APRILTAG_ROTATION_POWER = 0.25;
    private static final int TARGET_APRILTAG_ID = 20;  // Blue alliance goal
    
    // Dead wheel odometry constants
    private static final double ODO_COUNTS_PER_REV = 8192.0;
    private static final double ODO_WHEEL_DIAMETER_INCHES = 1.26;
    private static final double ODO_COUNTS_PER_INCH = ODO_COUNTS_PER_REV / (ODO_WHEEL_DIAMETER_INCHES * Math.PI);
    
    // Servo positions
    private static final double GATE_SERVO_DOWN = 0.0;
    private static final double GATE_SERVO_UP = 0.417;
    
    // AprilTag IDs
    private static final int[] OBELISK_APRILTAG_IDS = {21, 22, 23};  // Obelisk tags for positioning
    
    // Field positions (in feet, measured from field center)
    // Blue alliance spike marks (right side of field, mirrored from red)
    private static final double BLUE_MIDDLE_SPIKE_X = 3.0;  // Middle spike mark (mirrored)
    private static final double BLUE_MIDDLE_SPIKE_Y = -1.0;  // Below center line
    
    private static final double BLUE_TOP_SPIKE_X = 3.0;     // Top spike mark (same X)
    private static final double BLUE_TOP_SPIKE_Y = 1.0;      // Above center line
    
    // Shooting position (at field center for AprilTag alignment)
    private static final double BLUE_SHOOT_X = 0.0;
    private static final double BLUE_SHOOT_Y = 0.0;
    
    // Park position (blue observation zone - mirrored)
    private static final double BLUE_PARK_X = 3.0;
    private static final double BLUE_PARK_Y = -4.0;
    
    // Odometry tracking (will be set based on actual start position)
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;
    private int lastForwardOdoPos = 0;
    private int lastRightOdoPos = 0;
    
    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launchMotor1, launchMotor2, rampMotor;
    private DcMotor forwardOdo, rightOdo;
    private Servo gateServo;
    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();
        
        telemetry.addLine("========================================");
        telemetry.addLine("BLUE ALLIANCE: 2 SPIKE MARKS AUTO");
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addLine("INITIALIZING...");
        telemetry.addLine("Detecting Obelisk AprilTags for position...");
        telemetry.addLine("========================================");
        telemetry.update();
        
        // Wait for camera to initialize
        sleep(1000);
        
        // Detect starting position using Obelisk AprilTags
        boolean positionDetected = false;
        int detectionAttempts = 0;
        int maxAttempts = 50;  // 5 seconds of attempts
        
        while (!isStarted() && !isStopRequested() && !positionDetected && detectionAttempts < maxAttempts) {
            AprilTagDetection obeliskTag = getAnyObeliskTag();
            
            if (obeliskTag != null) {
                // Calculate robot position based on AprilTag detection
                calculatePositionFromObelisk(obeliskTag);
                positionDetected = true;
                
                telemetry.addLine("========================================");
                telemetry.addLine("✓ POSITION DETECTED!");
                telemetry.addLine("========================================");
                telemetry.addData("Obelisk Tag", "ID %d detected", obeliskTag.id);
                telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotHeading));
            } else {
                telemetry.addLine("========================================");
                telemetry.addLine("DETECTING POSITION...");
                telemetry.addLine("========================================");
                telemetry.addData("Searching for Obelisk Tags", "21, 22, 23");
                telemetry.addData("Attempt", "%d / %d", detectionAttempts + 1, maxAttempts);
                telemetry.addLine();
                telemetry.addLine("Make sure camera can see obelisk!");
            }
            
            telemetry.addLine();
            telemetry.addLine("Target Sequence:");
            telemetry.addData("  1. Middle Spike", "X: %.1f, Y: %.1f", BLUE_MIDDLE_SPIKE_X, BLUE_MIDDLE_SPIKE_Y);
            telemetry.addData("  2. Shoot Position", "X: %.1f, Y: %.1f", BLUE_SHOOT_X, BLUE_SHOOT_Y);
            telemetry.addData("  3. Top Spike", "X: %.1f, Y: %.1f", BLUE_TOP_SPIKE_X, BLUE_TOP_SPIKE_Y);
            telemetry.addData("  4. Shoot Position", "X: %.1f, Y: %.1f", BLUE_SHOOT_X, BLUE_SHOOT_Y);
            telemetry.addData("  5. Park", "X: %.1f, Y: %.1f", BLUE_PARK_X, BLUE_PARK_Y);
            telemetry.addLine("========================================");
            telemetry.update();
            
            detectionAttempts++;
            sleep(100);
        }
        
        // Fallback if no tag detected
        if (!positionDetected) {
            telemetry.addLine("⚠ WARNING: Could not detect position!");
            telemetry.addLine("Using default start position (0, 0)");
            telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
            telemetry.addLine("Press START to continue anyway");
            telemetry.update();
        }
        
        waitForStart();
        if (isStopRequested()) return;
        
        // Start ramp motor
        rampMotor.setPower(RAMP_MOTOR_POWER);
        
        // ========== SEQUENCE START ==========
        
        // 1. Drive to middle spike mark
        telemetry.addData("Step 1", "Driving to Middle Spike");
        telemetry.update();
        driveToPosition(BLUE_MIDDLE_SPIKE_X, BLUE_MIDDLE_SPIKE_Y, 0);
        
        // 2. Intake balls from middle spike
        telemetry.addData("Step 2", "Intaking Middle Spike");
        telemetry.update();
        intakeBalls(3.5);  // 3.5 seconds to intake
        
        // 3. Drive to shooting position
        telemetry.addData("Step 3", "Moving to Shoot Position");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        
        // 4. Align with AprilTag
        telemetry.addData("Step 4", "Aligning with Goal");
        telemetry.update();
        alignWithAprilTag(TARGET_APRILTAG_ID, 3.0);  // 3 second timeout
        
        // 5. Shoot balls
        telemetry.addData("Step 5", "Shooting Middle Spike Balls");
        telemetry.update();
        shootBalls(2.5);  // 2.5 seconds to shoot
        
        // 6. Drive to top spike mark
        telemetry.addData("Step 6", "Driving to Top Spike");
        telemetry.update();
        driveToPosition(BLUE_TOP_SPIKE_X, BLUE_TOP_SPIKE_Y, 0);
        
        // 7. Intake balls from top spike
        telemetry.addData("Step 7", "Intaking Top Spike");
        telemetry.update();
        intakeBalls(3.5);
        
        // 8. Drive back to shooting position
        telemetry.addData("Step 8", "Moving to Shoot Position");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        
        // 9. Align with AprilTag again
        telemetry.addData("Step 9", "Aligning with Goal");
        telemetry.update();
        alignWithAprilTag(TARGET_APRILTAG_ID, 3.0);
        
        // 10. Shoot balls
        telemetry.addData("Step 10", "Shooting Top Spike Balls");
        telemetry.update();
        shootBalls(2.5);
        
        // 11. Park near observation zone
        telemetry.addData("Step 11", "Parking");
        telemetry.update();
        driveToPosition(BLUE_PARK_X, BLUE_PARK_Y, 0);
        
        // Stop all motors
        stopAllMotors();
        
        telemetry.addLine("========================================");
        telemetry.addLine("AUTONOMOUS COMPLETE!");
        telemetry.addLine("========================================");
        telemetry.update();
        
        // Keep vision running for inspection
        sleep(2000);
        visionPortal.close();
    }
    
    private void initializeHardware() {
        // Drive motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        // Mechanism motors
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        launchMotor1 = hardwareMap.dcMotor.get("launchMotor1");
        launchMotor2 = hardwareMap.dcMotor.get("launchMotor2");
        rampMotor = hardwareMap.dcMotor.get("rampMotor");
        
        // Set motor modes
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Servo
        gateServo = hardwareMap.servo.get("GateServo");
        gateServo.setPosition(GATE_SERVO_DOWN);
        
        // Dead wheel encoders
        forwardOdo = hardwareMap.dcMotor.get("forwardOdo");
        rightOdo = hardwareMap.dcMotor.get("rightOdo");
        
        forwardOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set brake mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reverse right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        
        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        
        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(
                    org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, 
                    "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    
    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        double startTime = getRuntime();
        double timeout = 5.0;  // 5 second timeout per movement
        
        while (opModeIsActive() && (getRuntime() - startTime) < timeout) {
            updateOdometry();
            
            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;
            double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // Check if reached target
            if (distanceToTarget * 12 < POSITION_TOLERANCE_INCHES) {
                stopDriveMotors();
                break;
            }
            
            // Calculate angle to target
            double angleToTarget = Math.atan2(deltaX, deltaY);
            
            // Speed control with slowdown
            double speed = AUTO_MAX_SPEED;
            if (distanceToTarget < SLOWDOWN_DISTANCE_FEET) {
                double slowdownRatio = distanceToTarget / SLOWDOWN_DISTANCE_FEET;
                speed = AUTO_MIN_SPEED + (AUTO_MAX_SPEED - AUTO_MIN_SPEED) * slowdownRatio;
            }
            
            // Convert to robot frame
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x = Math.sin(angleToTarget - heading) * speed;
            double y = Math.cos(angleToTarget - heading) * speed;
            
            // Calculate heading correction
            double headingError = targetHeading - heading;
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            double rx = headingError * 0.5;  // Proportional heading control
            
            // Drive
            driveFieldCentric(x, y, rx, heading);
            
            // Telemetry
            telemetry.addData("Target", "X: %.2f, Y: %.2f", targetX, targetY);
            telemetry.addData("Current", "X: %.2f, Y: %.2f", robotX, robotY);
            telemetry.addData("Distance", "%.2f ft", distanceToTarget);
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.update();
        }
        
        stopDriveMotors();
        sleep(100);  // Brief settle time
    }
    
    private void alignWithAprilTag(int targetTagId, double timeout) {
        double startTime = getRuntime();
        boolean aligned = false;
        
        while (opModeIsActive() && (getRuntime() - startTime) < timeout && !aligned) {
            updateOdometry();
            
            AprilTagDetection detection = getAprilTagDetection(targetTagId);
            
            if (detection != null) {
                double yawError = detection.ftcPose.yaw;  // Positive = tag is to the right
                double bearingError = detection.ftcPose.bearing;  // Horizontal angle to target
                double range = detection.ftcPose.range;  // Distance in inches
                
                telemetry.addData("AprilTag", "ID %d DETECTED", targetTagId);
                telemetry.addData("Yaw Error", "%.2f degrees", yawError);
                telemetry.addData("Bearing", "%.2f degrees", bearingError);
                telemetry.addData("Range", "%.1f inches", range);
                
                // Check if aligned (both yaw and bearing within tolerance)
                if (Math.abs(yawError) < APRILTAG_ALIGNMENT_TOLERANCE && 
                    Math.abs(bearingError) < APRILTAG_ALIGNMENT_TOLERANCE) {
                    aligned = true;
                    telemetry.addData("Status", "✓ ALIGNED!");
                } else {
                    // Use bearing for alignment (like in teleop navigateToAlliance)
                    // Bearing tells us which direction to rotate to face the tag
                    double rotationPower = Math.signum(yawError) * APRILTAG_ROTATION_POWER;
                    
                    // If bearing is large, also adjust position slightly
                    double strafeCorrection = 0;
                    if (Math.abs(bearingError) > 5.0) {
                        strafeCorrection = -Math.signum(bearingError) * 0.15;  // Strafe to center
                    }
                    
                    driveFieldCentric(strafeCorrection, 0, rotationPower, robotHeading);
                    telemetry.addData("Status", "Aligning... (Rotate: %.2f, Strafe: %.2f)", 
                                    rotationPower, strafeCorrection);
                }
            } else {
                telemetry.addData("AprilTag", "NOT DETECTED");
                telemetry.addData("Status", "Searching...");
                // Gentle rotation to find tag (like in teleop)
                driveFieldCentric(0, 0, 0.15, robotHeading);
            }
            
            telemetry.update();
        }
        
        stopDriveMotors();
        
        if (aligned) {
            telemetry.addData("Alignment", "✓ SUCCESS - Dead on target!");
        } else {
            telemetry.addData("Alignment", "TIMEOUT - Proceeding anyway");
        }
        telemetry.update();
        sleep(100);
    }
    
    private void intakeBalls(double duration) {
        intakeMotor.setPower(INTAKE_POWER);
        sleep((long)(duration * 1000));
        intakeMotor.setPower(0);
    }
    
    private void shootBalls(double duration) {
        // Spin up launch motors
        launchMotor1.setPower(LAUNCH_MOTOR_POWER);
        launchMotor2.setPower(-LAUNCH_MOTOR_POWER);
        sleep(500);  // Spin-up time
        
        // Open gate to release balls
        gateServo.setPosition(GATE_SERVO_UP);
        sleep((long)(duration * 1000));
        
        // Close gate and stop launch motors
        gateServo.setPosition(GATE_SERVO_DOWN);
        launchMotor1.setPower(0);
        launchMotor2.setPower(0);
        sleep(200);  // Brief settle
    }
    
    private void driveFieldCentric(double x, double y, double rx, double botHeading) {
        // Field-centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        
        rotX = rotX * 1.1;  // Strafe correction
        
        // Calculate motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    
    private void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    
    private void stopAllMotors() {
        stopDriveMotors();
        intakeMotor.setPower(0);
        launchMotor1.setPower(0);
        launchMotor2.setPower(0);
        rampMotor.setPower(0);
    }
    
    private void updateOdometry() {
        int forwardPos = forwardOdo.getCurrentPosition();
        int rightPos = rightOdo.getCurrentPosition();
        
        int deltaForward = forwardPos - lastForwardOdoPos;
        int deltaRight = rightPos - lastRightOdoPos;
        
        lastForwardOdoPos = forwardPos;
        lastRightOdoPos = rightPos;
        
        double forwardInches = deltaForward / ODO_COUNTS_PER_INCH;
        double rightInches = deltaRight / ODO_COUNTS_PER_INCH;
        
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        double robotDX = rightInches;
        double robotDY = forwardInches;
        
        double fieldDX = robotDX * Math.cos(currentHeading) - robotDY * Math.sin(currentHeading);
        double fieldDY = robotDX * Math.sin(currentHeading) + robotDY * Math.cos(currentHeading);
        
        robotX += fieldDX / 12.0;
        robotY += fieldDY / 12.0;
        robotHeading = currentHeading;
    }
    
    private AprilTagDetection getAprilTagDetection(int targetId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }
    
    private AprilTagDetection getAnyObeliskTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            for (int obeliskId : OBELISK_APRILTAG_IDS) {
                if (detection.id == obeliskId) {
                    return detection;
                }
            }
        }
        return null;
    }
    
    private void calculatePositionFromObelisk(AprilTagDetection obeliskTag) {
        // Obelisk is at field center (0, 0)
        // AprilTag detection gives us range, bearing, and yaw
        
        // Range is distance from camera to tag (in inches, convert to feet)
        double rangeToTag = obeliskTag.ftcPose.range / 12.0;
        
        // Bearing is the horizontal angle to the tag (degrees, convert to radians)
        double bearingToTag = Math.toRadians(obeliskTag.ftcPose.bearing);
        
        // Yaw is how rotated the robot is relative to tag
        double yawToTag = Math.toRadians(obeliskTag.ftcPose.yaw);
        
        // Get current robot heading from IMU
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        // Calculate absolute angle from robot to obelisk
        double absoluteAngleToTag = currentHeading + bearingToTag;
        
        // Obelisk position (field center)
        double obeliskX = 0.0;
        double obeliskY = 0.0;
        
        // Calculate robot position relative to obelisk
        // The robot is 'rangeToTag' away at angle 'absoluteAngleToTag' from obelisk
        robotX = obeliskX - rangeToTag * Math.sin(absoluteAngleToTag);
        robotY = obeliskY - rangeToTag * Math.cos(absoluteAngleToTag);
        robotHeading = currentHeading;
        
        // Note: This assumes camera is at robot center. If camera is offset from 
        // robot center, you'll need to add offset compensation here.
    }
}
