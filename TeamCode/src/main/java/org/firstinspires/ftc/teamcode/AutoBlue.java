package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name="AutoFRBlue", group="Autonomous")
public class AutoBlue extends LinearOpMode {

    // Motor power constants
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCH_MOTOR_POWER = 0.7;  // Reduced for better accuracy
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

    // Obelisk position (center structure with AprilTags)
    private static final double OBELISK_X = 0.0;
    private static final double OBELISK_Y = 6.0;

    // Default starting position if AprilTag detection fails
    private static final double BLUE_DEFAULT_START_X = 4.0;
    private static final double BLUE_DEFAULT_START_Y = -5.0;

    // Tracking (will be set based on actual start position)
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launchMotor, rampMotor;
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
            robotX = BLUE_DEFAULT_START_X;
            robotY = BLUE_DEFAULT_START_Y;
            telemetry.addLine("⚠ WARNING: Could not detect position!");
            telemetry.addLine("Using default Blue start position");
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
        intakeBalls(3.5, BLUE_MIDDLE_SPIKE_X, BLUE_MIDDLE_SPIKE_Y);  // 3.5 seconds to intake

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
        intakeBalls(3.5, BLUE_TOP_SPIKE_X, BLUE_TOP_SPIKE_Y);

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
        launchMotor = hardwareMap.dcMotor.get("launchMotor");
        rampMotor = hardwareMap.dcMotor.get("rampMotor");

        // Set motor modes
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brake mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    private void intakeBalls(double duration, double spike_x, double spike_y) {
        intakeMotor.setPower(INTAKE_POWER);
        driveToPosition(spike_x + 2.0, spike_y, 0);
        sleep((long)(duration * 1000));
        //intakeMotor.setPower(0);

        launchMotor.setPower(0);  // Stop launch motor
        sleep(500);  // Wait 500ms
        intakeMotor.setPower(1.0);  // Start intake motor (power = 1.0)
        rampMotor.setPower(-0.5);  // Start ramp motor in reverse direction (power = 0.5)
        launchMotor.setPower(-0.8);  // Start launch motor in reverse (power = -0.7)

    }

    private void shootBalls(double duration) {
        // Spin up launch motors
        //launchMotor.setPower(LAUNCH_MOTOR_POWER);
        //sleep(1500);  // Spin-up time;

        // Stop intake and ramp
        intakeMotor.setPower(0);
        rampMotor.setPower(0);

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        launchMotor.setPower(0);
        sleep(1000);
        launchMotor.setPower(0.8);
        sleep(1500);

        // Start feeding
        intakeMotor.setPower(1.0);
        rampMotor.setPower(-0.4);  // Reverse power during launch (40% speed)

    }

        private void driveFieldCentric ( double x, double y, double rx, double botHeading){
            // Field-centric transformation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Strafe correction

            // Calculate motor powers
            //double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            //double frontLeftPower = (rotY + rotX + rx) / denominator;
            //double backLeftPower = (rotY - rotX + rx) / denominator;
            //double frontRightPower = (rotY - rotX - rx) / denominator;
            //double backRightPower = (rotY + rotX - rx) / denominator;


            double frontLeftPower = -rotY + rotX + rx;
            double backLeftPower = -rotY - rotX + rx;  // Inverted y for proper strafe
            double frontRightPower = -rotY - rotX - rx;
            double backRightPower = -rotY + rotX - rx;  // Inverted y for proper strafe

            // Find the maximum absolute power to maintain proportional relationships
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            // Scale down proportionally if any power exceeds maxDrivePower
            // This preserves the motor power ratios while respecting the power limit
            if (maxPower > 1.0) {
                double scale = 1.0 / maxPower;
                frontLeftPower *= scale;
                backLeftPower *= scale;
                frontRightPower *= scale;
                backRightPower *= scale;
            }

            // Clamp all motor powers to ±0.5 to ensure they never exceed the limit
            frontLeftPower = Range.clip(frontLeftPower, -0.5, 0.5);
            backLeftPower = Range.clip(backLeftPower, -0.5, 0.5);
            frontRightPower = Range.clip(frontRightPower, -0.5, 0.5);
            backRightPower = Range.clip(backRightPower, -0.5, 0.5);


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

        private void stopDriveMotors () {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        private void stopAllMotors () {
            stopDriveMotors();
            intakeMotor.setPower(0);
            launchMotor.setPower(0);
            rampMotor.setPower(0);
        }

        private AprilTagDetection getAprilTagDetection ( int targetId){
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == targetId) {
                    return detection;
                }
            }
            return null;
        }

        public AprilTagDetection getAnyObeliskTag() {
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

        private void calculatePositionFromObelisk (AprilTagDetection obeliskTag){
            // Obelisk is at (0, 6) on the field
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

            // Obelisk position
            double obeliskX = OBELISK_X;
            double obeliskY = OBELISK_Y;

            // Calculate robot position relative to obelisk
            // The robot is 'rangeToTag' away at angle 'absoluteAngleToTag' from obelisk
            robotX = obeliskX - rangeToTag * Math.sin(absoluteAngleToTag);
            robotY = obeliskY - rangeToTag * Math.cos(absoluteAngleToTag);
            robotHeading = currentHeading;

            // Note: This assumes camera is at robot center. If camera is offset from
            // robot center, you'll need to add offset compensation here.
        }
    }
