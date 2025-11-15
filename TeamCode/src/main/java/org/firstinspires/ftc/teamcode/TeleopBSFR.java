package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class SVTestTeleop12nov extends LinearOpMode {
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_HALF_POWER = 0.5;
    private static final double LAUNCH_MOTOR_POWER = 1.0;
    private static final double LAUNCH_MOTOR_TARGET_POWER = 0.65;
    private static final double RAMP_MOTOR_POWER = 1.0;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double AUTO_ALIGN_ROTATION_SPEED = 0.15;
    private static final double ROTATION_SPEED = 0.5;

    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 1.0;
    private static final double GAMEPAD2_MAX_POWER = 0.5;

    // ========== APRILTAG ALIGNMENT CONSTANTS ==========
    private static final double ANGLE_TOLERANCE_DEGREES = 3.0;
    private static final double POSITION_TOLERANCE_INCHES = 8.0;
    private static final double BRAKING_ZONE_INCHES = 18.0;
    private static final double AUTO_NAV_MAX_SPEED = 0.25;
    private static final double AUTO_NAV_MID_SPEED = 0.15;
    private static final double AUTO_NAV_MIN_SPEED = 0.08;
    private static final double SLOWDOWN_DISTANCE_FEET = 6.0;
    private static final double BRAKING_DISTANCE_FEET = 3.0;

    // ========== APRILTAG CONFIGURATION ==========
    private static final int RED_APRILTAG_ID = 24;
    private static final int BLUE_APRILTAG_ID = 20;
    private static final int[] OBELISK_APRILTAG_IDS = {21, 22, 23};

    // ========== FIELD POSITIONS (CORRECTED) ==========
    // Tag 20 - Blue Alliance Goal
    private static final double TAG_20_X = -4.86;  // feet
    private static final double TAG_20_Y = -4.64;  // feet
    
    // Tag 24 - Red Alliance Goal
    private static final double TAG_24_X = 4.86;   // feet
    private static final double TAG_24_Y = 4.64;   // feet
    
    // Obelisk tags (21, 22, 23)
    private static final double OBELISK_X = 0.0;   // feet
    private static final double OBELISK_Y = 6.0;   // feet
    
    // Target positions - depend on alliance
    private static final double BLUE_TARGET_X = -1.0;  // feet
    private static final double BLUE_TARGET_Y = 1.0;   // feet
    private static final double RED_TARGET_X = 1.0;    // feet
    private static final double RED_TARGET_Y = 1.0;    // feet

    // ========== STATE VARIABLES ==========
    private enum Alliance {NONE, RED, BLUE}

    private Alliance selectedAlliance = Alliance.NONE;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========== APRILTAG AUTO-ALIGNMENT STATE ==========
    private boolean autoAligning = false;
    private boolean autoNavigatingToZero = false;
    
    // ========== POSITION TRACKING ==========
    private double robotX = 0.0;  // in feet
    private double robotY = 0.0;  // in feet
    private double robotHeading = 0.0;  // in radians

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

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset and configure encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor modes for mechanisms
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU with proper orientation
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // ========== INIT SETUP ROUTINE ==========
        telemetry.addLine("========================================");
        telemetry.addLine("INITIALIZING ROBOT");
        telemetry.addLine("========================================");
        telemetry.addData("IMU Status", "Initialized");
        telemetry.addData("Camera Status", visionPortal.getCameraState());
        telemetry.addData("Encoders", "Reset");
        telemetry.addLine("========================================");
        telemetry.addLine("✓ ROBOT READY");
        telemetry.addLine("Press LB for BLUE, RB for RED");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();

        // Initialize gamepad state tracking
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Toggle states
        boolean slowMode = false;
        boolean intakeHalfSpeed = false;
        boolean intakeForwardActive = false;
        boolean intakeReverseActive = false;
        boolean launchMotorActive = false;
        boolean rampMotorActive = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Store previous gamepad states
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // ========== ALLIANCE SELECTION (BOTH GAMEPADS) ==========
            // Left Bumper = BLUE alliance
            if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) ||
                    (currentGamepad2.left_bumper && !previousGamepad2.left_bumper)) {
                selectedAlliance = Alliance.BLUE;
            }
            
            // Right Bumper = RED alliance
            if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ||
                    (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)) {
                selectedAlliance = Alliance.RED;
            }

            // ========== DPAD UP - ALIGN TO ALLIANCE TAG (BOTH GAMEPADS) ==========
            if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) ||
                    (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
                if (selectedAlliance != Alliance.NONE) {
                    autoAligning = true;
                    autoNavigatingToZero = false;
                } else {
                    telemetry.addLine("⚠ Please select alliance first (LB=Blue, RB=Red)");
                }
            }
            
            // ========== DPAD DOWN - NAVIGATE TO TARGET POSITION (BOTH GAMEPADS) ==========
            if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) ||
                    (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
                
                // Check if alliance is selected
                if (selectedAlliance == Alliance.NONE) {
                    telemetry.addLine("⚠ Please select alliance first (LB=Blue, RB=Red)");
                } else {
                    // Try to detect position from any available tag
                    AprilTagDetection tag20 = getAprilTagDetection(BLUE_APRILTAG_ID);
                    AprilTagDetection tag24 = getAprilTagDetection(RED_APRILTAG_ID);
                    AprilTagDetection obeliskTag = getAnyObeliskTag();
                    
                    if (tag20 != null) {
                        calculatePositionFromTag(tag20, TAG_20_X, TAG_20_Y, imu);
                        autoNavigatingToZero = true;
                        autoAligning = false;
                        telemetry.addLine("✓ Position detected from Tag 20 (Blue Goal)");
                    } else if (tag24 != null) {
                        calculatePositionFromTag(tag24, TAG_24_X, TAG_24_Y, imu);
                        autoNavigatingToZero = true;
                        autoAligning = false;
                        telemetry.addLine("✓ Position detected from Tag 24 (Red Goal)");
                    } else if (obeliskTag != null) {
                        calculatePositionFromObelisk(obeliskTag, imu);
                        autoNavigatingToZero = true;
                        autoAligning = false;
                        telemetry.addLine("✓ Position detected from Obelisk tag");
                    } else {
                        telemetry.addLine("⚠ Cannot navigate: No tags detected!");
                        telemetry.addLine("Please ensure camera can see Tag 20, 24, or Obelisk (21-23)");
                    }
                }
            }

            // Check for driver override from either gamepad
            boolean driverOverride = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;

            if (driverOverride) {
                if (autoAligning) autoAligning = false;
                if (autoNavigatingToZero) autoNavigatingToZero = false;
            }
            
            // Update robot heading from IMU
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            
            // Update position periodically using AprilTag if available
            if (!autoNavigatingToZero) {
                AprilTagDetection updateTag20 = getAprilTagDetection(BLUE_APRILTAG_ID);
                AprilTagDetection updateTag24 = getAprilTagDetection(RED_APRILTAG_ID);
                AprilTagDetection updateObeliskTag = getAnyObeliskTag();
                
                if (updateTag20 != null) {
                    calculatePositionFromTag(updateTag20, TAG_20_X, TAG_20_Y, imu);
                } else if (updateTag24 != null) {
                    calculatePositionFromTag(updateTag24, TAG_24_X, TAG_24_Y, imu);
                } else if (updateObeliskTag != null) {
                    calculatePositionFromObelisk(updateObeliskTag, imu);
                }
            }

            double y = 0, x = 0, rx = 0;
            String autoStatus = "MANUAL";
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";

            // ========== DRIVER PRIORITY: GAMEPAD1 > GAMEPAD2 ==========
            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE;

            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;

            if (autoAligning) {
                // ========== APRILTAG AUTO-ALIGNMENT ==========
                int targetTag = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : BLUE_APRILTAG_ID;
                AprilTagDetection detection = getAprilTagDetection(targetTag);

                if (detection != null) {
                    double yawError = detection.ftcPose.yaw;
                    if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES) {
                        rx = 0;
                        autoStatus = "ALIGNED ✓";
                    } else {
                        rx = Math.signum(yawError) * AUTO_ALIGN_ROTATION_SPEED;
                        autoStatus = String.format("ALIGNING TO TAG %d (%.1f°)", targetTag, yawError);
                    }
                    x = 0;
                    y = 0;
                } else {
                    // Searching for tag - rotate slowly
                    rx = (selectedAlliance == Alliance.RED) ? 
                        AUTO_ALIGN_ROTATION_SPEED * SLOW_MODE_MULTIPLIER : 
                        -AUTO_ALIGN_ROTATION_SPEED * SLOW_MODE_MULTIPLIER;
                    x = 0;
                    y = 0;
                    autoStatus = String.format("SEARCHING FOR TAG %d...", targetTag);
                }
            } else if (autoNavigatingToZero) {
                // ========== AUTONOMOUS NAVIGATION TO TARGET POSITION ==========
                // Target depends on alliance
                double targetX = (selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
                double targetY = (selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;
                
                // Update position from AprilTag during navigation
                AprilTagDetection navTag20 = getAprilTagDetection(BLUE_APRILTAG_ID);
                AprilTagDetection navTag24 = getAprilTagDetection(RED_APRILTAG_ID);
                AprilTagDetection navObeliskTag = getAnyObeliskTag();
                
                if (navTag20 != null) {
                    calculatePositionFromTag(navTag20, TAG_20_X, TAG_20_Y, imu);
                } else if (navTag24 != null) {
                    calculatePositionFromTag(navTag24, TAG_24_X, TAG_24_Y, imu);
                } else if (navObeliskTag != null) {
                    calculatePositionFromObelisk(navObeliskTag, imu);
                }
                
                double deltaX = targetX - robotX;
                double deltaY = targetY - robotY;
                double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                double distanceInches = distanceToTarget * 12.0;
                double distanceFeet = distanceToTarget;

                if (distanceInches < POSITION_TOLERANCE_INCHES) {
                    // Reached target
                    x = 0;
                    y = 0;
                    rx = 0;
                    autoStatus = String.format("AT TARGET (%.1f, %.1f) ✓ (%.1f\" away)", 
                            targetX, targetY, distanceInches);
                } else {
                    // Calculate angle to target
                    double angleToTarget = Math.atan2(deltaX, deltaY);
                    
                    // Progressive speed control
                    double speed = AUTO_NAV_MAX_SPEED;
                    
                    if (distanceInches < 12.0) {
                        speed = AUTO_NAV_MIN_SPEED * 0.3;
                    } else if (distanceInches < BRAKING_ZONE_INCHES) {
                        double brakingRatio = (distanceInches - 12.0) / (BRAKING_ZONE_INCHES - 12.0);
                        speed = AUTO_NAV_MIN_SPEED * (0.3 + 0.4 * brakingRatio);
                    } else if (distanceFeet < BRAKING_DISTANCE_FEET) {
                        double brakingRatio = (distanceFeet - BRAKING_ZONE_INCHES / 12.0) / (BRAKING_DISTANCE_FEET - BRAKING_ZONE_INCHES / 12.0);
                        speed = AUTO_NAV_MIN_SPEED * 0.7 + (AUTO_NAV_MID_SPEED - AUTO_NAV_MIN_SPEED * 0.7) * brakingRatio;
                    } else if (distanceFeet < SLOWDOWN_DISTANCE_FEET) {
                        double slowdownRatio = (distanceFeet - BRAKING_DISTANCE_FEET) / (SLOWDOWN_DISTANCE_FEET - BRAKING_DISTANCE_FEET);
                        speed = AUTO_NAV_MID_SPEED + (AUTO_NAV_MAX_SPEED - AUTO_NAV_MID_SPEED) * slowdownRatio;
                    }
                    
                    // Slowdown when tags are visible
                    if (navTag20 != null || navTag24 != null) {
                        speed *= 0.9;
                    } else if (navObeliskTag == null) {
                        speed *= 0.7;
                    }

                    // Ensure minimum speed
                    double minSpeed;
                    if (distanceInches > 36.0) {
                        minSpeed = AUTO_NAV_MID_SPEED;
                    } else if (distanceInches > 24.0) {
                        minSpeed = AUTO_NAV_MIN_SPEED * 1.5;
                    } else if (distanceInches > 12.0) {
                        minSpeed = AUTO_NAV_MIN_SPEED;
                    } else {
                        minSpeed = AUTO_NAV_MIN_SPEED * 0.6;
                    }
                    speed = Math.max(speed, minSpeed);

                    // Convert to robot frame
                    double heading = robotHeading;
                    double robotRelativeAngle = angleToTarget - heading;
                    while (robotRelativeAngle > Math.PI) robotRelativeAngle -= 2 * Math.PI;
                    while (robotRelativeAngle < -Math.PI) robotRelativeAngle += 2 * Math.PI;
                    
                    x = Math.sin(robotRelativeAngle) * speed;
                    y = Math.cos(robotRelativeAngle) * speed;
                    
                    // Heading correction
                    double headingError = angleToTarget - heading;
                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;
                    
                    double rotationGain = 0.12;
                    if (distanceInches < 24.0) {
                        rotationGain = 0.06;
                    }
                    if (distanceInches < 12.0) {
                        rotationGain = 0.03;
                    }
                    rx = headingError * rotationGain;
                    
                    double maxRotationSpeed = 0.12;
                    if (distanceInches < 12.0) {
                        maxRotationSpeed = 0.06;
                    }
                    rx = Math.max(Math.min(rx, maxRotationSpeed), -maxRotationSpeed);
                    
                    autoStatus = String.format("NAV TO (%.1f, %.1f): %.1f\" @ %.1f%%", 
                            targetX, targetY, distanceInches, speed * 100);
                }
            } else {
                // ========== MANUAL CONTROL ==========
                y = 0;
                x = 0;
                rx = 0;
                maxDrivePower = GAMEPAD1_MAX_POWER;
                activeDriver = "NONE";
                
                // Check Gamepad1 first (priority)
                if (gamepad1Active) {
                    double leftStickY = currentGamepad1.left_stick_y;
                    x = leftStickY;
                    
                    double rightStickX = currentGamepad1.right_stick_x;
                    y = rightStickX;
                    
                    // Rotation from triggers
                    if (currentGamepad1.left_trigger > JOYSTICK_DEADZONE) {
                        rx = -currentGamepad1.left_trigger * ROTATION_SPEED;  // Counter-clockwise
                    } else if (currentGamepad1.right_trigger > JOYSTICK_DEADZONE) {
                        rx = currentGamepad1.right_trigger * ROTATION_SPEED;  // Clockwise
                    }
                    
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (gamepad2Active) {
                    double leftStickY = currentGamepad2.left_stick_y;
                    x = leftStickY;
                    
                    double rightStickX = -currentGamepad2.right_stick_x;
                    y = rightStickX;
                    
                    // Rotation from triggers
                    if (currentGamepad2.left_trigger > JOYSTICK_DEADZONE) {
                        rx = -currentGamepad2.left_trigger * ROTATION_SPEED;  // Counter-clockwise
                    } else if (currentGamepad2.right_trigger > JOYSTICK_DEADZONE) {
                        rx = currentGamepad2.right_trigger * ROTATION_SPEED;  // Clockwise
                    }
                    
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                }

                // Apply deadzone
                x = applyDeadzone(x);
                y = applyDeadzone(y);
                rx = applyDeadzone(rx);

                // Apply slow mode
                if (slowMode) {
                    y *= SLOW_MODE_MULTIPLIER;
                    x *= SLOW_MODE_MULTIPLIER;
                    rx *= SLOW_MODE_MULTIPLIER;
                }
            }

            // ========== IMU RESET (BOTH GAMEPADS - BACK BUTTON) ==========
            if ((currentGamepad1.back && !previousGamepad1.back) ||
                    (currentGamepad2.back && !previousGamepad2.back)) {
                imu.resetYaw();
                robotX = 0.0;
                robotY = 0.0;
                robotHeading = 0.0;
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // ========== MECANUM WHEEL POWER CALCULATION ==========
            double frontLeftPower = -y + x + rx;
            double backLeftPower = -y - x + rx;
            double frontRightPower = -y - x - rx;
            double backRightPower = -y + x - rx;

            // Scale down if exceeds max power
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > maxDrivePower) {
                double scale = maxDrivePower / maxPower;
                frontLeftPower *= scale;
                backLeftPower *= scale;
                frontRightPower *= scale;
                backRightPower *= scale;
            }

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ========== INTAKE CONTROL (BOTH GAMEPADS) ==========
            if ((currentGamepad1.start && !previousGamepad1.start) ||
                    (currentGamepad2.start && !previousGamepad2.start)) {
                intakeHalfSpeed = !intakeHalfSpeed;
            }

            if ((currentGamepad1.b && !previousGamepad1.b) ||
                    (currentGamepad2.b && !previousGamepad2.b)) {
                intakeForwardActive = !intakeForwardActive;
                if (intakeForwardActive) intakeReverseActive = false;
            }

            if ((currentGamepad1.a && !previousGamepad1.a) ||
                    (currentGamepad2.a && !previousGamepad2.a)) {
                intakeReverseActive = !intakeReverseActive;
                if (intakeReverseActive) intakeForwardActive = false;
            }

            double intakePower = 0;
            String intakeStatus = "STOPPED";
            if (intakeForwardActive) {
                double speedToUse = intakeHalfSpeed ? INTAKE_HALF_POWER : INTAKE_FULL_POWER;
                intakePower = speedToUse;
                intakeStatus = "FORWARD (" + (intakeHalfSpeed ? "50%" : "100%") + ")";
            } else if (intakeReverseActive) {
                double speedToUse = intakeHalfSpeed ? INTAKE_HALF_POWER : INTAKE_FULL_POWER;
                intakePower = -speedToUse;
                intakeStatus = "REVERSE (" + (intakeHalfSpeed ? "50%" : "100%") + ")";
            }
            intakeMotor.setPower(intakePower);

            // ========== LAUNCH MOTOR CONTROL (BOTH GAMEPADS) ==========
            double launchPower = 0;
            String launchStatus = "STOPPED";
            if (launchMotorActive) {
                launchPower = LAUNCH_MOTOR_TARGET_POWER;
                launchMotor.setPower(launchPower);
                launchStatus = "RUNNING";
            } else {
                launchMotor.setPower(0);
            }
            
            // ========== SHOOT SEQUENCE (X BUTTON) ==========
            if ((currentGamepad1.x && !previousGamepad1.x) ||
                    (currentGamepad2.x && !previousGamepad2.x)) {
                // Stop intake and ramp
                intakeMotor.setPower(0);
                rampMotor.setPower(0);
                
                // Start launch motor if not already running
                if (!launchMotorActive) {
                    launchMotorActive = true;
                    launchMotor.setPower(LAUNCH_MOTOR_TARGET_POWER);
                    sleep(3000);  // Wait for launch motor to spin up
                } else {
                    sleep(500);
                }
                
                // Start feeding
                intakeMotor.setPower(1.0);
                rampMotor.setPower(-RAMP_MOTOR_POWER * 0.4);
            }
            
            // Y button - Stop launch and start intake (for pickup)
            if ((currentGamepad1.y && !previousGamepad1.y) ||
                    (currentGamepad2.y && !previousGamepad2.y)) {
                launchMotorActive = false;
                launchMotor.setPower(0);
                sleep(500);
                intakeMotor.setPower(1.0);
            }

            // ========== ENHANCED TELEMETRY ==========
            telemetry.addLine("========== MATCH MODE ==========");
            telemetry.addLine();
            telemetry.addData("Alliance", selectedAlliance);
            if (selectedAlliance == Alliance.NONE) {
                telemetry.addLine("⚠ Press LB for BLUE or RB for RED");
            }
            telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotHeading));
            telemetry.addLine();
            
            if (!autoNavigatingToZero && !autoAligning) {
                telemetry.addData("Drive Mode", "ROBOT-CENTRIC");
                telemetry.addData("Movement", "Left Stick Y: Forward/Back");
                telemetry.addData("Strafe", "Right Stick X: Left/Right");
                telemetry.addData("Rotation", "LT/RT: Turn Left/Right");
            }
            
            telemetry.addData("Active Driver", activeDriver);
            telemetry.addLine();
            telemetry.addData("Drive Mode", autoStatus);
            
            if (autoAligning) {
                telemetry.addData(">>> AUTO-ALIGN", "ACTIVE <<<");
                int targetTag = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : BLUE_APRILTAG_ID;
                AprilTagDetection alignTag = getAprilTagDetection(targetTag);
                if (alignTag != null) {
                    telemetry.addData("Tag %d Detected", targetTag);
                    telemetry.addData("  Yaw Error", "%.1f°", alignTag.ftcPose.yaw);
                    telemetry.addData("  Range", "%.1f inches", alignTag.ftcPose.range);
                } else {
                    telemetry.addData("Searching for Tag", targetTag);
                }
            }
            
            if (autoNavigatingToZero) {
                telemetry.addData(">>> NAV TO TARGET", "ACTIVE <<<");
                double targetX = (selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
                double targetY = (selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;
                telemetry.addData("Target Position", "(%.1f, %.1f) ft", targetX, targetY);
                telemetry.addData("Current Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                double distToTarget = Math.sqrt((targetX - robotX) * (targetX - robotX) + 
                                               (targetY - robotY) * (targetY - robotY)) * 12.0;
                telemetry.addData("Distance to Target", "%.1f inches", distToTarget);
                
                // Show which tag is being used
                AprilTagDetection telemetryTag20 = getAprilTagDetection(BLUE_APRILTAG_ID);
                AprilTagDetection telemetryTag24 = getAprilTagDetection(RED_APRILTAG_ID);
                AprilTagDetection telemetryObeliskTag = getAnyObeliskTag();
                
                if (telemetryTag20 != null) {
                    telemetry.addData("Using Tag", "20 (Blue Goal)");
                    telemetry.addData("  Range", "%.1f inches", telemetryTag20.ftcPose.range);
                } else if (telemetryTag24 != null) {
                    telemetry.addData("Using Tag", "24 (Red Goal)");
                    telemetry.addData("  Range", "%.1f inches", telemetryTag24.ftcPose.range);
                } else if (telemetryObeliskTag != null) {
                    telemetry.addData("Using Tag", "%d (Obelisk)", telemetryObeliskTag.id);
                } else {
                    telemetry.addData("Using Tag", "NO TAGS - Position may drift");
                }
            }
            
            telemetry.addLine();
            telemetry.addData("Intake", intakeStatus);
            telemetry.addData("Launch", launchStatus);
            telemetry.addLine();
            
            if (slowMode) {
                telemetry.addData("Slow Mode", "ACTIVE (30%)");
                telemetry.addLine();
            }
            
            telemetry.addData("Motor Powers", "");
            telemetry.addData("  FL", "%.2f", frontLeftPower);
            telemetry.addData("  BL", "%.2f", backLeftPower);
            telemetry.addData("  FR", "%.2f", frontRightPower);
            telemetry.addData("  BR", "%.2f", backRightPower);
            telemetry.addLine();
            telemetry.addLine("========== CONTROLS ==========");
            telemetry.addData("LB/RB", "Select Alliance (Blue/Red)");
            telemetry.addData("D-pad UP", "Align to Alliance Tag");
            telemetry.addData("D-pad DOWN", "Navigate to (0,0)");
            telemetry.addData("A/B", "Intake Reverse/Forward");
            telemetry.addData("X", "Shoot Sequence");
            telemetry.addData("Y", "Stop Launch, Start Intake");
            telemetry.update();
        }

        // Clean up vision
        visionPortal.close();
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

    private void calculatePositionFromObelisk(AprilTagDetection obeliskTag, IMU imu) {
        calculatePositionFromTag(obeliskTag, OBELISK_X, OBELISK_Y, imu);
    }
    
    private void calculatePositionFromTag(AprilTagDetection tag, double tagX, double tagY, IMU imu) {
        // Calculate robot position from AprilTag at known field position
        double rangeToTag = tag.ftcPose.range / 12.0;  // Convert inches to feet
        double bearingToTag = Math.toRadians(tag.ftcPose.bearing);
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double absoluteAngleToTag = currentHeading + bearingToTag;

        robotX = tagX - rangeToTag * Math.sin(absoluteAngleToTag);
        robotY = tagY - rangeToTag * Math.cos(absoluteAngleToTag);
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < JOYSTICK_DEADZONE) {
            return 0;
        }
        return value;
    }
}
