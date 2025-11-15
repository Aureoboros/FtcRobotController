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
public class OfficialTeleop1 extends LinearOpMode {
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
    private static final boolean GAMEPAD1_ACTIVE = true;
    private static final boolean GAMEPAD2_ACTIVE = true;

    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 0.5;
    private static final double GAMEPAD2_MAX_POWER = 0.5;
    private static final double DPAD_POWER = 0.8;

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

    // ========== FIELD POSITIONS ==========
    private static final double OBELISK_X = 0.0;
    private static final double OBELISK_Y = 6.0;
    
    // AprilTag positions on field
    private static final double TAG_20_X = -0.7;  // Blue alliance goal tag X
    private static final double TAG_20_Y = 73.4;  // Blue alliance goal tag Y
    private static final double TAG_24_X = 0.7;   // Red alliance goal tag X (estimated)
    private static final double TAG_24_Y = 73.4;  // Red alliance goal tag Y (estimated)
    
    // Target positions based on alliance (in inches from tag)
    private static final double BLUE_TARGET_X_INCHES = 0.0;
    private static final double BLUE_TARGET_Y_INCHES = 3.0;
    private static final double RED_TARGET_X_INCHES = 0.0;
    private static final double RED_TARGET_Y_INCHES = -3.0;
    
    // Target positions in feet (for calculations)
    private static final double BLUE_TARGET_X = BLUE_TARGET_X_INCHES / 12.0;
    private static final double BLUE_TARGET_Y = BLUE_TARGET_Y_INCHES / 12.0;
    private static final double RED_TARGET_X = RED_TARGET_X_INCHES / 12.0;
    private static final double RED_TARGET_Y = RED_TARGET_Y_INCHES / 12.0;

    // ========== STATE VARIABLES ==========
    private enum Alliance {NONE, RED, BLUE}

    private Alliance selectedAlliance = Alliance.NONE;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========== APRILTAG AUTO-ALIGNMENT STATE ==========
    private boolean autoAligning = false;  // DPAD RIGHT - align to tag
    private boolean autoNavigatingToPosition = false;  // DPAD LEFT - navigate to position
    
    // ========== RB BUTTON ROTATION SEARCH STATE ==========
    private boolean rotatingToTag = false;
    private double rbRotationStartHeading = 0.0;
    private double rbLastHeading = 0.0;
    private double rbCumulativeRotation = 0.0;
    private static final double RB_ROTATION_SPEED = 0.25;
    
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
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
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
        telemetry.addLine("LEFT BUMPER = Blue Alliance");
        telemetry.addLine("RIGHT BUMPER = Red Alliance");
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
        boolean launchMotorReverse = false;
        boolean wheelsLocked = false;
        double runtimeLaunchMotorPower = 0.7;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Store previous gamepad states
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // ========== ALLIANCE SELECTION (BUMPERS - BOTH GAMEPADS) ==========
            // LEFT BUMPER = Blue Alliance
            if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) ||
                    (currentGamepad2.left_bumper && !previousGamepad2.left_bumper)) {
                selectedAlliance = Alliance.BLUE;
                telemetry.addLine("✓ BLUE ALLIANCE SELECTED");
                telemetry.update();
                sleep(500);
            }
            
            // RIGHT BUMPER = Red Alliance
            if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ||
                    (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)) {
                selectedAlliance = Alliance.RED;
                telemetry.addLine("✓ RED ALLIANCE SELECTED");
                telemetry.update();
                sleep(500);
            }

            // ========== DPAD LEFT - NAVIGATE TO POSITION (BOTH GAMEPADS) ==========
            if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left) ||
                    (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)) {
                if (selectedAlliance == Alliance.NONE) {
                    telemetry.addLine("⚠ SELECT ALLIANCE FIRST!");
                    telemetry.addLine("LEFT BUMPER = Blue | RIGHT BUMPER = Red");
                } else {
                    int targetTagId = (selectedAlliance == Alliance.BLUE) ? BLUE_APRILTAG_ID : RED_APRILTAG_ID;
                    AprilTagDetection targetTag = getAprilTagDetection(targetTagId);
                    
                    if (targetTag != null) {
                        double tagX = (selectedAlliance == Alliance.BLUE) ? TAG_20_X : TAG_24_X;
                        double tagY = (selectedAlliance == Alliance.BLUE) ? TAG_20_Y : TAG_24_Y;
                        calculatePositionFromTag(targetTag, tagX, tagY, imu);
                        autoNavigatingToPosition = true;
                        autoAligning = false;
                        telemetry.addLine("✓ Navigating to " + selectedAlliance + " position");
                    } else {
                        telemetry.addLine("⚠ Alliance tag not detected!");
                        telemetry.addLine("Tag ID: " + targetTagId);
                    }
                }
            }
            
            // ========== DPAD RIGHT - ANGLE TO APRILTAG (BOTH GAMEPADS) ==========
            if ((currentGamepad1.dpad_right && !previousGamepad1.dpad_right) ||
                    (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
                if (selectedAlliance == Alliance.NONE) {
                    telemetry.addLine("⚠ SELECT ALLIANCE FIRST!");
                    telemetry.addLine("LEFT BUMPER = Blue | RIGHT BUMPER = Red");
                } else {
                    autoAligning = true;
                    autoNavigatingToPosition = false;
                    telemetry.addLine("✓ Aligning to " + selectedAlliance + " tag");
                }
            }

            // Check for driver override
            boolean driverOverride = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;

            if (driverOverride && !rotatingToTag) {
                if (autoAligning) autoAligning = false;
                if (autoNavigatingToPosition) autoNavigatingToPosition = false;
            }
            
            // Update robot heading from IMU
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = 0, x = 0, rx = 0;
            String autoStatus = "MANUAL";
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";

            // Check which gamepad is active
            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE;

            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;

            if (autoAligning) {
                // ========== AUTO-ALIGNMENT TO ALLIANCE TAG ==========
                int targetTag = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : BLUE_APRILTAG_ID;
                AprilTagDetection detection = getAprilTagDetection(targetTag);

                if (detection != null) {
                    double yawError = detection.ftcPose.yaw;
                    if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES) {
                        rx = 0;
                        autoStatus = "ALIGNED ✓";
                    } else {
                        rx = Math.signum(yawError) * AUTO_ALIGN_ROTATION_SPEED;
                        autoStatus = String.format("ALIGNING (%.1f°)", yawError);
                    }
                    x = 0;
                    y = 0;
                } else {
                    rx = (selectedAlliance == Alliance.RED) ? 
                        AUTO_ALIGN_ROTATION_SPEED * SLOW_MODE_MULTIPLIER : 
                        -AUTO_ALIGN_ROTATION_SPEED * SLOW_MODE_MULTIPLIER;
                    x = 0;
                    y = 0;
                    autoStatus = "SEARCHING FOR TAG...";
                }
            } else if (autoNavigatingToPosition) {
                // ========== AUTO-NAVIGATION TO TARGET POSITION ==========
                int targetTagId = (selectedAlliance == Alliance.BLUE) ? BLUE_APRILTAG_ID : RED_APRILTAG_ID;
                AprilTagDetection navTag = getAprilTagDetection(targetTagId);
                
                if (navTag != null) {
                    double tagX = (selectedAlliance == Alliance.BLUE) ? TAG_20_X : TAG_24_X;
                    double tagY = (selectedAlliance == Alliance.BLUE) ? TAG_20_Y : TAG_24_Y;
                    calculatePositionFromTag(navTag, tagX, tagY, imu);
                }
                
                // Calculate target position based on alliance
                double targetX = robotX + ((selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X);
                double targetY = robotY + ((selectedAlliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y);
                
                double deltaX = targetX - robotX;
                double deltaY = targetY - robotY;
                double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                double distanceInches = distanceToTarget * 12.0;
                double distanceFeet = distanceToTarget;

                if (distanceInches < POSITION_TOLERANCE_INCHES) {
                    x = 0;
                    y = 0;
                    rx = 0;
                    autoStatus = String.format("AT POSITION ✓ (%.1f\" away)", distanceInches);
                } else {
                    double angleToTarget = Math.atan2(deltaX, deltaY);
                    
                    // Progressive speed control
                    double speed = AUTO_NAV_MAX_SPEED;
                    
                    if (distanceInches < 12.0) {
                        speed = AUTO_NAV_MIN_SPEED * 0.3;
                    } else if (distanceInches < BRAKING_ZONE_INCHES) {
                        double brakingRatio = (distanceInches - 12.0) / (BRAKING_ZONE_INCHES - 12.0);
                        speed = AUTO_NAV_MIN_SPEED * (0.3 + 0.4 * brakingRatio);
                    } else if (distanceFeet < BRAKING_DISTANCE_FEET) {
                        double brakingRatio = (distanceFeet - BRAKING_ZONE_INCHES / 12.0) / 
                                             (BRAKING_DISTANCE_FEET - BRAKING_ZONE_INCHES / 12.0);
                        speed = AUTO_NAV_MIN_SPEED * 0.7 + 
                               (AUTO_NAV_MID_SPEED - AUTO_NAV_MIN_SPEED * 0.7) * brakingRatio;
                    } else if (distanceFeet < SLOWDOWN_DISTANCE_FEET) {
                        double slowdownRatio = (distanceFeet - BRAKING_DISTANCE_FEET) / 
                                              (SLOWDOWN_DISTANCE_FEET - BRAKING_DISTANCE_FEET);
                        speed = AUTO_NAV_MID_SPEED + 
                               (AUTO_NAV_MAX_SPEED - AUTO_NAV_MID_SPEED) * slowdownRatio;
                    }
                    
                    if (navTag != null) {
                        speed *= 0.9;
                    }

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

                    double heading = robotHeading;
                    double robotRelativeAngle = angleToTarget - heading;
                    while (robotRelativeAngle > Math.PI) robotRelativeAngle -= 2 * Math.PI;
                    while (robotRelativeAngle < -Math.PI) robotRelativeAngle += 2 * Math.PI;
                    
                    x = Math.sin(robotRelativeAngle) * speed;
                    y = Math.cos(robotRelativeAngle) * speed;
                    
                    double headingError = angleToTarget - heading;
                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;
                    
                    double rotationGain = 0.12;
                    if (distanceInches < 24.0) rotationGain = 0.06;
                    if (distanceInches < 12.0) rotationGain = 0.03;
                    
                    rx = headingError * rotationGain;
                    
                    double maxRotationSpeed = 0.12;
                    if (distanceInches < 12.0) maxRotationSpeed = 0.06;
                    rx = Math.max(Math.min(rx, maxRotationSpeed), -maxRotationSpeed);
                    
                    autoStatus = String.format("NAV TO POSITION: %.1f\" @ %.1f%%", 
                            distanceInches, speed * 100);
                }
            } else {
                // ========== MANUAL CONTROL ==========
                y = 0;
                x = 0;
                rx = 0;
                maxDrivePower = GAMEPAD1_MAX_POWER;
                activeDriver = "NONE";
                
                if (gamepad1Active) {
                    double leftStickY = currentGamepad1.left_stick_y;
                    x = leftStickY;
                    
                    double rightStickX = currentGamepad1.right_stick_x;
                    y = rightStickX;
                    
                    rx = 0;
                    
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (gamepad2Active) {
                    double leftStickY = currentGamepad2.left_stick_y;
                    x = leftStickY;
                    
                    double rightStickX = -currentGamepad2.right_stick_x;
                    y = rightStickX;
                    
                    rx = 0;
                    
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                }

                x = applyDeadzone(x);
                y = applyDeadzone(y);
                rx = applyDeadzone(rx);

                if (slowMode) {
                    y *= SLOW_MODE_MULTIPLIER;
                    x *= SLOW_MODE_MULTIPLIER;
                    rx *= SLOW_MODE_MULTIPLIER;
                }
            }

            // ========== IMU RESET (BOTH GAMEPADS - BACK BUTTON) ==========
            boolean backAloneG1 = currentGamepad1.back && !previousGamepad1.back && 
                    !(launchMotorActive && !launchMotorReverse && 
                      (currentGamepad1.dpad_up || currentGamepad1.dpad_down));
            boolean backAloneG2 = currentGamepad2.back && !previousGamepad2.back && 
                    !(launchMotorActive && !launchMotorReverse && 
                      (currentGamepad2.dpad_up || currentGamepad2.dpad_down));
            
            if (backAloneG1 || backAloneG2) {
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
            
            frontLeftPower = Range.clip(frontLeftPower, -0.5, 0.5);
            backLeftPower = Range.clip(backLeftPower, -0.5, 0.5);
            frontRightPower = Range.clip(frontRightPower, -0.5, 0.5);
            backRightPower = Range.clip(backRightPower, -0.5, 0.5);

            // Set motor powers
            if (currentGamepad1.left_trigger > JOYSTICK_DEADZONE || currentGamepad2.left_trigger > JOYSTICK_DEADZONE) {
                double ltPower = Math.max(currentGamepad1.left_trigger, currentGamepad2.left_trigger);
                ltPower = Range.clip(ltPower, 0.0, 0.5);
                frontLeftMotor.setPower(ltPower);
                backLeftMotor.setPower(-ltPower);
                frontRightMotor.setPower(ltPower);
                backRightMotor.setPower(-ltPower);
            } else if (currentGamepad1.right_trigger > JOYSTICK_DEADZONE || currentGamepad2.right_trigger > JOYSTICK_DEADZONE) {
                double rtPower = Math.max(currentGamepad1.right_trigger, currentGamepad2.right_trigger);
                rtPower = Range.clip(rtPower, 0.0, 0.5);
                frontLeftMotor.setPower(-rtPower);
                backLeftMotor.setPower(rtPower);
                frontRightMotor.setPower(-rtPower);
                backRightMotor.setPower(rtPower);
            } else {
                if (wheelsLocked) {
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);
                } else {
                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                }
            }

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
            intakeMotor.setPower(Range.clip(intakePower, -1.0, 1.0));

            // ========== LAUNCH MOTOR CONTROL (BOTH GAMEPADS) ==========
            if ((currentGamepad1.x && !previousGamepad1.x) ||
                    (currentGamepad2.x && !previousGamepad2.x)) {
                launchMotorActive = !launchMotorActive;
                sleep(500);
                rampMotor.setPower(RAMP_MOTOR_POWER);
            }

            double launchPower = 0;
            double rampPower = 0;
            String launchStatus = "STOPPED";
            if (launchMotorActive) {
                launchPower = -LAUNCH_MOTOR_POWER;
                rampPower = RAMP_MOTOR_POWER;
                launchStatus = "RUNNING";
            }
            launchMotor.setPower(Range.clip(launchPower, -1.0, 1.0));
            rampMotor.setPower(Range.clip(rampPower, -1.0, 1.0));

            // ========== RB BUTTON - ROTATE TO FIND TAG ==========
            if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ||
                    (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)) {
                if (!rotatingToTag) {
                    rotatingToTag = true;
                    rbRotationStartHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    rbLastHeading = rbRotationStartHeading;
                    rbCumulativeRotation = 0.0;
                } else {
                    rotatingToTag = false;
                }
            }

            if (rotatingToTag) {
                robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double currentHeadingChange = robotHeading - rbLastHeading;
                
                // Normalize heading change to [-π, π]
                while (currentHeadingChange > Math.PI) currentHeadingChange -= 2 * Math.PI;
                while (currentHeadingChange < -Math.PI) currentHeadingChange += 2 * Math.PI;
                
                rbCumulativeRotation += Math.abs(currentHeadingChange);
                rbLastHeading = robotHeading;
                
                // Check if we've rotated 360 degrees
                if (rbCumulativeRotation >= 2 * Math.PI) {
                    rotatingToTag = false;
                    rbCumulativeRotation = 0.0;
                } else {
                    rx = RB_ROTATION_SPEED;
                    x = 0;
                    y = 0;
                    autoStatus = "SEARCHING FOR TAG...";
                }
            }

            // ========== ENHANCED TELEMETRY ==========
            telemetry.addLine("========== MATCH MODE ==========");
            telemetry.addLine();
            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addData("Active Driver", activeDriver);
            telemetry.addLine();
            telemetry.addData("Drive Mode", autoStatus);
            if (autoAligning) {
                telemetry.addData(">>> AUTO-ALIGN", "ACTIVE <<<");
            }
            if (autoNavigatingToPosition) {
                telemetry.addData(">>> AUTO-NAV", "ACTIVE <<<");
            }
            telemetry.addLine();
            telemetry.addData("Intake", intakeStatus);
            telemetry.addData("Launch", launchStatus);
            telemetry.addLine();
            telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotHeading));
            telemetry.update();
        }
    }

    // ========== HELPER METHODS ==========
    private AprilTagDetection getAprilTagDetection(int targetId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }

    private void calculatePositionFromTag(AprilTagDetection tag, double tagX, double tagY, IMU imu) {
        double rangeToTag = tag.ftcPose.range / 12.0;
        double bearingToTag = Math.toRadians(tag.ftcPose.bearing);
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double absoluteAngleToTag = currentHeading + bearingToTag;
        robotX = tagX - rangeToTag * Math.sin(absoluteAngleToTag);
        robotY = tagY - rangeToTag * Math.cos(absoluteAngleToTag);
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return value;
    }
}
