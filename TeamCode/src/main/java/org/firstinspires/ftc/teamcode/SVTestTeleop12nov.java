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
// yummy yummy cheesecake for our scrumptious souls. i will sell my organs for cheesecake. i am sorry, i was possessed by the demons.

@TeleOp
public class SVTestTeleop12nov extends LinearOpMode {
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_HALF_POWER = 0.5;
    private static final double LAUNCH_MOTOR_POWER = 1.0;
    private static final double LAUNCH_MOTOR_TARGET_POWER = 0.65;  // Target power when DPAD UP is pressed
    private static final double RAMP_MOTOR_POWER = 1.0;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double AUTO_ALIGN_ROTATION_SPEED = 0.15;  // Reduced for tighter turns
    private static final double ROTATION_SPEED = 0.5;
    private static final boolean GAMEPAD1_ACTIVE = true;
    private static final boolean GAMEPAD2_ACTIVE = true;

    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 1.0;
    private static final double GAMEPAD2_MAX_POWER = 0.5;
    private static final double DPAD_POWER = 0.8;

    // ========== APRILTAG ALIGNMENT CONSTANTS ==========
    private static final double ANGLE_TOLERANCE_DEGREES = 3.0;
    private static final double POSITION_TOLERANCE_INCHES = 8.0;  // Increased tolerance for better stopping
    private static final double BRAKING_ZONE_INCHES = 18.0;  // Start aggressive braking at 18 inches
    private static final double AUTO_NAV_MAX_SPEED = 0.25;  // Increased for actual movement (was 0.12)
    private static final double AUTO_NAV_MID_SPEED = 0.15;  // Medium speed for mid-range (was 0.08)
    private static final double AUTO_NAV_MIN_SPEED = 0.08;  // Minimum speed near target (was 0.04)
    private static final double SLOWDOWN_DISTANCE_FEET = 6.0;  // Start slowing down earlier (6 feet)
    private static final double BRAKING_DISTANCE_FEET = 3.0;  // Aggressive braking within 3 feet

    // ========== APRILTAG CONFIGURATION ==========
    private static final int RED_APRILTAG_ID = 24;
    private static final int BLUE_APRILTAG_ID = 20;
    private static final int[] OBELISK_APRILTAG_IDS = {21, 22, 23};  // Obelisk tags for positioning

    // ========== FIELD POSITIONS ==========
    // Obelisk position (center structure with AprilTags) - used for position calculation
    private static final double OBELISK_X = 0.0;
    private static final double OBELISK_Y = 6.0;
    
    // AprilTag 20 position (Blue alliance goal tag)
    // Field coordinates: Tag 20 is at (-0.7, 73.4) feet from field center (0, 0)
    private static final double TAG_20_X = -0.7;  // X position of Tag 20 on field (in feet)
    private static final double TAG_20_Y = 73.4;  // Y position of Tag 20 on field (in feet)
    
    // Target position (0, 0) - field center
    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = 0.0;
    
    // ========== CUSTOM TARGET POSITION ==========
    private double customTargetX = 0.0;  // in feet (field coordinates)
    private double customTargetY = 0.0;  // in feet (field coordinates)
    private double customTargetZ = 0.0;  // in inches (height/distance, for reference)
    private boolean navigatingToCustomTarget = false;

    // ========== STATE VARIABLES ==========
    private enum Alliance {NONE, RED, BLUE}

    private Alliance selectedAlliance = Alliance.NONE;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========== APRILTAG AUTO-ALIGNMENT STATE ==========
    private boolean autoAligning = false;
    private boolean autoNavigatingToZero = false;
    private boolean aimingAtTag20 = false;
    private static final int TARGET_TAG_20_ID = 20;
    
    // ========== 360-DEGREE SEARCH STATE ==========
    private boolean searching360 = false;
    private double searchStartHeading = 0.0;  // in radians
    private double lastSearchHeading = 0.0;  // in radians - track previous heading for wraparound detection
    private double cumulativeRotation = 0.0;  // in radians - track total rotation
    private static final double SEARCH_ROTATION_SPEED = 0.3;
    private static final double SEARCH_COMPLETION_TOLERANCE = 10.0;  // degrees - allow 10° overshoot
    
    // ========== RB BUTTON ROTATION SEARCH STATE ==========
    private boolean rotatingToTag20 = false;  // RB button rotation search for Tag 20
    private double rbRotationStartHeading = 0.0;  // in radians
    private double rbLastHeading = 0.0;  // in radians
    private double rbCumulativeRotation = 0.0;  // in radians
    private static final double RB_ROTATION_SPEED = 0.25;  // Slow rotation speed for RB search
    
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
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Enable continuous movement
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initial motor powers to zero
        //frontLeftMotor.setPower(0);
        //frontRightMotor.setPower(0);
        //backLeftMotor.setPower(0);
        //backRightMotor.setPower(0);

        // Initialize IMU with proper orientation (Control Hub sideways, logo facing left)
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
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();

        // Initialize gamepad state tracking for both gamepads
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
        boolean launchMotorReverse = false;  // Flag for reverse launch motor mode (Y button)
        boolean wheelsLocked = false;  // Flag to lock wheels during launch
        double runtimeLaunchMotorPower = 0.5;  // Runtime adjustable launch motor power (default 0.5, min 0.5, max 0.75)

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Store previous gamepad states
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);


            // ========== RB BUTTON - ROTATE TO FIND TAG 20 (BOTH GAMEPADS) ==========
            // Hold RB to rotate slowly until AprilTag 20 is detected
            boolean rbPressed = currentGamepad1.right_bumper || currentGamepad2.right_bumper;
            boolean rbJustPressed = (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ||
                    (currentGamepad2.right_bumper && !previousGamepad2.right_bumper);
            boolean rbJustReleased = (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) ||
                    (!currentGamepad2.right_bumper && previousGamepad2.right_bumper);
            
            if (rbJustPressed || (rbPressed && !rotatingToTag20)) {
                // Start rotation search (on press or if currently held but not yet active)
                rotatingToTag20 = true;
                rbRotationStartHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                rbLastHeading = rbRotationStartHeading;
                rbCumulativeRotation = 0.0;
            } else if (rbJustReleased || (!rbPressed && rotatingToTag20)) {
                // Stop rotation search when button is released
                rotatingToTag20 = false;
                rbCumulativeRotation = 0.0;
            }

            // ========== TRIGGERS NOW USED FOR ROTATION ==========
            // LT and RT are now used for manual rotation control
            // Auto-align and Tag 20 features disabled to allow rotation control
            // (Left trigger and right trigger now control rotation in manual control section)
            
            // ========== DPAD DOWN - NAVIGATE TO (0,0) USING TAG 20 OR OBELISK (BOTH GAMEPADS) ==========
            // Try Tag 20 first (if visible), then fall back to obelisk tags
            if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) ||
                    (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
                // First try to detect position from Tag 20
                AprilTagDetection tag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                if (tag20 != null) {
                    // Calculate position from Tag 20
                    calculatePositionFromTag(tag20, TAG_20_X, TAG_20_Y, imu);
                    autoNavigatingToZero = true;
                    autoAligning = false;
                    aimingAtTag20 = false;
                    telemetry.addLine("✓ Position detected from Tag 20");
                } else {
                    // Fall back to obelisk tags
                    AprilTagDetection obeliskTag = getAnyObeliskTag();
                    if (obeliskTag != null) {
                        calculatePositionFromObelisk(obeliskTag, imu);
                        autoNavigatingToZero = true;
                        autoAligning = false;
                        aimingAtTag20 = false;
                        telemetry.addLine("✓ Position detected from Obelisk tag");
                    } else {
                        telemetry.addLine("⚠ Cannot navigate: No tags detected!");
                        telemetry.addLine("Please ensure camera can see Tag 20 or Obelisk tag");
                    }
                }
            }
            
            // ========== DPAD RIGHT - NAVIGATE TO (0,0) USING TAG 20 ONLY (BOTH GAMEPADS) ==========
            // Navigate to (0,0) using only Tag 20 for positioning
            if ((currentGamepad1.dpad_right && !previousGamepad1.dpad_right) ||
                    (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
                AprilTagDetection tag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                if (tag20 != null) {
                    calculatePositionFromTag(tag20, TAG_20_X, TAG_20_Y, imu);
                    autoNavigatingToZero = true;
                    autoAligning = false;
                    aimingAtTag20 = false;
                    navigatingToCustomTarget = false;
                    telemetry.addLine("✓ Navigating to (0,0) using Tag 20");
                } else {
                    telemetry.addLine("⚠ Tag 20 not detected!");
                    telemetry.addLine("Hold RIGHT TRIGGER to search for Tag 20");
                }
            }
            
            // ========== DPAD UP - DETECT POSITION AND SET TARGET TO TAG 20 (BOTH GAMEPADS) ==========
            // Detect current robot position from Tag 20 and set target to Tag 20's field position
            if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) ||
                    (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
                AprilTagDetection tag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                if (tag20 != null) {
                    // Calculate where the robot currently is (using Tag 20's known field position)
                    calculatePositionFromTag(tag20, TAG_20_X, TAG_20_Y, imu);
                    
                    // Set target to Tag 20's known field position
                    customTargetX = TAG_20_X;  // Tag 20's field X position (feet)
                    customTargetY = TAG_20_Y;  // Tag 20's field Y position (feet)
                    customTargetZ = tag20.ftcPose.z;  // Z from telemetry (inches, for reference)
                    
                    telemetry.addLine("✓ Position detected and target set to Tag 20");
                    telemetry.addData("Current Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                    telemetry.addData("Tag 20 Field Position", "X: %.2f ft, Y: %.2f ft", TAG_20_X, TAG_20_Y);
                    telemetry.addData("Tag 20 Telemetry (x,y,z)", "X: %.1f in, Y: %.1f in, Z: %.1f in", 
                            tag20.ftcPose.x, tag20.ftcPose.y, tag20.ftcPose.z);
                    telemetry.addData("Target Position", "X: %.2f ft, Y: %.2f ft, Z: %.1f in", 
                            customTargetX, customTargetY, customTargetZ);
                    double distanceToTag = Math.sqrt((customTargetX - robotX) * (customTargetX - robotX) + 
                                                    (customTargetY - robotY) * (customTargetY - robotY)) * 12.0;
                    telemetry.addData("Distance to Tag 20", "%.1f inches", distanceToTag);
                    telemetry.addLine("Press DPAD LEFT to navigate to Tag 20");
                } else {
                    telemetry.addLine("⚠ Tag 20 not detected!");
                    telemetry.addLine("Cannot detect position without Tag 20");
                    telemetry.addLine("Hold RB to search for Tag 20");
                }
            }
            
            // ========== DPAD LEFT - NAVIGATE TO CUSTOM TARGET (BOTH GAMEPADS) ==========
            // Navigate to the custom target position set from Tag 20
            if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left) ||
                    (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)) {
                AprilTagDetection tag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                if (tag20 != null) {
                    // Update current position from Tag 20
                    calculatePositionFromTag(tag20, TAG_20_X, TAG_20_Y, imu);
                    // Start navigation to custom target
                    navigatingToCustomTarget = true;
                    autoNavigatingToZero = false;
                    autoAligning = false;
                    aimingAtTag20 = false;
                    telemetry.addLine("✓ Navigating to custom target");
                    telemetry.addData("Current Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                    telemetry.addData("Target Position", "X: %.2f ft, Y: %.2f ft, Z: %.1f in", 
                            customTargetX, customTargetY, customTargetZ);
                } else {
                    telemetry.addLine("⚠ Tag 20 not detected!");
                    telemetry.addLine("Need Tag 20 for position tracking");
                    telemetry.addLine("Hold RB to search for Tag 20");
                }
            }

            // Check for driver override from either gamepad
            // Left joystick for forward/backward, right joystick for left/right strafe
            // LT/RT triggers for rotation (always override auto modes)
            boolean driverOverride = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;

            if (driverOverride && !rotatingToTag20) {
                // Driver override - but don't override RB rotation search
                if (autoAligning) autoAligning = false;
                if (autoNavigatingToZero) autoNavigatingToZero = false;
                if (navigatingToCustomTarget) navigatingToCustomTarget = false;
                if (aimingAtTag20) aimingAtTag20 = false;
                // Don't cancel rotatingToTag20 - let it continue until Tag 20 is found
            }
            
            // Update robot heading from IMU (for position tracking and auto-navigation)
            // Note: For robot-centric drive, this is only used for position tracking
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            
            // Update position periodically using AprilTag if available
            // This helps maintain accurate position even when not actively navigating
            // Only update if not already navigating (navigation updates position itself)
            if (!autoNavigatingToZero) {
                AprilTagDetection obeliskTagUpdate = getAnyObeliskTag();
                if (obeliskTagUpdate != null) {
                    // Update position from tag
                    calculatePositionFromObelisk(obeliskTagUpdate, imu);
                }
            }

            double y = 0, x = 0, rx = 0;
            String autoStatus = "MANUAL";
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";

             //========== DRIVER PRIORITY: GAMEPAD1 > GAMEPAD2 ==========
            // Check for left joystick (forward/backward), right joystick (left/right strafe), or triggers (rotation) input
            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE;

            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;

            // ========== RB BUTTON ROTATION SEARCH FOR TAG 20 ==========
            // Note: Motor powers are set directly in the motor power section below (like LB/LT/RT)
            // This section just updates status for telemetry if rotation is active but status not set yet
            if (rotatingToTag20 && autoStatus.equals("MANUAL")) {
                double rotationDegrees = Math.toDegrees(rbCumulativeRotation);
                autoStatus = String.format("RB: SEARCHING FOR TAG 20... (%.0f° rotated)", rotationDegrees);
            }

            if (aimingAtTag20) {
                // ========== AIM AT APRILTAG ID 20 ==========
                AprilTagDetection detection = getAprilTagDetection(TARGET_TAG_20_ID);

                if (detection != null) {
                    // Tag found! Stop 360-degree search if active
                    searching360 = false;
                    
                    double yawError = detection.ftcPose.yaw;  // Positive = tag is to the right
                    double bearingError = detection.ftcPose.bearing;  // Horizontal angle to target
                    double range = detection.ftcPose.range;  // Distance in inches

                    // Check if aligned (both yaw and bearing within tolerance)
                    if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES &&
                            Math.abs(bearingError) < ANGLE_TOLERANCE_DEGREES) {
                        // Aligned, stop
                        x = 0;
                        y = 0;
                        rx = 0;
                        autoStatus = "AIMED AT TAG 20 ✓";
                    } else {
                        // Rotate to face tag (yaw correction) - proportional control
                        // Convert yaw error from degrees to power (scaled)
                        double rotationPower = Math.max(Math.min(yawError * 0.03, AUTO_ALIGN_ROTATION_SPEED), -AUTO_ALIGN_ROTATION_SPEED);
                        rx = rotationPower;
                        
                        // Strafe correction if bearing is large (tag is off to the side)
                        // Bearing tells us if tag is left or right of center
                        double strafeCorrection = 0;
                        if (Math.abs(bearingError) > 3.0) {
                            // Proportional strafe correction
                            strafeCorrection = -Math.max(Math.min(bearingError * 0.02, 0.3), -0.3);
                        }
                        x = strafeCorrection;
                        y = 0;
                        
                        autoStatus = String.format("AIMING AT TAG 20 (Yaw: %.1f°, Bear: %.1f°, Range: %.1f\")", 
                                yawError, bearingError, range);
                    }
                } else {
                    // Tag not detected
                    if (!searching360) {
                        // Start 360-degree search
                        searching360 = true;
                        searchStartHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                        lastSearchHeading = searchStartHeading;
                        cumulativeRotation = 0.0;
                        autoStatus = "STARTING 360° SEARCH...";
                    } else {
                        // Continue 360-degree search
                        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                        robotHeading = currentHeading;  // Update robot heading
                        
                        // Calculate incremental rotation (handle wraparound)
                        double headingDelta = currentHeading - lastSearchHeading;
                        
                        // Normalize to [-π, π]
                        while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
                        while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
                        
                        // Since we're rotating clockwise (positive rx), and IMU typically
                        // increases counterclockwise, negative delta means clockwise rotation
                        // Accumulate absolute value to track total rotation distance
                        cumulativeRotation += Math.abs(headingDelta);
                        
                        // Update last heading
                        lastSearchHeading = currentHeading;
                        
                        // Convert cumulative rotation to degrees
                        double rotationDegrees = Math.toDegrees(cumulativeRotation);
                        
                        // Check if we've completed 360 degrees
                        if (rotationDegrees >= (360.0 - SEARCH_COMPLETION_TOLERANCE)) {
                            // Completed 360 degrees, stop searching
                            searching360 = false;
                            cumulativeRotation = 0.0;
                            x = 0;
                            y = 0;
                            rx = 0;
                            autoStatus = "360° SEARCH COMPLETE - TAG NOT FOUND";
                        } else {
                            // Continue rotating clockwise
                            rx = SEARCH_ROTATION_SPEED;
                            x = 0;
                            y = 0;
                            autoStatus = String.format("SEARCHING 360°... (%.0f° / 360°)", rotationDegrees);
                        }
                    }
                }
            } else {
                // Not aiming at tag, reset search state
                if (searching360) {
                    searching360 = false;
                    cumulativeRotation = 0.0;
                }
            }
            
            if (autoNavigatingToZero) {
                // ========== AUTONOMOUS NAVIGATION TO (0,0) ==========
                // Update position from AprilTag during navigation for better accuracy
                AprilTagDetection navTag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                AprilTagDetection navObeliskTag = null;
                if (navTag20 != null) {
                    // Update position from Tag 20 if visible
                    calculatePositionFromTag(navTag20, TAG_20_X, TAG_20_Y, imu);
                } else {
                    // Fall back to obelisk tags if Tag 20 not visible
                    navObeliskTag = getAnyObeliskTag();
                    if (navObeliskTag != null) {
                        calculatePositionFromObelisk(navObeliskTag, imu);
                    }
                }
                
                double deltaX = TARGET_X - robotX;
                double deltaY = TARGET_Y - robotY;
                double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                double distanceInches = distanceToTarget * 12.0;
                double distanceFeet = distanceToTarget;

                if (distanceInches < POSITION_TOLERANCE_INCHES) {
                    // Reached target, stop immediately
                    x = 0;
                    y = 0;
                    rx = 0;
                    autoStatus = String.format("AT (0,0) ✓ (%.1f\" away)", distanceInches);
                } else {
                    // Calculate angle to target in field coordinates
                    // In FTC, Y is typically forward (positive = forward), X is right (positive = right)
                    // atan2(y, x) gives angle from positive x-axis
                    // But we need angle from positive y-axis (forward direction)
                    // So we use atan2(deltaX, deltaY) which gives angle from positive y-axis
                    double angleToTarget = Math.atan2(deltaX, deltaY);
                    
                    // Progressive speed control with multiple zones for smooth deceleration
                    double speed = AUTO_NAV_MAX_SPEED;
                    
                    if (distanceInches < 12.0) {
                        // Final braking zone (within 12 inches) - very slow
                        speed = AUTO_NAV_MIN_SPEED * 0.3;  // Very slow in final 12 inches
                    } else if (distanceInches < BRAKING_ZONE_INCHES) {
                        // Aggressive braking zone (12 to 18 inches)
                        double brakingRatio = (distanceInches - 12.0) / (BRAKING_ZONE_INCHES - 12.0);
                        speed = AUTO_NAV_MIN_SPEED * (0.3 + 0.4 * brakingRatio);  // 0.3-0.7x min speed
                    } else if (distanceFeet < BRAKING_DISTANCE_FEET) {
                        // Near braking zone (18 inches to 3 feet) - moderate braking
                        double brakingRatio = (distanceFeet - BRAKING_ZONE_INCHES / 12.0) / (BRAKING_DISTANCE_FEET - BRAKING_ZONE_INCHES / 12.0);
                        speed = AUTO_NAV_MIN_SPEED * 0.7 + (AUTO_NAV_MID_SPEED - AUTO_NAV_MIN_SPEED * 0.7) * brakingRatio;
                    } else if (distanceFeet < SLOWDOWN_DISTANCE_FEET) {
                        // Gradual slowdown zone (3 to 6 feet)
                        double slowdownRatio = (distanceFeet - BRAKING_DISTANCE_FEET) / (SLOWDOWN_DISTANCE_FEET - BRAKING_DISTANCE_FEET);
                        speed = AUTO_NAV_MID_SPEED + (AUTO_NAV_MAX_SPEED - AUTO_NAV_MID_SPEED) * slowdownRatio;
                    } else {
                        // Full speed zone (beyond 6 feet)
                        speed = AUTO_NAV_MAX_SPEED;
                    }
                    
                    // Additional slowdown when Tag 20 is visible (for better detection accuracy)
                    // BUT don't slow down too much - we need minimum movement speed
                    if (navTag20 != null) {
                        // Only reduce speed by 10% when Tag 20 is visible (was 20%)
                        // This keeps the robot moving while maintaining good detection
                        speed *= 0.9;  // 10% reduction when Tag 20 is visible for better detection
                    } else if (navObeliskTag == null) {
                        // No tags detected - slow down moderately to improve detection chances
                        speed *= 0.7;  // 30% reduction if no tags visible (was 50%)
                    }

                    // Ensure minimum speed is maintained - CRITICAL for movement
                    // Don't let speed get too low or robot won't move
                    double minSpeed;
                    if (distanceInches > 36.0) {
                        // Far from target - maintain higher minimum speed
                        minSpeed = AUTO_NAV_MID_SPEED;  // Higher minimum when far
                    } else if (distanceInches > 24.0) {
                        // Medium distance - moderate minimum speed
                        minSpeed = AUTO_NAV_MIN_SPEED * 1.5;
                    } else if (distanceInches > 12.0) {
                        // Close to target - allow slower movement
                        minSpeed = AUTO_NAV_MIN_SPEED;
                    } else {
                        // Very close to target - allow very slow movement
                        minSpeed = AUTO_NAV_MIN_SPEED * 0.6;
                    }
                    speed = Math.max(speed, minSpeed);  // Ensure minimum speed

                    // Convert to robot frame - calculate robot-relative movement direction
                    double heading = robotHeading;
                    // angleToTarget is in field coordinates, subtract heading to get robot-relative angle
                    double robotRelativeAngle = angleToTarget - heading;
                    // Normalize angle to [-PI, PI]
                    while (robotRelativeAngle > Math.PI) robotRelativeAngle -= 2 * Math.PI;
                    while (robotRelativeAngle < -Math.PI) robotRelativeAngle += 2 * Math.PI;
                    
                    // Calculate robot-relative movement (x = strafe, y = forward/back)
                    // For mecanum: x is strafe (positive = right), y is forward (positive = forward)
                    x = Math.sin(robotRelativeAngle) * speed;
                    y = Math.cos(robotRelativeAngle) * speed;
                    
                    // Heading correction to face target (reduced for smoother movement)
                    double headingError = angleToTarget - heading;
                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;
                    
                    // Reduce rotation speed significantly, especially near target
                    double rotationGain = 0.12;  // Reduced from 0.3 for smoother rotation (was 0.15)
                    if (distanceInches < 24.0) {
                        rotationGain = 0.06;  // Even slower rotation when close (was 0.08)
                    }
                    if (distanceInches < 12.0) {
                        rotationGain = 0.03;  // Very slow rotation when very close
                    }
                    rx = headingError * rotationGain;
                    
                    // Cap rotation speed to prevent overshooting
                    double maxRotationSpeed = 0.12;  // Reduced from 0.15
                    if (distanceInches < 12.0) {
                        maxRotationSpeed = 0.06;  // Very slow rotation when close
                    }
                    rx = Math.max(Math.min(rx, maxRotationSpeed), -maxRotationSpeed);
                    
                    autoStatus = String.format("NAV TO (0,0): %.1f\" @ %.1f%% | X:%.2f Y:%.2f", 
                            distanceInches, speed * 100, x, y);
                }
            } else if (navigatingToCustomTarget) {
                // ========== AUTONOMOUS NAVIGATION TO CUSTOM TARGET (X, Y, Z) ==========
                // Update position from AprilTag during navigation for better accuracy
                AprilTagDetection navTag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                AprilTagDetection navObeliskTag = null;
                if (navTag20 != null) {
                    // Update position from Tag 20 if visible
                    calculatePositionFromTag(navTag20, TAG_20_X, TAG_20_Y, imu);
                } else {
                    // Fall back to obelisk tags if Tag 20 not visible
                    navObeliskTag = getAnyObeliskTag();
                    if (navObeliskTag != null) {
                        calculatePositionFromObelisk(navObeliskTag, imu);
                    }
                }
                
                double deltaX = customTargetX - robotX;
                double deltaY = customTargetY - robotY;
                double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                double distanceInches = distanceToTarget * 12.0;
                double distanceFeet = distanceToTarget;

                if (distanceInches < POSITION_TOLERANCE_INCHES) {
                    // Reached target, stop immediately
                    x = 0;
                    y = 0;
                    rx = 0;
                    autoStatus = String.format("AT TARGET ✓ (%.1f\" away) | Z: %.1f\"", distanceInches, customTargetZ);
                } else {
                    // Calculate angle to target in field coordinates
                    double angleToTarget = Math.atan2(deltaX, deltaY);
                    
                    // Progressive speed control with multiple zones for smooth deceleration
                    double speed = AUTO_NAV_MAX_SPEED;
                    
                    if (distanceInches < 12.0) {
                        // Final braking zone (within 12 inches) - very slow
                        speed = AUTO_NAV_MIN_SPEED * 0.3;
                    } else if (distanceInches < BRAKING_ZONE_INCHES) {
                        // Aggressive braking zone (12 to 18 inches)
                        double brakingRatio = (distanceInches - 12.0) / (BRAKING_ZONE_INCHES - 12.0);
                        speed = AUTO_NAV_MIN_SPEED * (0.3 + 0.4 * brakingRatio);
                    } else if (distanceFeet < BRAKING_DISTANCE_FEET) {
                        // Near braking zone (18 inches to 3 feet) - moderate braking
                        double brakingRatio = (distanceFeet - BRAKING_ZONE_INCHES / 12.0) / (BRAKING_DISTANCE_FEET - BRAKING_ZONE_INCHES / 12.0);
                        speed = AUTO_NAV_MIN_SPEED * 0.7 + (AUTO_NAV_MID_SPEED - AUTO_NAV_MIN_SPEED * 0.7) * brakingRatio;
                    } else if (distanceFeet < SLOWDOWN_DISTANCE_FEET) {
                        // Gradual slowdown zone (3 to 6 feet)
                        double slowdownRatio = (distanceFeet - BRAKING_DISTANCE_FEET) / (SLOWDOWN_DISTANCE_FEET - BRAKING_DISTANCE_FEET);
                        speed = AUTO_NAV_MID_SPEED + (AUTO_NAV_MAX_SPEED - AUTO_NAV_MID_SPEED) * slowdownRatio;
                    } else {
                        // Full speed zone (beyond 6 feet)
                        speed = AUTO_NAV_MAX_SPEED;
                    }
                    
                    // Additional slowdown when Tag 20 is visible
                    if (navTag20 != null) {
                        speed *= 0.9;
                    } else if (navObeliskTag == null) {
                        speed *= 0.7;
                    }

                    // Ensure minimum speed is maintained
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
                    
                    autoStatus = String.format("NAV TO TARGET: %.1f\" @ %.1f%% | X:%.2f Y:%.2f Z:%.1f\"", 
                            distanceInches, speed * 100, x, y, customTargetZ);
                }
            } else if (rotatingToTag20) {
                // ========== RB BUTTON ROTATION SEARCH FOR TAG 20 ==========
                // This is already handled above, but ensure it doesn't get overridden
                // Values are already set in the earlier RB rotation block
            } else if (autoAligning) {
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
            } else {
                // ========== MANUAL CONTROL ==========
                // Dpad: movement in all 4 directions (up/down/left/right)
                // Right joystick: in-place rotation only
                // Priority: Gamepad1 > Gamepad2
                
                // Initialize movement and rotation to zero
                y = 0;
                x = 0;
                rx = 0;
                maxDrivePower = GAMEPAD1_MAX_POWER;
                activeDriver = "NONE";
                
                // Check Gamepad1 first (has priority)
                if (gamepad1Active) {
                    // JOYSTICK CONTROLS: Adjusted for motor setup (same mapping as dpad)
                    // Left joystick Y: up = forward, down = backward
                    // Right joystick X: left = strafe left, right = strafe right
                    // Motor setup requires: swapped x/y axes with inverted signs
                    
                    // Left joystick Y controls forward/backward movement
                    // Map to x axis (swapped) - negative x = forward (from dpad mapping)
                    // Gamepad Y is negative when pushed up, so left_stick_y directly maps to x
                    double leftStickY = currentGamepad1.left_stick_y;  // Negative when up
                    x = leftStickY;  // x controls forward/backward (swapped from standard)
                    
                    // Right joystick X controls left/right strafe
                    // Map to y axis (swapped) - negative y = left, positive y = right (corrected mapping)
                    // right_stick_x is negative when pushed left, positive when pushed right
                    double rightStickX = currentGamepad1.right_stick_x;  // Direct mapping (no inversion)
                    y = rightStickX;  // y controls left/right (swapped from standard)
                    
                    // Rotation control from triggers - DISABLED
                    // Note: LT and RT are now used for wheel control, rotation disabled
                    rx = 0;  // No rotation control
                    
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (gamepad2Active) {
                    // JOYSTICK CONTROLS: Adjusted for motor setup (same mapping as dpad)
                    // Left joystick Y: up = forward, down = backward
                    // Right joystick X: left = strafe left, right = strafe right
                    // Motor setup requires: swapped x/y axes with inverted signs
                    
                    // Left joystick Y controls forward/backward movement
                    // Map to x axis (swapped) - negative x = forward (from dpad mapping)
                    // Gamepad Y is negative when pushed up, so left_stick_y directly maps to x
                    double leftStickY = currentGamepad2.left_stick_y;  // Negative when up
                    x = leftStickY;  // x controls forward/backward (swapped from standard)
                    
                    // Right joystick X controls left/right strafe
                    // Map to y axis (swapped) - positive y = left, negative y = right (from dpad mapping)
                    // right_stick_x is negative when pushed left, so invert it
                    double rightStickX = -currentGamepad2.right_stick_x;  // Invert for left = positive y
                    y = rightStickX;  // y controls left/right (swapped from standard)
                    
                    // Rotation control from triggers - DISABLED
                    // Note: LT and RT are now used for wheel control, rotation disabled
                    rx = 0;  // No rotation control
                    
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                } else {
                    // No gamepad active - ensure all values are zero
                    x = 0;
                    y = 0;
                    rx = 0;
                }

                // Apply deadzone to all joystick inputs
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
            // Only reset if BACK is pressed alone (not with DPAD in launch mode)
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

            // ========== ROBOT-CENTRIC DRIVE (PREDICTABLE) ==========
            // Simple robot-centric drive: joystick controls movement
            // Left joystick Y: up = forward, down = backward
            // Right joystick X: left = strafe left, right = strafe right
            // LT (Left Trigger) = left rotation, RT (Right Trigger) = right rotation
            // No field-centric transformation - joystick direction = robot movement direction
            
            // Adjusted mapping for motor setup (same as dpad):
            // x = forward/backward (negative = forward, positive = backward) - from left joystick Y (swapped)
            // y = strafe left/right (positive = left, negative = right) - from right joystick X (swapped, inverted)
            // rx = rotation (negative = left rotation/counter-clockwise, positive = right rotation/clockwise) - from triggers
            // Left rotation: left side backward, right side forward (rx negative)
            // Right rotation: left side forward, right side backward (rx positive)

            // ========== MECANUM WHEEL POWER CALCULATION ==========
            // Standard mecanum wheel formula for robot-centric drive
            // Invert y for proper strafe direction to match motor wiring
            double frontLeftPower = -y + x + rx;
            double backLeftPower = -y - x + rx;  // Inverted y for proper strafe
            double frontRightPower = -y - x - rx;
            double backRightPower = -y + x - rx;  // Inverted y for proper strafe

            // Find the maximum absolute power to maintain proportional relationships
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            // Scale down proportionally if any power exceeds maxDrivePower
            // This preserves the motor power ratios while respecting the power limit
            if (maxPower > maxDrivePower) {
                double scale = maxDrivePower / maxPower;
                frontLeftPower *= scale;
                backLeftPower *= scale;
                frontRightPower *= scale;
                backRightPower *= scale;
            }

            // Set motor powers
            // RB Override: Rotate slowly until Tag 20 detected (takes highest priority)
            if (rotatingToTag20) {
                // Check for Tag 20 detection
                AprilTagDetection tag20Detection = getAprilTagDetection(TARGET_TAG_20_ID);
                
                if (tag20Detection != null) {
                    // Tag 20 detected! Stop rotation
                    rotatingToTag20 = false;
                    rbCumulativeRotation = 0.0;
                    autoStatus = "TAG 20 DETECTED ✓";
                    frontLeftMotor.setPower(0.0);
                    backLeftMotor.setPower(0.0);
                    frontRightMotor.setPower(0.0);
                    backRightMotor.setPower(0.0);
                } else {
                    // Tag not detected yet, continue rotating slowly clockwise
                    // Update rotation tracking
                    double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    robotHeading = currentHeading;
                    
                    // Calculate incremental rotation (handle wraparound)
                    double headingDelta = currentHeading - rbLastHeading;
                    while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
                    while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
                    rbCumulativeRotation += Math.abs(headingDelta);
                    rbLastHeading = currentHeading;
                    
                    // Update status
                    double rotationDegrees = Math.toDegrees(rbCumulativeRotation);
                    autoStatus = String.format("RB: SEARCHING FOR TAG 20... (%.0f° rotated)", rotationDegrees);
                    
                    // Rotate slowly clockwise using RT pattern
                    // Front left: forward, Back left: forward, Front right: backward, Back right: backward
                    frontLeftMotor.setPower(-RB_ROTATION_SPEED);  // Forward (negative = forward)
                    backLeftMotor.setPower(RB_ROTATION_SPEED);  // Forward (positive = forward)
                    frontRightMotor.setPower(-RB_ROTATION_SPEED);  // Backward (negative = backward)
                    backRightMotor.setPower(RB_ROTATION_SPEED);  // Backward (positive = backward)
                }
            }
            // LB/LT Override: Control all 4 wheels (takes priority after RB)
            // Front left: backward, Back left: backward, Front right: forward, Back right: forward
            // LB (bumper) is binary (full power), LT (trigger) is proportional (0.0 to 1.0)
            else if (currentGamepad1.left_bumper || currentGamepad2.left_bumper) {
                // LB: Full power (binary)
                frontLeftMotor.setPower(1.0);  // Backward at full power (positive = backward for this motor)
                backLeftMotor.setPower(-1.0);  // Backward at full power (negative = backward for this motor)
                frontRightMotor.setPower(1.0);  // Forward at full power (positive = forward for this motor)
                backRightMotor.setPower(-1.0);  // Forward at full power (negative = forward for this motor)
            } else if (currentGamepad1.left_trigger > JOYSTICK_DEADZONE || currentGamepad2.left_trigger > JOYSTICK_DEADZONE) {
                // LT: Proportional speed (uses actual trigger value 0.0 to 1.0)
                double ltPower = Math.max(currentGamepad1.left_trigger, currentGamepad2.left_trigger);
                frontLeftMotor.setPower(ltPower);  // Backward proportional to trigger (positive = backward)
                backLeftMotor.setPower(-ltPower);  // Backward proportional to trigger (negative = backward)
                frontRightMotor.setPower(ltPower);  // Forward proportional to trigger (positive = forward)
                backRightMotor.setPower(-ltPower);  // Forward proportional to trigger (negative = forward)
            } else if (currentGamepad1.right_trigger > JOYSTICK_DEADZONE || currentGamepad2.right_trigger > JOYSTICK_DEADZONE) {
                // RT Override: Control all 4 wheels with different pattern (proportional)
                // Front left: forward, Back left: forward, Front right: backward, Back right: backward
                double rtPower = Math.max(currentGamepad1.right_trigger, currentGamepad2.right_trigger);
                frontLeftMotor.setPower(-rtPower);  // Forward proportional to trigger (negative = forward)
                backLeftMotor.setPower(rtPower);  // Forward proportional to trigger (positive = forward)
                frontRightMotor.setPower(-rtPower);  // Backward proportional to trigger (negative = backward)
                backRightMotor.setPower(rtPower);  // Backward proportional to trigger (positive = backward)
            } else {
                // Normal mecanum drive (wheels locked during launch only)
                if (wheelsLocked) {
                    // Keep wheels locked at 0 during launch
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
            // Toggle intake speed with START button
            if ((currentGamepad1.start && !previousGamepad1.start) ||
                    (currentGamepad2.start && !previousGamepad2.start)) {
                intakeHalfSpeed = !intakeHalfSpeed;
            }

            // B button - toggle intake forward
            if ((currentGamepad1.b && !previousGamepad1.b) ||
                    (currentGamepad2.b && !previousGamepad2.b)) {
                intakeForwardActive = !intakeForwardActive;
                if (intakeForwardActive) intakeReverseActive = false;
            }

            // A button - Stop all motors (intake and ramp immediately, launch after 0.5s)
            if ((currentGamepad1.a && !previousGamepad1.a) ||
                    (currentGamepad2.a && !previousGamepad2.a)) {
                // Stop intake and ramp immediately
                intakeMotor.setPower(0);
                rampMotor.setPower(0);
                intakeForwardActive = false;
                intakeReverseActive = false;
                rampMotorActive = false;
                
                // Stop launch motor after 0.5 seconds
                sleep(500);
                launchMotor.setPower(0);
                launchMotorActive = false;
                launchMotorReverse = false;
                wheelsLocked = false;  // Unlock wheels when launch is stopped
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

            // ========== LAUNCH MOTOR CONTROL (BOTH GAMEPADS) ==========
            // Note: Dpad is now used for robot movement, so launch motor controls removed from dpad
            // Launch motor can still be controlled via X button (shoot sequence)
            
            // Update launch motor power based on state
            double launchPower = 0;
            String launchStatus = "STOPPED";
            if (launchMotorReverse) {
                // Reverse mode (Y button) - don't override
                launchPower = -0.7;
                launchMotor.setPower(launchPower);
                launchStatus = "REVERSE";
            } else if (launchMotorActive) {
                launchPower = runtimeLaunchMotorPower;  // Use runtime adjustable power
                launchMotor.setPower(launchPower);
                launchStatus = "RUNNING";
            } else {
                launchMotor.setPower(0);
            }
            
            // ========== SHOOT SEQUENCE (X BUTTON) ==========
            // X button - Complete shoot sequence: stop intake, start launch (if not running), wait, then feed
            if ((currentGamepad1.x && !previousGamepad1.x) ||
                    (currentGamepad2.x && !previousGamepad2.x)) {
                // Stop intake and ramp
                intakeMotor.setPower(0);
                rampMotor.setPower(0);
                
                // Disable reverse mode if active
                launchMotorReverse = false;
                
                // Lock wheels during launch to prevent movement
                wheelsLocked = true;
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                
                // Start launch motor if not already running
                if (!launchMotorActive) {
                    launchMotorActive = true;
                    launchMotor.setPower(runtimeLaunchMotorPower);  // Use runtime adjustable power
                    sleep(3000);  // Wait for launch motor to spin up
                } else {
                    // Already running, just wait a moment
                    sleep(500);
                }
                
                // Start feeding
                intakeMotor.setPower(1.0);
                rampMotor.setPower(-RAMP_MOTOR_POWER * 0.4);
            }
            
            // Unlock wheels when launch is done
            if (wheelsLocked && !launchMotorActive && !launchMotorReverse) {
                wheelsLocked = false;
            }
            
            // ========== BACK + DPAD UP/DOWN - DYNAMIC LAUNCH MOTOR SPEED ADJUSTMENT (LAUNCH MODE ONLY) ==========
            // Only works when launch motor is active (launch mode)
            // BACK + DPAD UP: increase speed by 0.05 (max 0.75)
            // BACK + DPAD DOWN: decrease speed by 0.05 (min 0.5)
            if (launchMotorActive && !launchMotorReverse) {
                // Check for BACK + DPAD UP combination
                boolean backDpadUpG1 = currentGamepad1.back && currentGamepad1.dpad_up;
                boolean backDpadUpG2 = currentGamepad2.back && currentGamepad2.dpad_up;
                boolean backDpadUpPrevG1 = previousGamepad1.back && previousGamepad1.dpad_up;
                boolean backDpadUpPrevG2 = previousGamepad2.back && previousGamepad2.dpad_up;
                
                // Check for BACK + DPAD DOWN combination
                boolean backDpadDownG1 = currentGamepad1.back && currentGamepad1.dpad_down;
                boolean backDpadDownG2 = currentGamepad2.back && currentGamepad2.dpad_down;
                boolean backDpadDownPrevG1 = previousGamepad1.back && previousGamepad1.dpad_down;
                boolean backDpadDownPrevG2 = previousGamepad2.back && previousGamepad2.dpad_down;
                
                // Increase speed (BACK + DPAD UP)
                if ((backDpadUpG1 && !backDpadUpPrevG1) || (backDpadUpG2 && !backDpadUpPrevG2)) {
                    runtimeLaunchMotorPower += 0.05;
                    if (runtimeLaunchMotorPower > 0.75) {
                        runtimeLaunchMotorPower = 0.75;  // Clamp to max 0.75
                    }
                    
                    // Round to 2 decimal places to avoid floating point issues
                    runtimeLaunchMotorPower = Math.round(runtimeLaunchMotorPower * 100.0) / 100.0;
                    
                    // Update launch motor power immediately
                    launchMotor.setPower(runtimeLaunchMotorPower);
                    
                    telemetry.addLine("Launch Motor Speed: " + String.format("%.2f", runtimeLaunchMotorPower) + " (INCREASED)");
                }
                
                // Decrease speed (BACK + DPAD DOWN)
                if ((backDpadDownG1 && !backDpadDownPrevG1) || (backDpadDownG2 && !backDpadDownPrevG2)) {
                    runtimeLaunchMotorPower -= 0.05;
                    if (runtimeLaunchMotorPower < 0.5) {
                        runtimeLaunchMotorPower = 0.5;  // Clamp to min 0.5
                    }
                    
                    // Round to 2 decimal places to avoid floating point issues
                    runtimeLaunchMotorPower = Math.round(runtimeLaunchMotorPower * 100.0) / 100.0;
                    
                    // Update launch motor power immediately
                    launchMotor.setPower(runtimeLaunchMotorPower);
                    
                    telemetry.addLine("Launch Motor Speed: " + String.format("%.2f", runtimeLaunchMotorPower) + " (DECREASED)");
                }
            }
            
            // Y button - Stop launch and start intake (for pickup)
            if ((currentGamepad1.y && !previousGamepad1.y) ||
                    (currentGamepad2.y && !previousGamepad2.y)) {
                launchMotorActive = false;
                launchMotorReverse = true;  // Enable reverse mode
                wheelsLocked = false;  // Unlock wheels
                launchMotor.setPower(0);  // Stop launch motor
                sleep(500);  // Wait 500ms
                intakeMotor.setPower(1.0);  // Start intake motor (power = 1.0)
                rampMotor.setPower(-0.5);  // Start ramp motor in reverse direction (power = 0.5)
                launchMotor.setPower(-0.8);  // Start launch motor in reverse (power = -0.7)
            }

            // ========== ENHANCED TELEMETRY ==========
            telemetry.addLine("========== MATCH MODE ==========");
            telemetry.addLine();
            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotHeading));
            if (!autoNavigatingToZero && !navigatingToCustomTarget && !autoAligning && !aimingAtTag20 && !rotatingToTag20) {
                telemetry.addData("Drive Mode", "ROBOT-CENTRIC (Dpad + Right Stick)");
                telemetry.addData("Movement Input", "X: %.2f (Dpad L/R), Y: %.2f (Dpad U/D), RX: %.2f (Right Stick)", x, y, rx);
                telemetry.addData("Motor Powers", "FL: %.2f, BL: %.2f, FR: %.2f, BR: %.2f", 
                        frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            }
            telemetry.addData("Active Driver", activeDriver);
            telemetry.addLine();
            telemetry.addData("Drive Mode", autoStatus);
            // Debug: Show RB button status
            boolean rbDebug = currentGamepad1.right_bumper || currentGamepad2.right_bumper;
            telemetry.addData("RB Button", rbDebug ? "PRESSED" : "not pressed");
            if (autoAligning) {
                telemetry.addData(">>> AUTO-ALIGN", "ACTIVE <<<");
            }
            if (rotatingToTag20) {
                telemetry.addData(">>> RB: ROTATING TO FIND TAG 20", "ACTIVE <<<");
                double rotationDegrees = Math.toDegrees(rbCumulativeRotation);
                telemetry.addData("Rotation Progress", "%.0f° rotated", rotationDegrees);
                telemetry.addData("Rotation Speed", "%.2f", RB_ROTATION_SPEED);
                
                AprilTagDetection tag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                if (tag20 != null) {
                    telemetry.addLine();
                    telemetry.addData(">>> TAG 20 DETECTED! ✓", "");
                    telemetry.addData("Tag 20 Range", "%.1f inches", tag20.ftcPose.range);
                    telemetry.addData("Tag 20 X", "%.1f inches", tag20.ftcPose.x);
                    telemetry.addData("Tag 20 Y", "%.1f inches", tag20.ftcPose.y);
                    telemetry.addData("Tag 20 Z", "%.1f inches", tag20.ftcPose.z);
                    telemetry.addData("Tag 20 Yaw", "%.1f°", tag20.ftcPose.yaw);
                    telemetry.addData("Tag 20 Bearing", "%.1f°", tag20.ftcPose.bearing);
                    telemetry.addData("Tag 20 Elevation", "%.1f°", tag20.ftcPose.elevation);
                } else {
                    telemetry.addData("Tag 20 Status", "NOT DETECTED - ROTATING...");
                    // Show what tags ARE detected for debugging
                    List<AprilTagDetection> allDetections = aprilTag.getDetections();
                    if (allDetections.size() > 0) {
                        telemetry.addData("Detected Tags", "%d tags visible (not Tag 20)", allDetections.size());
                        for (AprilTagDetection det : allDetections) {
                            telemetry.addData("  Tag ID", "%d", det.id);
                        }
                    } else {
                        telemetry.addData("No Tags", "Visible in camera");
                    }
                }
            }
            if (aimingAtTag20) {
                telemetry.addData(">>> AIMING AT TAG 20", "ACTIVE <<<");
                if (searching360) {
                    double rotationDegrees = Math.toDegrees(cumulativeRotation);
                    telemetry.addData("Search Progress", "%.0f° / 360°", Math.min(rotationDegrees, 360.0));
                }
                AprilTagDetection tag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                if (tag20 != null) {
                    telemetry.addData("Tag 20 Range", "%.1f inches", tag20.ftcPose.range);
                    telemetry.addData("Tag 20 X", "%.1f inches", tag20.ftcPose.x);
                    telemetry.addData("Tag 20 Y", "%.1f inches", tag20.ftcPose.y);
                    telemetry.addData("Tag 20 Z", "%.1f inches", tag20.ftcPose.z);
                    telemetry.addData("Tag 20 Yaw", "%.1f°", tag20.ftcPose.yaw);
                    telemetry.addData("Tag 20 Bearing", "%.1f°", tag20.ftcPose.bearing);
                    telemetry.addData("Tag 20 Elevation", "%.1f°", tag20.ftcPose.elevation);
                    
                    // Camera angle diagnostics
                    double elevation = tag20.ftcPose.elevation;
                    if (Math.abs(elevation) > 30) {
                        telemetry.addLine("⚠ CAMERA ANGLE WARNING:");
                        if (elevation > 30) {
                            telemetry.addData("  → Camera pointing too HIGH", "Lower camera angle");
                        } else {
                            telemetry.addData("  → Camera pointing too LOW", "Raise camera angle");
                        }
                    }
                } else {
                    if (searching360) {
                        telemetry.addData("Tag 20", "NOT DETECTED - SEARCHING");
                    } else {
                        telemetry.addData("Tag 20", "NOT DETECTED");
                    }
                    // Show what tags ARE detected for debugging
                    List<AprilTagDetection> allDetections = aprilTag.getDetections();
                    if (allDetections.size() > 0) {
                        telemetry.addData("Detected Tags", "%d tags visible", allDetections.size());
                        for (AprilTagDetection det : allDetections) {
                            telemetry.addData("  Tag ID", "%d", det.id);
                        }
                        telemetry.addLine("⚠ Tag 20 not in view - check camera angle");
                    } else {
                        telemetry.addData("No Tags", "Visible in camera");
                        telemetry.addLine("⚠ No tags detected - check:");
                        telemetry.addLine("  1. Camera angle (should be ~horizontal)");
                        telemetry.addLine("  2. Camera field of view");
                        telemetry.addLine("  3. Lighting conditions");
                    }
                }
            }
            if (navigatingToCustomTarget) {
                telemetry.addData(">>> NAV TO CUSTOM TARGET", "ACTIVE <<<");
                telemetry.addData("Current Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                double distToTarget = Math.sqrt((customTargetX - robotX) * (customTargetX - robotX) + 
                                               (customTargetY - robotY) * (customTargetY - robotY)) * 12.0;
                telemetry.addData("Distance to Target", "%.1f inches", distToTarget);
                telemetry.addData("Delta", "X: %.2f ft, Y: %.2f ft", (customTargetX - robotX), (customTargetY - robotY));
                telemetry.addData("Target Position", "X: %.2f ft, Y: %.2f ft, Z: %.1f in", customTargetX, customTargetY, customTargetZ);
                
                // Show which tag is being used for positioning
                AprilTagDetection telemetryTag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                AprilTagDetection telemetryObeliskTag = getAnyObeliskTag();
                if (telemetryTag20 != null) {
                    telemetry.addData("Using Tag", "Tag 20 for position");
                    telemetry.addData("Tag 20 Range", "%.1f inches", telemetryTag20.ftcPose.range);
                    telemetry.addData("Tag 20 X, Y, Z", "%.1f, %.1f, %.1f in", 
                            telemetryTag20.ftcPose.x, telemetryTag20.ftcPose.y, telemetryTag20.ftcPose.z);
                } else if (telemetryObeliskTag != null) {
                    telemetry.addData("Using Tag", "Obelisk tag for position");
                } else {
                    telemetry.addData("Position Update", "No tags visible");
                }
            }
            if (autoNavigatingToZero) {
                telemetry.addData(">>> NAV TO (0,0)", "ACTIVE <<<");
                telemetry.addData("Current Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                double distToTarget = Math.sqrt((TARGET_X - robotX) * (TARGET_X - robotX) + 
                                               (TARGET_Y - robotY) * (TARGET_Y - robotY)) * 12.0;
                telemetry.addData("Distance to (0,0)", "%.1f inches", distToTarget);
                telemetry.addData("Delta", "X: %.2f ft, Y: %.2f ft", (TARGET_X - robotX), (TARGET_Y - robotY));
                telemetry.addData("Target Position", "(0, 0) - Field Center");
                
                // Show which tag is being used for positioning
                AprilTagDetection telemetryTag20 = getAprilTagDetection(TARGET_TAG_20_ID);
                AprilTagDetection telemetryObeliskTag = getAnyObeliskTag();
                if (telemetryTag20 != null) {
                    telemetry.addData("Using Tag", "Tag 20 for position");
                    telemetry.addData("Tag 20 Range", "%.1f inches", telemetryTag20.ftcPose.range);
                    telemetry.addData("Tag 20 Bearing", "%.1f°", telemetryTag20.ftcPose.bearing);
                } else if (telemetryObeliskTag != null) {
                    telemetry.addData("Using Tag", "Obelisk tag %d for position", telemetryObeliskTag.id);
                } else {
                    telemetry.addData("Using Tag", "NO TAGS - Position may be inaccurate");
                }
                telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotHeading));
                telemetry.addData("Command", "X: %.3f, Y: %.3f, RX: %.3f", x, y, rx);
            }
            telemetry.addLine();
            telemetry.addData("Intake", intakeStatus);
            telemetry.addData("Launch", launchStatus);
            telemetry.addData("Launch Motor Speed", "%.2f", launchPower);
            if (launchMotorActive && !launchMotorReverse) {
                telemetry.addData("Launch Motor Speed (Target)", "%.2f (BACK+DPAD UP/DOWN to adjust)", runtimeLaunchMotorPower);
            } else {
                telemetry.addData("Launch Motor Speed (Target)", "%.2f", runtimeLaunchMotorPower);
            }
            telemetry.addLine();
            if (slowMode) {
                telemetry.addData("Slow Mode", "ACTIVE (30%)");
            }
            telemetry.addLine();
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
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
        // Calculate position from obelisk tag at known position (OBELISK_X, OBELISK_Y)
        calculatePositionFromTag(obeliskTag, OBELISK_X, OBELISK_Y, imu);
    }
    
    private void calculatePositionFromTag(AprilTagDetection tag, double tagX, double tagY, IMU imu) {
        // Calculate robot position from any AprilTag at known field position
        // tagX, tagY = known field position of the tag (in feet)
        // AprilTag detection gives us range, bearing, and yaw

        // Range is distance from camera to tag (in inches, convert to feet)
        double rangeToTag = tag.ftcPose.range / 12.0;

        // Bearing is the horizontal angle to the tag (degrees, convert to radians)
        double bearingToTag = Math.toRadians(tag.ftcPose.bearing);

        // Get current robot heading from IMU
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate absolute angle from robot to tag
        double absoluteAngleToTag = currentHeading + bearingToTag;

        // Calculate robot position relative to tag
        // The robot is 'rangeToTag' away at angle 'absoluteAngleToTag' from tag
        // Note: We don't update robotHeading here - it should come from IMU for consistency
        robotX = tagX - rangeToTag * Math.sin(absoluteAngleToTag);
        robotY = tagY - rangeToTag * Math.cos(absoluteAngleToTag);
        // Don't update robotHeading here - keep it from IMU for field-centric drive consistency

        // Note: This assumes camera is at robot center. If camera is offset from
        // robot center, you'll need to add offset compensation here.
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < JOYSTICK_DEADZONE) {
            return 0;
        }
        return value;
    }
}