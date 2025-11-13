package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
// yummy yummy cheesecake for our scrumptious souls. i will sell my organs for cheesecake. i am sorry, i was possessed by the demons.

/**
 * MECANUM WHEEL PROGRAMMING GUIDE
 * ================================
 * 
 * HOW MECANUM WHEELS WORK:
 * - Mecanum wheels have rollers mounted at 45-degree angles
 * - By spinning wheels in different combinations, the robot can move in any direction
 * - The key is combining three inputs: forward/backward, strafe, and rotation
 * 
 * BASIC MECANUM DRIVE FORMULA:
 * - Front Left  = y + x + rx
 * - Front Right = y - x - rx
 * - Back Left   = y - x + rx
 * - Back Right  = y + x - rx
 * 
 * WHERE:
 * - y  = Forward/Backward (-1.0 to +1.0)
 * - x  = Strafe Left/Right (-1.0 to +1.0)
 * - rx = Rotation (-1.0 to +1.0)
 * 
 * EXAMPLES:
 * - Move Forward: y=1, x=0, rx=0 → All wheels = 1 (all forward)
 * - Strafe Right: y=0, x=1, rx=0 → FL=1, FR=-1, BL=-1, BR=1
 * - Rotate CW:    y=0, x=0, rx=1 → Left=1, Right=-1
 * 
 * IMPORTANT STEPS:
 * 1. Set motor directions correctly (left=FORWARD, right=REVERSE typically)
 * 2. Calculate wheel powers using the formula above
 * 3. Normalize powers if any exceed 1.0 (maintains direction and ratios)
 * 4. Apply power limits and clipping
 * 5. Set motor powers
 * 
 * TROUBLESHOOTING:
 * - If robot moves wrong direction: Reverse motor directions
 * - If strafing doesn't work: Check wheel roller orientation
 * - If movement is jerky: Add deadzone to joystick inputs
 * - If robot drifts: Consider field-centric drive (see code comments)
 */

@TeleOp
public class TestTeleOp01 extends LinearOpMode {
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_HALF_POWER = 0.5;
    private static final double LAUNCH_MOTOR_POWER = 1.0;
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

    // ========== APRILTAG CONFIGURATION ==========
    private static final int RED_APRILTAG_ID = 24;
    private static final int BLUE_APRILTAG_ID = 20;

    // ========== STATE VARIABLES ==========
    private enum Alliance {NONE, RED, BLUE}

    private Alliance selectedAlliance = Alliance.NONE;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========== APRILTAG AUTO-ALIGNMENT STATE ==========
    private boolean autoAligning = false;
    
    // ========== WHEEL TEST MODE ==========
    private boolean wheelTestMode = false;
    private static final double TEST_WHEEL_POWER = 0.5;

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

        // ========== SET MECANUM MOTOR DIRECTIONS ==========
        // Mecanum wheels require specific motor directions for proper movement.
        // Typical configuration:
        // - Left side motors: FORWARD (positive power = forward movement)
        // - Right side motors: REVERSE (positive power = forward movement)
        // This setup allows the mecanum formula to work correctly.
        // If your robot moves incorrectly, you may need to reverse these directions.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Store previous gamepad states
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // ========== WHEEL TEST MODE TOGGLE (BOTH BUMPERS PRESSED) ==========
            if ((currentGamepad1.left_bumper && currentGamepad1.right_bumper && 
                 !(previousGamepad1.left_bumper && previousGamepad1.right_bumper)) ||
                (currentGamepad2.left_bumper && currentGamepad2.right_bumper && 
                 !(previousGamepad2.left_bumper && previousGamepad2.right_bumper))) {
                wheelTestMode = !wheelTestMode;
                if (wheelTestMode) {
                    autoAligning = false; // Disable auto-align in test mode
                }
            }
            
            // ========== ALLIANCE SELECTION (BOTH GAMEPADS) ==========
            // Only allow alliance selection when not in test mode
            if (!wheelTestMode) {
                if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper && !currentGamepad1.right_bumper) ||
                        (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && !currentGamepad2.right_bumper)) {
                    selectedAlliance = Alliance.RED;
                } else if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper && !currentGamepad1.left_bumper) ||
                        (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && !currentGamepad2.left_bumper)) {
                    selectedAlliance = Alliance.BLUE;
                }
            }

            // ========== LEFT TRIGGER - APRILTAG AUTO-ALIGNMENT (BOTH GAMEPADS) ==========
            // Disabled in test mode
            if (!wheelTestMode) {
                if (currentGamepad1.left_trigger > 0.5 || currentGamepad2.left_trigger > 0.5) {
                    if (selectedAlliance != Alliance.NONE) {
                        autoAligning = true;
                    } else {
                        autoAligning = false;
                    }
                }
            }

            // Check for driver override from either gamepad
            boolean driverOverride = Math.abs(currentGamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.left_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE;

            if (driverOverride && autoAligning) {
                autoAligning = false;
            }

            double y = 0, x = 0, rx = 0;
            String autoStatus = "MANUAL";
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";

            // ========== DRIVER PRIORITY: GAMEPAD1 > GAMEPAD2 ==========
//            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
//                    Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
//                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
//                    currentGamepad1.dpad_up || currentGamepad1.dpad_down ||
//                    currentGamepad1.dpad_left || currentGamepad1.dpad_right;

//            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_x) > JOYSTICK_DEADZONE ||
//                    Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
//                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
//                    currentGamepad2.dpad_up || currentGamepad2.dpad_down ||
//                    currentGamepad2.dpad_left || currentGamepad2.dpad_right;

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
                        if (selectedAlliance == Alliance.RED) {
                            rx = AUTO_ALIGN_ROTATION_SPEED;
                        } else {
                            rx = -AUTO_ALIGN_ROTATION_SPEED;
                        }
                        autoStatus = String.format("ALIGNING (%.1f°)", yawError);
                    }
                } else {
                    if (selectedAlliance == Alliance.RED) {
                        rx = AUTO_ALIGN_ROTATION_SPEED * SLOW_MODE_MULTIPLIER;
                    } else {
                        rx = -AUTO_ALIGN_ROTATION_SPEED * SLOW_MODE_MULTIPLIER;
                    }
                    autoStatus = "SEARCHING FOR TAG...";
                }

                // Allow manual translation during auto-alignment
                if (GAMEPAD1_ACTIVE) {
                    y = -currentGamepad1.left_stick_y;
                    x = currentGamepad1.left_stick_x;
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (GAMEPAD2_ACTIVE) {
                    y = -currentGamepad2.left_stick_y;
                    x = currentGamepad2.left_stick_x;
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                }

                // Apply deadzone
                y = applyDeadzone(y);
                x = applyDeadzone(x);
            } else {
                // ========== MANUAL CONTROL ==========
                if (GAMEPAD1_ACTIVE) {
                    y = -currentGamepad1.left_stick_y;
                    x = currentGamepad1.left_stick_x;
                    rx = currentGamepad1.right_stick_x;
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (GAMEPAD2_ACTIVE) {
                    y = -currentGamepad2.left_stick_y;
                    x = currentGamepad2.left_stick_x;
                    rx = currentGamepad2.right_stick_x;
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                }

                // Apply deadzone
                y = applyDeadzone(y);
                x = applyDeadzone(x);
                rx = applyDeadzone(rx);

                // Toggle slow mode with Y button (either gamepad)
                if ((currentGamepad1.y && !previousGamepad1.y) ||
                        (currentGamepad2.y && !previousGamepad2.y)) {
                    slowMode = !slowMode;
                }

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
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // ========== WHEEL POWER VARIABLES (DECLARED FOR TELEMETRY) ==========
            double frontLeftPower = 0;
            double frontRightPower = 0;
            double backLeftPower = 0;
            double backRightPower = 0;

            // ========== WHEEL TEST MODE ==========
            // In test mode, control individual wheels with joysticks
            if (wheelTestMode) {
                // Use gamepad1 or gamepad2 for test mode
                Gamepad testGamepad = GAMEPAD1_ACTIVE ? currentGamepad1 : currentGamepad2;
                
                // Left stick Y: Front Left wheel (push forward = positive, pull back = negative)
                double frontLeftInput = -testGamepad.left_stick_y;
                // Left stick X: Front Right wheel (push right = positive, push left = negative)
                double frontRightInput = testGamepad.left_stick_x;
                // Right stick Y: Back Left wheel (push forward = positive, pull back = negative)
                double backLeftInput = -testGamepad.right_stick_y;
                // Right stick X: Back Right wheel (push right = positive, push left = negative)
                double backRightInput = testGamepad.right_stick_x;
                
                // Apply deadzone and scale to test power
                frontLeftPower = applyDeadzone(frontLeftInput) * TEST_WHEEL_POWER;
                frontRightPower = applyDeadzone(frontRightInput) * TEST_WHEEL_POWER;
                backLeftPower = applyDeadzone(backLeftInput) * TEST_WHEEL_POWER;
                backRightPower = applyDeadzone(backRightInput) * TEST_WHEEL_POWER;
                
                // Apply test mode powers directly
                frontLeftMotor.setPower(Range.clip(frontLeftPower, -1.0, 1.0));
                frontRightMotor.setPower(Range.clip(frontRightPower, -1.0, 1.0));
                backLeftMotor.setPower(Range.clip(backLeftPower, -1.0, 1.0));
                backRightMotor.setPower(Range.clip(backRightPower, -1.0, 1.0));
                
                // Skip normal drive calculations in test mode
                // Continue to telemetry section
            } else {
                // ========== MECANUM DRIVE POWER CALCULATION ==========
                // MECANUM WHEELS EXPLANATION:
                // Mecanum wheels have rollers mounted at 45-degree angles.
                // By spinning wheels in different combinations, the robot can:
                // - Move forward/backward (all wheels same direction)
                // - Strafe left/right (diagonal wheels spin opposite)
                // - Rotate (left and right sides spin opposite)
                // - Combine all three movements simultaneously
                //
                // WHEEL ORIENTATION (typical setup):
                // Front Left:  Rollers pointing from top-left to bottom-right  (\)
                // Front Right: Rollers pointing from top-right to bottom-left  (/)
                // Back Left:   Rollers pointing from top-right to bottom-left  (/)
                // Back Right:  Rollers pointing from top-left to bottom-right  (\)
                //
                // INPUT VALUES:
                // y  = Forward/Backward: -1.0 (back) to +1.0 (forward)
                // x  = Strafe Left/Right: -1.0 (left) to +1.0 (right)
                // rx = Rotation: -1.0 (counter-clockwise) to +1.0 (clockwise)
                //
                // MECANUM DRIVE FORMULA:
                // Each wheel power is a combination of all three inputs:
                frontLeftPower = y + x + rx;   // Forward + Strafe Right + Rotate Clockwise
                frontRightPower = y - x - rx;  // Forward - Strafe Right - Rotate Clockwise
                backLeftPower = y - x + rx;    // Forward - Strafe Right + Rotate Clockwise
                backRightPower = y + x - rx;   // Forward + Strafe Right - Rotate Clockwise
                //
                // HOW IT WORKS:
                // - Pure Forward (y=1, x=0, rx=0): All wheels spin forward
                // - Pure Strafe Right (y=0, x=1, rx=0): FL and BR forward, FR and BL backward
                // - Pure Rotate (y=0, x=0, rx=1): Left wheels forward, right wheels backward
                // - Combinations: Add all three movements together
                
                // ========== FIELD-CENTRIC DRIVE (OPTIONAL) ==========
                // Uncomment below to enable field-centric drive (robot moves relative to field, not robot)
                // Field-centric means: Forward always moves toward opponent wall, regardless of robot rotation
                /*
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                // Rotate the input vector by the negative of the robot's heading
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                // Optional: Strafe correction factor (adjust if strafing is slower than forward)
                rotX = rotX * 1.1;  // Strafe correction (adjust based on testing)
                // Use rotated values in mecanum formula
                frontLeftPower = rotY + rotX + rx;
                frontRightPower = rotY - rotX - rx;
                backLeftPower = rotY - rotX + rx;
                backRightPower = rotY + rotX - rx;
                */

                // ========== POWER NORMALIZATION ==========
                // WHY NORMALIZE?
                // When combining y, x, and rx, some wheel powers can exceed 1.0.
                // Example: y=1.0, x=1.0, rx=1.0 → frontLeftPower = 3.0 (exceeds max!)
                // We must normalize to keep all powers between -1.0 and 1.0 while
                // preserving the correct direction and relative speeds.
                //
                // HOW IT WORKS:
                // 1. Find the maximum absolute power value
                // 2. If it exceeds 1.0, scale all powers down proportionally
                // 3. This maintains the correct movement direction and ratios
                double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                maxPower = Math.max(maxPower, Math.abs(frontRightPower));
                maxPower = Math.max(maxPower, Math.abs(backRightPower));

                // Normalize if any power exceeds 1.0 (preserves direction and ratios)
                if (maxPower > 1.0) {
                    double scale = 1.0 / maxPower;  // Calculate scaling factor
                    // Scale all powers by the same factor to maintain ratios
                    frontLeftPower *= scale;
                    backLeftPower *= scale;
                    frontRightPower *= scale;
                    backRightPower *= scale;
                }

                // Apply driver-specific power limit (multiplier)
                frontLeftPower *= maxDrivePower;
                backLeftPower *= maxDrivePower;
                frontRightPower *= maxDrivePower;
                backRightPower *= maxDrivePower;

                // Apply range clipping as final safety
                frontLeftMotor.setPower(Range.clip(frontLeftPower, -1.0, 1.0));
                backLeftMotor.setPower(Range.clip(backLeftPower, -1.0, 1.0));
                frontRightMotor.setPower(Range.clip(frontRightPower, -1.0, 1.0));
                backRightMotor.setPower(Range.clip(backRightPower, -1.0, 1.0));
            }

            // ========== INTAKE CONTROL (BOTH GAMEPADS) ==========
            // Disabled in test mode
            if (!wheelTestMode) {
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

                // A button - toggle intake reverse
                if ((currentGamepad1.a && !previousGamepad1.a) ||
                        (currentGamepad2.a && !previousGamepad2.a)) {
                    intakeReverseActive = !intakeReverseActive;
                    if (intakeReverseActive) intakeForwardActive = false;
                }
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
            // Disabled in test mode
            if (!wheelTestMode) {
                if ((currentGamepad1.x && !previousGamepad1.x) ||
                        (currentGamepad2.x && !previousGamepad2.x)) {
                    launchMotorActive = !launchMotorActive;
                    sleep(500);
                    rampMotor.setPower(-RAMP_MOTOR_POWER);
                }
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

            // ========== ENHANCED TELEMETRY ==========
            if (wheelTestMode) {
                telemetry.addLine("========== WHEEL TEST MODE ==========");
                telemetry.addLine();
                telemetry.addData(">>> TEST MODE", "ACTIVE <<<");
                telemetry.addLine("Left Stick Y: Front Left");
                telemetry.addLine("Left Stick X: Front Right");
                telemetry.addLine("Right Stick Y: Back Left");
                telemetry.addLine("Right Stick X: Back Right");
                telemetry.addLine();
                telemetry.addLine("Press BOTH BUMPERS to exit");
                telemetry.addLine();
            } else {
                telemetry.addLine("========== MATCH MODE ==========");
                telemetry.addLine();
                telemetry.addData("Alliance", selectedAlliance);
//            telemetry.addData("Heading", "%.0f°", Math.toDegrees(botHeading));
                telemetry.addData("Active Driver", activeDriver);
                telemetry.addLine();
                telemetry.addData("Drive Mode", autoStatus);
                if (autoAligning) {
                    telemetry.addData(">>> AUTO-ALIGN", "ACTIVE <<<");
                }
                telemetry.addLine();
                telemetry.addData("Intake", intakeStatus);
                telemetry.addData("Launch", launchStatus);
                telemetry.addLine();
                if (slowMode) {
                    telemetry.addData("Slow Mode", "ACTIVE (30%)");
                }
                telemetry.addLine();
            }
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
            telemetry.addData("FL Encoder", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BR Encoder", backRightMotor.getCurrentPosition());
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

    private double applyDeadzone(double value) {
        if (Math.abs(value) < JOYSTICK_DEADZONE) {
            return 0;
        }
        return value;
    }
}
