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

@TeleOp
public class testtelop01X extends LinearOpMode {
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

            // ========== ALLIANCE SELECTION (BOTH GAMEPADS) ==========
            if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) ||
                    (currentGamepad2.left_bumper && !previousGamepad2.left_bumper)) {
                selectedAlliance = Alliance.RED;
            } else if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ||
                    (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)) {
                selectedAlliance = Alliance.BLUE;
            }

            // ========== LEFT TRIGGER - APRILTAG AUTO-ALIGNMENT (BOTH GAMEPADS) ==========
            if (currentGamepad1.left_trigger > 0.5 || currentGamepad2.left_trigger > 0.5) {
                if (selectedAlliance != Alliance.NONE) {
                    autoAligning = true;
                } else {
                    autoAligning = false;
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
                    frontLeftMotor.setPower(y);
                    frontRightMotor.setPower(y);
                    backLeftMotor.setPower(y);
                    backRightMotor.setPower(y);
                    x = currentGamepad1.left_stick_x;
                    frontLeftMotor.setPower(x);
                    frontRightMotor.setPower(-x);
                    backLeftMotor.setPower(-x);
                    backRightMotor.setPower(x);
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (GAMEPAD2_ACTIVE) {
                    y = -currentGamepad2.left_stick_y;
                    frontLeftMotor.setPower(y);
                    frontRightMotor.setPower(y);
                    backLeftMotor.setPower(y);
                    backRightMotor.setPower(y);
                    x = currentGamepad2.left_stick_x;
                    frontLeftMotor.setPower(x);
                    frontRightMotor.setPower(-x);
                    backLeftMotor.setPower(-x);
                    backRightMotor.setPower(x);
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                }

                y = applyDeadzone(y);
                x = applyDeadzone(x);
            } else {
                // ========== MANUAL CONTROL ==========
                if (GAMEPAD1_ACTIVE) {
                    y = -currentGamepad1.left_stick_y;
                    frontLeftMotor.setPower(y);
                    frontRightMotor.setPower(y);
                    backLeftMotor.setPower(y);
                    backRightMotor.setPower(y);
                    x = currentGamepad1.left_stick_x;
                    frontLeftMotor.setPower(x);
                    frontRightMotor.setPower(-x);
                    backLeftMotor.setPower(-x);
                    backRightMotor.setPower(x);
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (GAMEPAD2_ACTIVE) {
                    y = -currentGamepad2.left_stick_y;
                    frontLeftMotor.setPower(y);
                    frontRightMotor.setPower(y);
                    backLeftMotor.setPower(y);
                    backRightMotor.setPower(y);
                    x = currentGamepad2.left_stick_x;
                    frontLeftMotor.setPower(x);
                    frontRightMotor.setPower(-x);
                    backLeftMotor.setPower(-x);
                    backRightMotor.setPower(x);
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

            // ========== FIELD-CENTRIC TRANSFORMATION ==========
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//            rotX = rotX * 1.1;  // Strafe correction

            // ========== POWER NORMALIZATION (CRITICAL FIX) ==========
            // Changed back to rx
            double frontLeftPower = rx;
            double backLeftPower = rx;
            double frontRightPower = -rx;
            double backRightPower = -rx;

            // Find the maximum absolute power
            double maxPower = Math.abs(frontLeftPower);
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            // Normalize if any power exceeds the driver's max power limit
            if (maxPower > maxDrivePower) {
                frontLeftPower = (frontLeftPower / maxPower) * maxDrivePower;
                backLeftPower = (backLeftPower / maxPower) * maxDrivePower;
                frontRightPower = (frontRightPower / maxPower) * maxDrivePower;
                backRightPower = (backRightPower / maxPower) * maxDrivePower;
            }

            // Apply range clipping as final safety
            frontLeftMotor.setPower(Range.clip(frontLeftPower, -maxDrivePower, maxDrivePower));
            backLeftMotor.setPower(Range.clip(backLeftPower, -maxDrivePower, maxDrivePower));
            frontRightMotor.setPower(Range.clip(frontRightPower, -maxDrivePower, maxDrivePower));
            backRightMotor.setPower(Range.clip(backRightPower, -maxDrivePower, maxDrivePower));

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

            // A button - toggle intake reverse
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

            // ========== ENHANCED TELEMETRY ==========
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