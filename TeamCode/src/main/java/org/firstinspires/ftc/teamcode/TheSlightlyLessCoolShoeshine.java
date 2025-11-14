package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
//Hi i am back again with my horrible comments. Witness my glory. BTW the name of this code is a reference to a song. IDK if anyone knows it :((
@TeleOp
public class TheSlightlyLessCoolShoeshine extends LinearOpMode {
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_HALF_POWER = 0.5;
    private static final double LAUNCH_MOTOR_POWER = 1.0;
    private static final double RAMP_MOTOR_POWER = 1.0;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double AUTO_ALIGN_ROTATION_SPEED = 0.15;
    private static final double ROTATION_SPEED = 0.5;
    private static final boolean GAMEPAD1_ACTIVE = true;
    private static final boolean GAMEPAD2_ACTIVE = true;

    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 0.8;
    private static final double GAMEPAD2_MAX_POWER = 0.8;
    private static final double DPAD_POWER = 0.8;

    // Turning constants (from second code)
    private static final double THRESH_WM_POWER_FORTURN = 0.8;
    private static final double FRONT_WHEEL_TURN_MULTIPLIER = 0.6;
    private static final double TRIGGER_TURN_THRESHOLD = 0.2;

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

        // Initial motor powers to zero
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

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

            // ========== MODE BUTTON - APRILTAG AUTO-ALIGNMENT (BOTH GAMEPADS) ==========
            if ((currentGamepad1.start && !previousGamepad1.start) ||
                    (currentGamepad2.start && !previousGamepad2.start)) {
                if (selectedAlliance != Alliance.NONE) {
                    autoAligning = !autoAligning;
                }
            }

            // Check for driver override from either gamepad
            boolean driverOverride = Math.abs(currentGamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > TRIGGER_TURN_THRESHOLD ||
                    currentGamepad1.right_trigger > TRIGGER_TURN_THRESHOLD ||
                    currentGamepad1.dpad_up || currentGamepad1.dpad_down ||
                    currentGamepad1.dpad_left || currentGamepad1.dpad_right ||
                    Math.abs(currentGamepad2.left_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > TRIGGER_TURN_THRESHOLD ||
                    currentGamepad2.right_trigger > TRIGGER_TURN_THRESHOLD;

            if (driverOverride && autoAligning) {
                autoAligning = false;
            }

            double y = 0, x = 0, rx = 0;
            String autoStatus = "MANUAL";
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";

            // Check gamepad activity
            boolean gamepad1MoveActive = Math.abs(currentGamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    currentGamepad1.dpad_up || currentGamepad1.dpad_down ||
                    currentGamepad1.dpad_left || currentGamepad1.dpad_right;

            boolean gamepad1TurnActive = currentGamepad1.left_trigger > TRIGGER_TURN_THRESHOLD ||
                    currentGamepad1.right_trigger > TRIGGER_TURN_THRESHOLD;

            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > TRIGGER_TURN_THRESHOLD ||
                    currentGamepad2.right_trigger > TRIGGER_TURN_THRESHOLD ||
                    currentGamepad2.dpad_up || currentGamepad2.dpad_down ||
                    currentGamepad2.dpad_left || currentGamepad2.dpad_right;

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
                if (gamepad1MoveActive) {
                    y = -currentGamepad1.left_stick_x;
                    x = currentGamepad1.left_stick_y;
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (GAMEPAD2_ACTIVE && gamepad2Active) {
                    y = -currentGamepad2.left_stick_x;
                    x = currentGamepad2.left_stick_y;
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                }

                y = applyDeadzone(y);
                x = applyDeadzone(x);
            } else {
                // ========== MANUAL CONTROL ==========
                y = x = rx = 0;

                // Check for movement input
                if (gamepad1MoveActive) {
                    // D-pad movement for gamepad1
                    if (currentGamepad1.dpad_up) {
                        x = DPAD_POWER;  // Forward
                    } else if (currentGamepad1.dpad_down) {
                        x = -DPAD_POWER;  // Backward
                    }

                    if (currentGamepad1.dpad_left) {
                        y = DPAD_POWER;  // Strafe left
                    } else if (currentGamepad1.dpad_right) {
                        y = -DPAD_POWER;  // Strafe right
                    }

                    // Left stick overrides D-pad if both are pressed
                    // Now with gradient control - joystick value directly controls power
                    if (Math.abs(currentGamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
                            Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE) {
                        y = -currentGamepad1.left_stick_x;
                        x = currentGamepad1.left_stick_y;
                    }

                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";

                    y = applyDeadzone(y);
                    x = applyDeadzone(x);
                } else if (GAMEPAD2_ACTIVE && Math.abs(currentGamepad2.left_stick_x) > JOYSTICK_DEADZONE ||
                        Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE) {
                    // Gamepad 2 also gets gradient control
                    y = -currentGamepad2.left_stick_x;
                    x = currentGamepad2.left_stick_y;
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";

                    y = applyDeadzone(y);
                    x = applyDeadzone(x);
                }

                //  TRIGGER-BASED TURNING (UPDATED)
                // Check gamepad1 triggers first (priority)
                if (gamepad1TurnActive) {
                    if (currentGamepad1.left_trigger > TRIGGER_TURN_THRESHOLD) {
                        // Left trigger = turn left (gradient control)
                        frontLeftMotor.setPower(-0.8);
                        frontRightMotor.setPower(0.8);
                        backLeftMotor.setPower(-0.8);
                        backRightMotor.setPower(0.8);
                    } else if (currentGamepad1.right_trigger > TRIGGER_TURN_THRESHOLD) {
                        // Right trigger = turn right (gradient control)
                        frontLeftMotor.setPower(0.8);
                        frontRightMotor.setPower(-0.8);
                        backLeftMotor.setPower(0.8);
                        backRightMotor.setPower(-0.8);
                    }
                    activeDriver = "DRIVER 1";
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                }
                // Check gamepad2 triggers if gamepad1 not turning
                else if (GAMEPAD2_ACTIVE && (currentGamepad2.left_trigger > TRIGGER_TURN_THRESHOLD ||
                        currentGamepad2.right_trigger > TRIGGER_TURN_THRESHOLD)) {
                    if (currentGamepad2.left_trigger > TRIGGER_TURN_THRESHOLD) {
                        // Left trigger = turn left (gradient control)
                        rx = -currentGamepad2.left_trigger;
                    } else if (currentGamepad2.right_trigger > TRIGGER_TURN_THRESHOLD) {
                        // Right trigger = turn right (gradient control)
                        rx = currentGamepad2.right_trigger;
                    }
                    activeDriver = "DRIVER 2";
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                }

                // Apply slow mode
                if (slowMode) {
                    y *= SLOW_MODE_MULTIPLIER;
                    x *= SLOW_MODE_MULTIPLIER;
                    rx *= SLOW_MODE_MULTIPLIER;
                }
            }

            // ========== POWER CALCULATION WITH TURNING LOGIC FROM SECOND CODE ==========
            double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

            // CHECK IF LAUNCH MOTOR IS ACTIVE - LOCK WHEELS IF SO
            if (launchMotorActive) {
                // COMPLETE WHEEL LOCK - no movement allowed during launch
                frontLeftPower = 0;
                backLeftPower = 0;
                frontRightPower = 0;
                backRightPower = 0;
            } else if (Math.abs(rx) > JOYSTICK_DEADZONE) {
                // Turning mode - can turn while moving forward/backward
                // Combine translation (x, y) with rotation (rx)
                frontLeftPower = y + x - (rx * FRONT_WHEEL_TURN_MULTIPLIER);  // changed + to -
                backLeftPower = y - x - rx;                                    // changed + to -
                frontRightPower = y - x + (rx * FRONT_WHEEL_TURN_MULTIPLIER); // changed - to +
                backRightPower = y + x + rx;                                   // changed - to +

                // Normalize and clip for turning
                double maxPower = Math.abs(frontLeftPower);
                maxPower = Math.max(maxPower, Math.abs(backLeftPower));
                maxPower = Math.max(maxPower, Math.abs(frontRightPower));
                maxPower = Math.max(maxPower, Math.abs(backRightPower));

                if (maxPower > THRESH_WM_POWER_FORTURN) {
                    frontLeftPower = (frontLeftPower / maxPower) * THRESH_WM_POWER_FORTURN;
                    backLeftPower = (backLeftPower / maxPower) * THRESH_WM_POWER_FORTURN;
                    frontRightPower = (frontRightPower / maxPower) * THRESH_WM_POWER_FORTURN;
                    backRightPower = (backRightPower / maxPower) * THRESH_WM_POWER_FORTURN;
                }

                frontLeftPower = Range.clip(frontLeftPower, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
                backLeftPower = Range.clip(backLeftPower, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
                frontRightPower = Range.clip(frontRightPower, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
                backRightPower = Range.clip(backRightPower, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
            } else {
                // Standard mecanum drive (no rotation)
                frontLeftPower = y + x + rx;
                backLeftPower = y - x + rx;
                frontRightPower = y - x - rx;
                backRightPower = y + x - rx;

                // Normalize powers
                double maxPower = Math.abs(frontLeftPower);
                maxPower = Math.max(maxPower, Math.abs(backLeftPower));
                maxPower = Math.max(maxPower, Math.abs(frontRightPower));
                maxPower = Math.max(maxPower, Math.abs(backRightPower));

                if (maxPower > maxDrivePower) {
                    frontLeftPower = (frontLeftPower / maxPower) * maxDrivePower;
                    backLeftPower = (backLeftPower / maxPower) * maxDrivePower;
                    frontRightPower = (frontRightPower / maxPower) * maxDrivePower;
                    backRightPower = (backRightPower / maxPower) * maxDrivePower;
                }

                // Final clipping
                frontLeftPower = Range.clip(frontLeftPower, -maxDrivePower, maxDrivePower);
                backLeftPower = Range.clip(backLeftPower, -maxDrivePower, maxDrivePower);
                frontRightPower = Range.clip(frontRightPower, -maxDrivePower, maxDrivePower);
                backRightPower = Range.clip(backRightPower, -maxDrivePower, maxDrivePower);
            }

            // Apply motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

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

            // ========== INTAKE CONTROL (BOTH GAMEPADS) ==========
            // Toggle intake speed with GUIDE button (was START button)
            if ((currentGamepad1.guide && !previousGamepad1.guide) ||
                    (currentGamepad2.guide && !previousGamepad2.guide)) {
                intakeHalfSpeed = !intakeHalfSpeed;
            }

            // B button - quick spin of ramp motor
            //if ((currentGamepad1.b && !previousGamepad1.b) ||
            //      (currentGamepad2.b && !previousGamepad2.b)) {
            //rampMotor.setPower(-RAMP_MOTOR_POWER * 0.2);
            //sleep(500);
            //rampMotor.setPower(0);
            //}

            // A button - stop intake and spin launch motor backwards
            if ((currentGamepad1.a && !previousGamepad1.a) ||
                    (currentGamepad2.a && !previousGamepad2.a)) {
                intakeMotor.setPower(0);
                sleep(500);
                launchMotor.setPower(-LAUNCH_MOTOR_POWER * 0.3);
                sleep(5000);
                launchMotor.setPower(0);
            }

            // D-pad controls for intake
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                intakeForwardActive = !intakeForwardActive;
                if (intakeForwardActive) intakeReverseActive = false;
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
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

            double launchPower = 0;
            double rampPower = 0;
            String launchStatus = "STOPPED";

            // Y button - stop and pickup mode
            if ((currentGamepad1.y && !previousGamepad1.y) ||
                    (currentGamepad2.y && !previousGamepad2.y)) {
                launchMotorActive = false;
                launchMotor.setPower(0);
                launchStatus = "STOPPED";
                sleep(3000);

                intakeMotor.setPower(1.0);
                rampMotor.setPower(0);
            }

            // X button - shoot sequence
            if ((currentGamepad1.x && !previousGamepad1.x) ||
                    (currentGamepad2.x && !previousGamepad2.x)) {
                // LOCK ALL DRIVE MOTORS
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);

                intakeMotor.setPower(0);
                rampMotor.setPower(0);

                launchMotorActive = true;
                launchMotor.setPower(LAUNCH_MOTOR_POWER * 0.75);
                launchStatus = "RUNNING";
                sleep(3000);

                intakeMotor.setPower(1.0);
                rampMotor.setPower(-RAMP_MOTOR_POWER * 0.45);
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
            if (launchMotorActive) {
                telemetry.addData(">>> WHEELS LOCKED", "SHOOTING MODE <<<");
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
