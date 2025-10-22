package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    // Motor power constants - easy to tune without recompiling
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_HALF_POWER = 0.5;
    private static final double LAUNCH_MOTOR_POWER = 1.0;
    private static final double RAMP_MOTOR_POWER = 1.0;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
    // Dead wheel odometry constants
    private static final double ODO_COUNTS_PER_REV = 8192.0;  // REV Through Bore Encoder
    private static final double ODO_WHEEL_DIAMETER_INCHES = 1.26;  // 32mm converted
    private static final double ODO_COUNTS_PER_INCH = ODO_COUNTS_PER_REV / (ODO_WHEEL_DIAMETER_INCHES * Math.PI);
    
    // Autonomous navigation constants
    private static final double AUTO_MAX_SPEED = 0.7;
    private static final double AUTO_MIN_SPEED = 0.2;
    private static final double SLOWDOWN_DISTANCE_FEET = 1.0;
    private static final double POSITION_TOLERANCE_INCHES = 4.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 3.0;
    
    // Alliance positions (in feet from field center)
    private static final double RED_TARGET_X = -6.0;
    private static final double RED_TARGET_Y = -3.0;
    private static final double BLUE_TARGET_X = 6.0;
    private static final double BLUE_TARGET_Y = -3.0;
    
    // AprilTag facing positions
    private static final double RED_FACE_X = 6.0;
    private static final double RED_FACE_Y = 6.0;
    private static final double BLUE_FACE_X = -6.0;
    private static final double BLUE_FACE_Y = 6.0;
    
    // AprilTag IDs
    private static final int RED_APRILTAG_ID = 24;
    private static final int BLUE_APRILTAG_ID = 20;
    
    // Servo positions
    private static final double GATE_SERVO_DOWN = 0.0;
    private static final double GATE_SERVO_UP = 0.417;  // 75 degrees (75/180)
    
    // Odometry variables
    private double robotX = 0;  // in feet
    private double robotY = 0;  // in feet
    private double robotHeading = 0;  // in radians
    private int lastForwardOdoPos = 0;
    private int lastRightOdoPos = 0;
    
    // Alliance selection
    private enum Alliance { NONE, RED, BLUE }
    private Alliance selectedAlliance = Alliance.NONE;
    
    // Autonomous navigation state
    private boolean autoNavigating = false;
    
    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        // Declare additional motors
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launchMotor1 = hardwareMap.dcMotor.get("launchMotor1");
        DcMotor launchMotor2 = hardwareMap.dcMotor.get("launchMotor2");
        DcMotor rampMotor = hardwareMap.dcMotor.get("rampMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setmode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Declare servo
        Servo gateServo = hardwareMap.servo.get("GateServo");
        
        // Declare dead wheel encoders
        DcMotor forwardOdo = hardwareMap.dcMotor.get("forwardOdo");
        DcMotor rightOdo = hardwareMap.dcMotor.get("rightOdo");
        
        // Reset dead wheel encoders
        forwardOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        forwardOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Motor safety - set zero power behavior to brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reverse the right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Initialize servo
        gateServo.setPosition(GATE_SERVO_DOWN);
        
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        
        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        
        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        
        // INIT SETUP ROUTINE - Set safe starting positions
        telemetry.addLine("========================================");
        telemetry.addLine("INITIALIZING ROBOT");
        telemetry.addLine("========================================");
        telemetry.update();
        
        // Set servos to safe positions
        gateServo.setPosition(GATE_SERVO_DOWN);
        telemetry.addData("Gate Servo", "DOWN (safe position)");
        telemetry.update();
        sleep(200);
        
        // Verify IMU is ready
        telemetry.addData("IMU Status", "Initialized");
        telemetry.addData("IMU Heading", "%.2f degrees", 
            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        sleep(200);
        
        // Verify dead wheels are reset
        telemetry.addData("Forward Odometry", "Reset to 0");
        telemetry.addData("Right Odometry", "Reset to 0");
        telemetry.update();
        sleep(200);
        
        // Verify camera is ready
        telemetry.addData("Camera Status", visionPortal.getCameraState());
        telemetry.update();
        sleep(200);
        
        telemetry.addLine("========================================");
        telemetry.addLine("✓ ROBOT READY");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();
        
        // Initialize gamepad state tracking
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        
        // Toggle states
        boolean slowMode = false;
        boolean dpadControlEnabled = false;
        boolean intakeHalfSpeed = false;
        boolean intakeForwardActive = false;
        boolean intakeReverseActive = false;
        boolean launchMotorsActive = false;
        boolean detailedTelemetry = false;  // Toggle for debug vs match mode telemetry
        
        waitForStart();
        if (isStopRequested()) return;
        
        // Start ramp motor immediately
        rampMotor.setPower(RAMP_MOTOR_POWER);
        
        while (opModeIsActive()) {
            // Store the previous gamepad state
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            
            // Update odometry
            updateOdometry(forwardOdo, rightOdo, imu);
            
            // Alliance selection with triggers
            if (currentGamepad1.left_trigger > 0.5 && previousGamepad1.left_trigger <= 0.5) {
                selectedAlliance = Alliance.RED;
            } else if (currentGamepad1.right_trigger > 0.5 && previousGamepad1.right_trigger <= 0.5) {
                selectedAlliance = Alliance.BLUE;
            }
            
            // Left bumper - navigate to alliance position
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && selectedAlliance != Alliance.NONE) {
                autoNavigating = true;
            }
            
            // Right bumper - gate servo flick
            if (currentGamepad1.right_bumper) {
                gateServo.setPosition(GATE_SERVO_UP);
            } else {
                gateServo.setPosition(GATE_SERVO_DOWN);
            }
            
            // Check for driver override (any joystick movement)
            boolean driverOverride = Math.abs(currentGamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE;
            
            if (driverOverride && autoNavigating) {
                autoNavigating = false;
            }
            
            double y = 0, x = 0, rx = 0;
            
            if (autoNavigating) {
                // Autonomous navigation
                double[] navOutput = navigateToAlliance(selectedAlliance, imu);
                x = navOutput[0];
                y = navOutput[1];
                rx = navOutput[2];
                
                // Check if reached destination
                if (navOutput[3] > 0) {  // navOutput[3] is completion flag
                    autoNavigating = false;
                }
            } else {
                // Manual control
                y = -currentGamepad1.left_stick_y;
                x = currentGamepad1.left_stick_x;
                rx = currentGamepad1.right_stick_x;
                
                // Apply deadzone
                y = applyDeadzone(y);
                x = applyDeadzone(x);
                rx = applyDeadzone(rx);
                
                // D-pad control
                if (currentGamepad1.back && !previousGamepad1.back) {
                    dpadControlEnabled = !dpadControlEnabled;
                }
                
                if (dpadControlEnabled) {
                    if (currentGamepad1.dpad_up) y = 1.0;
                    else if (currentGamepad1.dpad_down) y = -1.0;
                    if (currentGamepad1.dpad_right) x = 1.0;
                    else if (currentGamepad1.dpad_left) x = -1.0;
                }
                
                // Toggle slow mode
                if (currentGamepad1.y && !previousGamepad1.y) {
                    slowMode = !slowMode;
                }
            }
            
            // Apply slow mode multiplier only when using D-pad
            double translationMultiplier = 1.0;  // No manual slow mode toggle anymore
            
            // Reset IMU
            if (currentGamepad1.options && !previousGamepad1.options) {
                imu.resetYaw();
            }
            
            // Toggle detailed telemetry with start button (hold for 0.5s to avoid accidental IMU reset)
            if (currentGamepad1.start && !previousGamepad1.start) {
                detailedTelemetry = !detailedTelemetry;
            }
            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            
            // Field-centric transformation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            
            rotX = rotX * 1.1 * translationMultiplier;
            rotY = rotY * translationMultiplier;
            
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
            
            // Toggle intake speed mode
            if (currentGamepad1.guide && !previousGamepad1.guide) {
                intakeHalfSpeed = !intakeHalfSpeed;
            }
            
            // Intake control
            if (currentGamepad1.b && !previousGamepad1.b) {
                intakeForwardActive = !intakeForwardActive;
                if (intakeForwardActive) intakeReverseActive = false;
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                intakeReverseActive = !intakeReverseActive;
                if (intakeReverseActive) intakeForwardActive = false;
            }
            
            double intakePower = 0;
            String intakeStatus = "STOPPED";
            if (intakeForwardActive) {
                double speedToUse = intakeHalfSpeed ? INTAKE_HALF_POWER : INTAKE_FULL_POWER;
                intakePower = speedToUse;
                intakeStatus = "FORWARD (" + (intakeHalfSpeed ? "HALF" : "FULL") + ")";
            } else if (intakeReverseActive) {
                double speedToUse = intakeHalfSpeed ? INTAKE_HALF_POWER : INTAKE_FULL_POWER;
                intakePower = -speedToUse;
                intakeStatus = "REVERSE (" + (intakeHalfSpeed ? "HALF" : "FULL") + ")";
            }
            intakeMotor.setPower(intakePower);
            
            // Launch motors
            if (currentGamepad1.x && !previousGamepad1.x) {
                launchMotorsActive = !launchMotorsActive;
            }
            
            double launch1Power = 0;
            double launch2Power = 0;
            String launchStatus = "STOPPED";
            if (launchMotorsActive) {
                launch1Power = LAUNCH_MOTOR_POWER;
                launch2Power = -LAUNCH_MOTOR_POWER;
                launchStatus = "RUNNING";
            }
            launchMotor1.setPower(launch1Power);
            launchMotor2.setPower(launch2Power);
            
            // Telemetry - Two modes: Match mode (clean) vs Debug mode (detailed)
            if (detailedTelemetry) {
                // DEBUG MODE - Full details for testing and troubleshooting
                telemetry.addLine("========== DEBUG MODE ==========");
                telemetry.addLine();
                
                telemetry.addLine("=== ALLIANCE & NAVIGATION ===");
                telemetry.addData("Selected Alliance", selectedAlliance);
                telemetry.addData("Auto Navigation", autoNavigating ? "ACTIVE" : "INACTIVE");
                telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                telemetry.addData("Robot Heading", "%.2f°", Math.toDegrees(robotHeading));
                telemetry.addLine();
                
                telemetry.addLine("=== DRIVE MODES ===");
                telemetry.addData("Slow Mode", slowMode ? "ENABLED" : "DISABLED");
                telemetry.addData("D-pad Control", dpadControlEnabled ? "ENABLED" : "DISABLED");
                telemetry.addLine();
                
                telemetry.addLine("=== MOTOR STATUS ===");
                telemetry.addData("Intake", intakeStatus);
                telemetry.addData("Intake Speed Mode", intakeHalfSpeed ? "HALF POWER" : "FULL POWER");
                telemetry.addData("Launch Motors", launchStatus);
                telemetry.addData("Ramp Motor", "RUNNING");
                telemetry.addData("Gate Servo", currentGamepad1.right_bumper ? "UP (75°)" : "DOWN (0°)");
                telemetry.addLine();
                
                telemetry.addLine("=== MOTOR POWERS ===");
                telemetry.addData("FL/FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
                telemetry.addData("BL/BR", "%.2f / %.2f", backLeftPower, backRightPower);
                telemetry.addData("Intake", "%.2f", intakePower);
                telemetry.addData("Launch 1/2", "%.2f / %.2f", launch1Power, launch2Power);
                telemetry.addLine();
                
                telemetry.addLine("=== ODOMETRY ===");
                telemetry.addData("Forward Odo", forwardOdo.getCurrentPosition());
                telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            } else {
                // MATCH MODE - Clean essentials only
                telemetry.addLine("========== MATCH MODE ==========");
                telemetry.addLine();
                
                // Critical info only
                telemetry.addData("Alliance", selectedAlliance);
                telemetry.addData("Position", "X: %.1f  Y: %.1f ft", robotX, robotY);
                telemetry.addData("Heading", "%.0f°", Math.toDegrees(robotHeading));
                telemetry.addLine();
                
                telemetry.addData("Intake", intakeStatus);
                telemetry.addData("Launch", launchStatus);
                telemetry.addData("Gate", currentGamepad1.right_bumper ? "OPEN" : "CLOSED");
                telemetry.addLine();
                
                if (autoNavigating) {
                    telemetry.addData("AUTO NAV", ">>> ACTIVE <<<");
                }
                
                if (slowMode) {
                    telemetry.addData("Drive Mode", "SLOW");
                }
            }
            
            telemetry.addLine();
            telemetry.addData("Telemetry Mode", detailedTelemetry ? "DEBUG" : "MATCH");
            telemetry.addData("Toggle", "Press START button");
            
            telemetry.update();
        }
        
        // Clean up vision
        visionPortal.close();
    }
    
    private void updateOdometry(DcMotor forwardOdo, DcMotor rightOdo, IMU imu) {
        // Read current encoder positions
        int forwardPos = forwardOdo.getCurrentPosition();
        int rightPos = rightOdo.getCurrentPosition();
        
        // Calculate deltas since last update
        int deltaForward = forwardPos - lastForwardOdoPos;
        int deltaRight = rightPos - lastRightOdoPos;
        
        // Update last positions
        lastForwardOdoPos = forwardPos;
        lastRightOdoPos = rightPos;
        
        // Convert encoder counts to inches
        double forwardInches = deltaForward / ODO_COUNTS_PER_INCH;
        double rightInches = deltaRight / ODO_COUNTS_PER_INCH;
        
        // Get current heading from IMU
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        // Calculate robot-centric displacement
        // Forward wheel measures forward/backward movement
        // Right wheel measures left/right (strafe) movement
        double robotDX = rightInches;   // Right wheel -> X movement (right is positive)
        double robotDY = forwardInches; // Forward wheel -> Y movement (forward is positive)
        
        // Convert to field-centric using current heading
        double fieldDX = robotDX * Math.cos(currentHeading) - robotDY * Math.sin(currentHeading);
        double fieldDY = robotDX * Math.sin(currentHeading) + robotDY * Math.cos(currentHeading);
        
        // Update global position (convert inches to feet)
        robotX += fieldDX / 12.0;
        robotY += fieldDY / 12.0;
        robotHeading = currentHeading;
    }
    
    private double[] navigateToAlliance(Alliance alliance, IMU imu) {
        double targetX = (alliance == Alliance.RED) ? RED_TARGET_X : BLUE_TARGET_X;
        double targetY = (alliance == Alliance.RED) ? RED_TARGET_Y : BLUE_TARGET_Y;
        double faceX = (alliance == Alliance.RED) ? RED_FACE_X : BLUE_FACE_X;
        double faceY = (alliance == Alliance.RED) ? RED_FACE_Y : BLUE_FACE_Y;
        int targetTag = (alliance == Alliance.RED) ? RED_APRILTAG_ID : BLUE_APRILTAG_ID;
        
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        double x = 0, y = 0, rx = 0;
        double completed = 0;
        
        // Check if at position
        if (distanceToTarget * 12 < POSITION_TOLERANCE_INCHES) {
            // At position, now align to AprilTag
            AprilTagDetection targetDetection = getAprilTagDetection(targetTag);
            if (targetDetection != null) {
                double yawError = targetDetection.ftcPose.yaw;
                if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES) {
                    completed = 1;  // Fully complete
                } else {
                    rx = Math.signum(yawError) * 0.3;  // Rotate to align
                }
            } else {
                // No AprilTag visible, use geometric facing
                double desiredAngle = Math.atan2(faceX - robotX, faceY - robotY);
                double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double angleError = desiredAngle - currentAngle;
                
                // Normalize angle
                while (angleError > Math.PI) angleError -= 2 * Math.PI;
                while (angleError < -Math.PI) angleError += 2 * Math.PI;
                
                if (Math.abs(Math.toDegrees(angleError)) < ANGLE_TOLERANCE_DEGREES) {
                    completed = 1;
                } else {
                    rx = Math.signum(angleError) * 0.3;
                }
            }
        } else {
            // Navigate to position
            double angleToTarget = Math.atan2(deltaX, deltaY);
            
            // Speed control based on distance
            double speed = AUTO_MAX_SPEED;
            if (distanceToTarget < SLOWDOWN_DISTANCE_FEET) {
                double slowdownRatio = distanceToTarget / SLOWDOWN_DISTANCE_FEET;
                speed = AUTO_MIN_SPEED + (AUTO_MAX_SPEED - AUTO_MIN_SPEED) * slowdownRatio;
            }
            
            // Convert to robot frame
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            x = Math.sin(angleToTarget - heading) * speed;
            y = Math.cos(angleToTarget - heading) * speed;
        }
        
        return new double[]{x, y, rx, completed};
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
