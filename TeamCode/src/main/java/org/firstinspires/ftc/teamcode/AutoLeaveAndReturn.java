package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto Leave and Return", group = "FTC2025")
public class AutoLeaveAndReturn extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== HARDWARE SETUP =====
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Set motor directions (reverse right side for forward movement)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== WAIT FOR START =====
        waitForStart();

        if (opModeIsActive()) {

            // === MOVE FORWARD (Leave the Launch Line) ===
            telemetry.addLine("Moving forward...");
            telemetry.update();

            // Simple forward movement: all motors same power (right side reversed in hardware)
            double forwardPower = 0.5;
            frontLeftMotor.setPower(forwardPower);
            backLeftMotor.setPower(forwardPower);
            frontRightMotor.setPower(forwardPower);
            backRightMotor.setPower(forwardPower);
            sleep(1500);   // drive ~2 ft forward
            
            // Stop all motors explicitly
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            
            // Ensure motors stop
            sleep(100);

            // === PAUSE ===
            telemetry.addLine("Pause...");
            telemetry.update();
            sleep(500);

            // === MOVE BACKWARD (Return toward Base) ===
            telemetry.addLine("Returning to base...");
            telemetry.update();

            // Simple backward movement: all motors negative power (right side reversed in hardware)
            double backwardPower = -0.5;
            frontLeftMotor.setPower(backwardPower);
            backLeftMotor.setPower(backwardPower);
            frontRightMotor.setPower(backwardPower);
            backRightMotor.setPower(backwardPower);
            sleep(1500);   // same duration to come back
            
            // Stop all motors explicitly
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            
            // Ensure motors stop
            sleep(100);

            // === END ===
            telemetry.addLine("Done!");
            telemetry.update();
            sleep(1000);
        }
    }
}
