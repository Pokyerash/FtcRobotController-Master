package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Strafe")
public class Strafe extends OpMode {

    // Hardware Initialization
    DcMotor LeftFront, RightFront, LeftBack, RightBack;

    @Override
    public void init() {
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        // Set motor directions if necessary
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        // Set encoders
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        // Example: Strafe right 1000 encoder ticks at half power
        strafe(0.5, 700);
    }

    @Override
    public void loop() {
        // You can add telemetry or other logic here if necessary
    }

    // Strafing function
    public void strafe(double power, int distance) {
        // Calculate the target position for each motor
        int targetFL = LeftFront.getCurrentPosition() + distance;
        int targetFR = RightFront.getCurrentPosition() - distance;
        int targetBL = LeftBack.getCurrentPosition() - distance;
        int targetBR = RightBack.getCurrentPosition() + distance;

        // Set target positions
        LeftFront.setTargetPosition(targetFL);
        RightFront.setTargetPosition(targetFR);
        LeftBack.setTargetPosition(targetBL);
        RightBack.setTargetPosition(targetBR);

        // Set to RUN_TO_POSITION mode
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to motors
        LeftFront.setPower(power);
        RightFront.setPower(power);
        LeftBack.setPower(power);
        RightBack.setPower(power);

        // Wait until target position is reached
        while (LeftFront.isBusy() && RightFront.isBusy() && LeftBack.isBusy() && RightBack.isBusy()) {
            // Optionally add telemetry or other logic here
        }

        // Stop all motion
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);

        // Set motors back to RUN_USING_ENCODER mode
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
