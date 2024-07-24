package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TwoMotorTeleOp", group="TeleOp")
public class TwoMotorTeleOp extends OpMode {
    // Declare motor objects
    private DcMotor motorG;
    private DcMotor motorMG;

    @Override
    public void init() {
        // Initialize the hardware variables.
        // The string used here as parameters must correspond to the names assigned during the robot configuration step in the FTC app.
        motorG = hardwareMap.get(DcMotor.class, "motorG");
        motorMG = hardwareMap.get(DcMotor.class, "motorMG");

        // Set motor direction if necessary
        motorG.setDirection(DcMotor.Direction.FORWARD);
        motorMG.setDirection(DcMotor.Direction.FORWARD);

        // Set initial power to zero
        motorG.setPower(0);
        motorMG.setPower(0);
    }

    @Override
    public void loop() {
        // Read gamepad inputs

        double ridicare = -gamepad2.left_trigger;  // Typically, the joystick is inverted
        double coborare = -gamepad2.right_trigger; // Typically, the joystick is inverted

        double sus = gamepad1.right_trigger;
        double jos = gamepad1.left_trigger;

        // Control the motors with the gamepad
        motorG.setPower(sus-jos);
        motorMG.setPower(ridicare-coborare);

        // Optionally, send telemetry data to the driver station
        telemetry.addData("MotorG Power", ridicare);
        telemetry.addData("MotorMG Power", coborare);
        telemetry.update();
    }
}
