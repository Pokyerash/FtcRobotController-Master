package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name ="SliderByTime")
public class SliderByTime extends LinearOpMode {
    // motor declaration
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor MG;//miscarea glisierei
        DcMotor G;// motor glisiera

        // the string is the hardware map name
        G = hardwareMap.get(DcMotor.class, "G");
        MG = hardwareMap.get(DcMotor.class, "MG");

        // wait for the autonomous period to start
        waitForStart();

        // set the start time
        long startTime = System.currentTimeMillis();

        // run the motor until 3 seconds have passed
        while (System.currentTimeMillis() - startTime < 1000) {
            G.setPower(-0.4); // adjust the power value as needed
            MG.setPower(-1);
        }

        // stop the motor
        MG.setPower(0);
        G.setPower(0);
    }
}