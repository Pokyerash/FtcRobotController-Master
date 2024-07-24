package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "TeleOpDEMO" ,group = "TeleOp")



public class  TeleOpDEMO extends LinearOpMode {


    DcMotor LeftFront;
    DcMotor RightFront;
    DcMotor LeftBack;
    DcMotor RightBack;
    public static double targetPosition;
    private double integralSum=0;
    public double returnPower(double reference, double state, double Kp, double Kd, double Ki){
        double error = reference -state;
        integralSum += error * timer.seconds();


        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum + Ki);
        return output;
    }
    public static double Kp=0.0045;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;




    DcMotor motorMG;//miscarea glisierei
    DcMotor motorG;// motor glisiera


    //declararea servourilor (pentru explicatie la denumiri consultati comentariile de mai jos :> )
    Servo CD;   // cleste dreapta
    Servo CS;   // cleste stanga
    Servo SUD; //servo up and down
    Servo AV;//servo avion


    @Override


    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        motorMG = hardwareMap.get(DcMotor.class, "MG");
        motorG = hardwareMap.get(DcMotor.class, "G");


        CS = hardwareMap.get(Servo.class, "CS");
        CD = hardwareMap.get(Servo.class, "CD");
        SUD = hardwareMap.get(Servo.class, "SUD");
        AV = hardwareMap.get(Servo.class, "AV");

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorMG.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorG.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorMG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        AV.setPosition(0.5);


        CD.setPosition(0.9);
        CS.setPosition(-1);
        //SUD.setPosition(0.2);
        SUD.setPosition(0);
        waitForStart();


        while (opModeIsActive()) {

              double Forward = -gamepad1.left_stick_x;
              double Strafe = -gamepad1.right_stick_y;
              double Rotate = gamepad1.right_stick_x;

                LeftBack.setPower((0.7) * Forward + (0.7) * Strafe + (0.7) * Rotate);
                RightBack.setPower((0.7) * Forward - (0.7) * Strafe + (0.7) * Rotate);
                RightFront.setPower((0.7) * Forward + (0.7) * Strafe - (0.7) * Rotate);
                LeftFront.setPower((0.7) * Forward - (0.7) * Strafe - (0.7) * Rotate);


            double ridicare = -gamepad1.right_trigger;  // Typically, the joystick is inverted
            double coborare = -gamepad1.left_trigger; // Typically, the joystick is inverted

            double jos = gamepad2.right_trigger;
            double sus = gamepad2.left_trigger;

            motorG.setPower(sus-jos);
            motorMG.setPower(ridicare-coborare);

            telemetry.addData("MotorG Power", ridicare);
            telemetry.addData("MotorMG Power", coborare);
            telemetry.update();


            /*if (gamepad2.left_bumper) {
                motorMG.setPower(1);
            }

            if (gamepad2.right_bumper) {
                motorMG.setPower(-1);
            }

         /*   if (gamepad2.dpad_up) {
                motorMG.setPower(0);
            } */

            if (gamepad1.dpad_up) {
                AV.setPosition(0.8);
            }

            if (gamepad2.x) {
                CD.setPosition(0.87);
                //prindere clesti dreapta
            }
            if (gamepad2.dpad_right) {
                //prindere clesti stanga
                CS.setPosition(-1);
            }


            if (gamepad2.b) {
                //lasare clesti dreapta
                CD.setPosition(0.6);
            }
            if (gamepad2.dpad_left) {
                CS.setPosition(0.22);
                //lasare clesti stanga
            }

            if (gamepad2.dpad_up) {
                CS.setPosition(-1);
                CD.setPosition(0.87);
                //inchidere amandoi
            }
            if (gamepad2.dpad_down) {
                CS.setPosition(0.22);
                CD.setPosition(0.6);
                //lasat amandoi
            }

            if (gamepad2.a) {
                SUD.setPosition(0.27);
            }

            if (gamepad2.y) {
                SUD.setPosition(0.15);
            }

            /*if (gamepad2.x) {
                C.setPosition(0);
            }

            if (gamepad2.b) {
                C.setPosition(0.23);
            }

            if (gamepad2.a) {
                SUD.setPosition(0.15);
            }

            if (gamepad2.y) {
                SUD.setPosition(-0.8);
            }*/

           /*if (gamepad1.y){
               motorG.setTargetPosition(-4000);
              motorG.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               motorG.setPower(1);
           }

           if (gamepad1.a){
                motorG.setTargetPosition(4000);
                motorG.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorG.setPower(1);
           }

           if(gamepad2.left_stick_button){
               targetPosition=targetPosition+1000;
           }*/

           /*if(gamepad1.x){
               motorMG.setTargetPosition(1);
               motorMG.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               motorMG.setPower(1);
           }

            if(gamepad1.b){
                motorMG.setTargetPosition(-1);
                motorMG.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorMG.setPower(1);
            }

            if (gamepad2.dpad_up) {
              motorMG.setPower(0);
            }*/

        }

           /* if (gamepad2.b) {
                //prindere clesti dreapta
            }

            if (gamepad2.x) {
                //prindere clesti stanga
            }

            if (gamepad2.dpad_right) {
                //lasare clesti dreapta
            }

            if (gamepad2.dpad_left){
                //lasare clesti stanga
            }

            if (gamepad2.y){
                //servo sus
            }

            if (gamepad2.a){
                //servo jos
            }

        }

     */
}


}
