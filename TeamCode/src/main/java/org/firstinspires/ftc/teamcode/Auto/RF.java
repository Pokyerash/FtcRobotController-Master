package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.ColorVisionProcessor;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionRedClose;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionRedFar;
import org.firstinspires.ftc.teamcode.Detection.PropPipeline;
import org.firstinspires.ftc.teamcode.Detection.TeamPropPipelineRed;
import org.firstinspires.ftc.teamcode.Hardware.PIDConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name ="RF")
public class RF extends LinearOpMode {

    DcMotor LeftFront;
    DcMotor RightFront;
    DcMotor LeftBack;
    DcMotor RightBack;
    Servo cleste;

    DcMotor MG;//miscarea glisierei
    DcMotor G;// motor glisiera


    //declararea servourilor (pentru explicatie la denumiri consultati comentariile de mai jos :> )
    Servo CD;   // cleste dreapta
    Servo CS;   // cleste stanga
    Servo SUD; //servo up and down
    Servo AV;//servo avion
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 0.96;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.33;
    static final double DRIVE_SPEED2 = 0.7;
    static final double TURN_SPEED = 0.25;
    RF.CASE Case;

    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    private static final double FOV = 40;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    enum Location{
        Left,
        Center,
        Right
    }




    public enum CASE {
        left,
        center,
        right
    }

    int cazzz=2;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    org.firstinspires.ftc.teamcode.Detection.PropPipeline PropPipeline = new PropPipeline();


    public static double targetPosition = 0;
    TeamPropPipelineRed teamPropPieline = new TeamPropPipelineRed();
    VisionPortal portal;
    PropDetectionRedFar processor;

    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        MG = hardwareMap.get(DcMotor.class, "MG");
        G = hardwareMap.get(DcMotor.class, "G");


        CS = hardwareMap.get(Servo.class, "CS");
        CD = hardwareMap.get(Servo.class, "CD");
        SUD = hardwareMap.get(Servo.class, "SUD");
        AV = hardwareMap.get(Servo.class, "AV");


        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MG.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        G.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        G.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        AV.setPosition(0.5);


        CD.setPosition(0.9);
        CS.setPosition(-1);
        //SUD.setPosition(0.2);
        SUD.setPosition(0);

        MG.setPower(1);
        sleep(650);
        MG.setPower(0);

        processor = new PropDetectionRedFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280, 720))
                .enableLiveView(true)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();

        while (opModeInInit()) {
            telemetry.addData("case",processor.detection);
            telemetry.addData("right",processor.rightSum);
            telemetry.addData("middle",processor.middleSum);
            dashboard.setTelemetryTransmissionInterval(55);
            telemetry.update();

        }



        waitForStart();
        if (opModeIsActive() && !isStopRequested()) {

            if(processor.detection==1){
                left();
            }

            if(processor.detection==2){
                center();
            }

            if(processor.detection==3){
                right();
            }
        }


    }

    private void left() {
            encoderDrive(DRIVE_SPEED, -6.3, 6.3, 5000);
            encoderDrive(DRIVE_SPEED, 5.5, 5.5, 5000);
            encoderDrive(DRIVE_SPEED, 0.1, -0.1, 5000);
            sleep(400);

            SUD.setPosition(0.25);
            sleep(200);
            CD.setPosition(0.5);
            sleep(200);
            // encoderDrive(DRIVE_SPEED, -9.6, 9.6, 5000);

            encoderDrive(DRIVE_SPEED, 0.5, -0.5, 5000);
            encoderDrive(DRIVE_SPEED, -5.5, -5.5, 5000);
            encoderDrive(DRIVE_SPEED, -4.7, 4.7, 5000);
            encoderDrive(DRIVE_SPEED, -5.5, -5.5, 5000);
            sleep(200);

            encoderDrive(DRIVE_SPEED, -19.5, 19.5, 5000);
            encoderDrive(DRIVE_SPEED, -5.4, -5.4, 5000);
            encoderDrive(DRIVE_SPEED, -5.6, 5.6, 5000);
            encoderDrive(DRIVE_SPEED, 5.5, 5.5, 5000);

        MG.setPower(-1);
        sleep(1000);
        MG.setPower(0);
        G.setPower(-0.5);
        sleep(1000);
        G.setPower(0);

            encoderDrive(DRIVE_SPEED, -1.2, 1.2, 5000);
            sleep(200);

            SUD.setPosition(0.15);


            CS.setPosition(0.5);
            sleep(200);

            encoderDrive(DRIVE_SPEED, 0.5, -0.5, 5000);
            encoderDrive(DRIVE_SPEED, -5.6, -5.6, 5000);
            G.setPower(0.5);
            sleep(700);
            G.setPower(0);
            SUD.setPosition(0.02);
            encoderDrive(DRIVE_SPEED, -5.5, 5.5, 5000);

        }

    private void center() {
        encoderDrive(DRIVE_SPEED, -5.6, 5.6, 5000);
        sleep(200);
        SUD.setPosition(0.2);

        sleep(200);
        CD.setPosition(0.5);
        sleep(200);
        encoderDrive(DRIVE_SPEED, -5.4, -5.4, 5000);
        sleep(200);
        encoderDrive(DRIVE_SPEED, -19.9, 19.9, 5000);

        MG.setPower(-1);
        sleep(1000);
        MG.setPower(0);
        G.setPower(-0.5);
        sleep(1000);
        G.setPower(0);


        SUD.setPosition(0.15);
        sleep(200);
        CS.setPosition(0.5);
        sleep(200);

        encoderDrive(DRIVE_SPEED, 0.5, -0.5, 5000);
        encoderDrive(DRIVE_SPEED, -5.6, -5.6, 5000);
        G.setPower(0.39);
        sleep(700);
        G.setPower(0);
        SUD.setPosition(0.02);
        encoderDrive(DRIVE_SPEED, -5.5, 5.5, 5000);



    }

    private void right() {
        encoderDrive(DRIVE_SPEED, -6.3, 6.3, 5000);
        encoderDrive(DRIVE_SPEED, -5.6, -5.6, 5000);

        SUD.setPosition(0.2);
        sleep(200);
        CD.setPosition(0.5);
        sleep(200);

        encoderDrive(DRIVE_SPEED, 0.5, -0.5, 5000);
        encoderDrive(DRIVE_SPEED, 5.5, 5.5, 5000);
        encoderDrive(DRIVE_SPEED, -4.7, 4.7, 5000);
        encoderDrive(DRIVE_SPEED, -5.5, -5.5, 5000);
        sleep(200);


        encoderDrive(DRIVE_SPEED, -19.8, 19.8, 5000);
        encoderDrive(DRIVE_SPEED, -5.4, -5.4, 5000);
        encoderDrive(DRIVE_SPEED, -3.2, 3.2, 5000);
        encoderDrive(DRIVE_SPEED, 5.4, 5.4, 5000);
        SUD.setPosition(0.15);

        MG.setPower(-1);
        sleep(1000);
        MG.setPower(0);
        G.setPower(-0.5);
        sleep(1000);
        G.setPower(0);

        sleep(200);
        CS.setPosition(0.5);
        sleep(200);

        G.setPower(0.5);
        sleep(700);
        G.setPower(0);

        encoderDrive(DRIVE_SPEED, 0.5, -0.5, 5000);
        encoderDrive(DRIVE_SPEED, -5.4, -5.4, 5000);
        SUD.setPosition(0.02);
        encoderDrive(DRIVE_SPEED, -5.6, 5.6, 5000);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LeftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = RightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarget = LeftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = RightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            LeftBack.setTargetPosition(newLeftTarget);
            RightBack.setTargetPosition(newRightTarget);
            LeftFront.setTargetPosition(newLeftTarget);
            RightFront.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftBack.isBusy() && RightBack.isBusy() && LeftFront.isBusy() && RightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        LeftBack.getCurrentPosition(), RightBack.getCurrentPosition(),LeftFront.getCurrentPosition(), RightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftBack.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            RightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void liftDrive(double speed, double target,
                          double timeoutS) {
        int newLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = G.getCurrentPosition() + (int) (target);
            G.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            G.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            G.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (G.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Lift:", G.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            G.setPower(0);

            sleep(250);   // optional pause after each move.

        }

    }
    public void susJos(double speed, double target, double timeoutS) {
        int newLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLiftTarget = MG.getCurrentPosition() + (int) (target);
            MG.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            MG.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion
            runtime.reset();
            MG.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and the motor is running
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && MG.isBusy()) {
                // Display it for the driver
                telemetry.addData("Lift:", MG.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            MG.setPower(0);
            sleep(250); // optional pause after each move
        }
    }

}