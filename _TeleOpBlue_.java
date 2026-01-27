package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "TELEOP")
public class _TeleOpBlue_ extends LinearOpMode{

    DcMotorEx GoBildaLauncher, REVLauncher;

    DcMotor GoBildaLoader, REVLoader;

    CRServo C1, C2;

    Servo Aimer;

    public double P = 100000;

    public double F = 3100;

    PIDFCoefficients PIDF = new PIDFCoefficients(P, 0, 0, F);

    /*
     * Launch speed and servo aiming was determined quasi-empirically.
     *
     *  +---------------+---------------+-----------+-----------------------+
     *  |     POINT     |   LAUNCHER    |   SERVO   |   DISTANCE TO TARGET  |
     *  +---------------+---------------+-----------+-----------------------+
     *  |   (+00;+00)   |   2850        |   0.5     |   101.823             |
     *  |   (-12;-12)   |   2650        |   0       |   84.853              |
     *  |   (-24;-24)   |   2350        |   0       |   67.882              |
     *  +---------------+---------------+-----------+-----------------------+
     *
     * Position for blue target: (-72;-72).
     *
     * Launcher function: y = 7.36572 x + 1999.99877
     *
     * Aimer function: y = 0.0147314 x - 1
     * */

    @Override
    public void runOpMode() throws InterruptedException{

        MecanumDrive base = new MecanumDrive(hardwareMap, new Pose2d(60 ,-12 ,0));


        /*
        * Positive rotation direction is clockwise from the perspective opposite the motor.
        * */

        GoBildaLauncher = hardwareMap.get(DcMotorEx.class, "GoBildaLauncher");
        GoBildaLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        GoBildaLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        GoBildaLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GoBildaLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        /*
        REVLauncher = hardwareMap.get(DcMotorEx.class, "REVLauncher");
        REVLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        REVLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        REVLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REVLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
         */
        /*
        * All intake motors go counterclockwise.
        * */

        GoBildaLoader = hardwareMap.get(DcMotor.class, "GoBildaIntake");
        GoBildaLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GoBildaLoader.setDirection(DcMotorSimple.Direction.REVERSE);

        REVLoader = hardwareMap.get(DcMotor.class, "REVLoader");
        REVLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        REVLoader.setDirection(DcMotorSimple.Direction.REVERSE);

        C1 = hardwareMap.get(CRServo.class, "CRServoL");
        C2 = hardwareMap.get(CRServo.class, "CRServoR");

        Aimer = hardwareMap.get(Servo.class, "ServoAimer");

        base.updatePoseEstimate();

        double LaunchVelocity;

        double Distance_Target;

        double ServoAngle;

        final double LinearScalar = 1;

        final double AngularScalar = 1;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() & !isStopRequested()){

            /*
            * Rotation matrix:
            *
            *   +---             ---+
            *   |  +cos a   -sin a  |
            *   |                   |
            *   |  +sin a   +cos a  |
            *   +---             ---+
            *
            * Applied to the vector we will obtain a rotated matrix, for now this is rather rudimentary.
            * */

            Vector2d Motion = new Vector2d(
                    (-gamepad1.left_stick_y)
                    * LinearScalar,
                    (-gamepad1.left_stick_x)
                    * LinearScalar
            );

            base.setDrivePowers(
                    new PoseVelocity2d(
                            Motion,
                            - gamepad1.right_stick_x * AngularScalar
                    )
            );

            base.updatePoseEstimate();

            telemetry.addData("x", base.localizer.getPose().position.x);
            telemetry.addData("y", base.localizer.getPose().position.y);
            telemetry.addData("h", base.localizer.getPose().heading.toDouble());
            //telemetry.addData("REVLaunchSpeed", 28 * REVLauncher.getVelocity() / 60);
            telemetry.addData("GBDLaunchSpeed", 28 * GoBildaLauncher.getVelocity() / 60);

            //TARGET LOCATION IS AT (-72;-72).

            Distance_Target = Math.sqrt(Math.pow(base.localizer.getPose().position.x + 72, 2)
                    + Math.pow(base.localizer.getPose().position.y + 72, 2));

            //FUNCTION GENERATED BY QUADRATIC REGRESSION.

            LaunchVelocity = 8.83887 * Distance_Target + 1599.99853;

            ServoAngle = 0.0147314 * Distance_Target - 1;

            if (ServoAngle > 1) {
                ServoAngle = 1;
            } else if (ServoAngle < 0){
                ServoAngle = 0;
            }
            Aimer.setPosition(ServoAngle);

            if (gamepad1.right_trigger != 0) {
                GoBildaLoader.setPower(1);
            } else {
                GoBildaLoader.setPower(0);
            }
            if (gamepad1.right_bumper){
                REVLoader.setPower(1);
            } else if (gamepad1.x){
                REVLoader.setPower(-1);
            } else {
                REVLoader.setPower(0);
            }
            if (gamepad1.a){
                //REVLauncher.setVelocity((-LaunchVelocity / 60.0) * 28);
                GoBildaLauncher.setVelocity((-LaunchVelocity / 60.0) * 28);
            }
            else if (gamepad1.left_trigger != 0){
                //REVLauncher.setVelocity((LaunchVelocity / 60.0) * 28);
                GoBildaLauncher.setVelocity((LaunchVelocity / 60.0) * 28);
            } else {
                //REVLauncher.setVelocity(0);
                GoBildaLauncher.setVelocity(0);
            }
            if (gamepad2.left_stick_x < 0){
                C1.setPower(-0.3);
                C2.setPower(-0.3);
            } else if (gamepad2.left_stick_x > 0){
                C1.setPower(0.3);
                C2.setPower(0.3);
            } else {
                C1.setPower(0);
                C2.setPower(0);
            }
            telemetry.update();
        }

    }

}
