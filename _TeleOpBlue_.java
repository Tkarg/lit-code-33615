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

    PIDFCoefficients PIDF = new PIDFCoefficients(150, 1.5, 0, 17);

    /*
    *
    * Gia tốc tiếp tuyến bánh bắn:
    *
    *       v = r * o
    *
    *       Với v là vận tốc, r là bán kính, o là vận tốc góc.
    *
    * Ta có khoảng cách đến điểm rơi bóng tính bằng công thức:
    *
    *       v^2 * sin(2 * z) / g
    *
    *       Với v là vận tốc bắn, z là góc bắn, và g là gia tốc trọng trường.
    *
    * Servo của rô-bốt sẽ chỉnh góc bắn trong khoảng 20 độ đến 60 độ so với phương ngang.
    *
    * */

    @Override
    public void runOpMode() throws InterruptedException{

        MecanumDrive base = new MecanumDrive(hardwareMap, new Pose2d(0 ,0 ,0));


        /*
        * Mô-tơ quay chiều dương là thuận chiều kim đồng hồ nhìn từ phía ngược lại.
        * */

        GoBildaLauncher = hardwareMap.get(DcMotorEx.class, "GoBildaLauncher");
        GoBildaLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        GoBildaLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        GoBildaLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GoBildaLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        REVLauncher = hardwareMap.get(DcMotorEx.class, "REVLauncher");
        REVLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        REVLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        REVLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        REVLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        /*
        * Tất cả mô-tơ lấy bóng đều quay ngược chiều.
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

        double LaunchVelocity = 2500;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() & !isStopRequested()){

            Vector2d Motion = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            base.setDrivePowers(
                    new PoseVelocity2d(
                            Motion,
                            - gamepad1.right_stick_x
                    )
            );

            base.updatePoseEstimate();

            /*
            * Error encountered here:
            *   IF-ELSE WITHOUT
            * */

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
                REVLauncher.setVelocity((-LaunchVelocity / 60.0) * 28);
                GoBildaLauncher.setVelocity((-LaunchVelocity / 60.0) * 28);
            }
            if (gamepad1.left_trigger != 0){
                REVLauncher.setVelocity((LaunchVelocity / 60.0) * 28);
                GoBildaLauncher.setVelocity((LaunchVelocity / 60.0) * 28);
                telemetry.addData("REVLaunchSpeed", REVLauncher.getVelocity());
                telemetry.addData("GBDLaunchSpeed", GoBildaLauncher.getVelocity());
                telemetry.update();
            } else {
                REVLauncher.setVelocity(0);
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
            if (gamepad2.right_trigger != 0){
                Aimer.setPosition(0);
            } else if (gamepad2.right_bumper){
                Aimer.setPosition(1);
            }
        }

    }

}
