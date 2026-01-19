package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

/*
*
* Trước khi đi vào phần mã chính, chúng ta sẽ có một vài lưu ý để thuận tiện lập trình.
*
* Mô-tơ coi chiều dương là chiều quay ngược chiều kim đồng hồ theo hướng mô-tơ.
*
* Mô-tơ REV và GoBilda hiện tại đang dùng REV 41 và GoBilda dòng 5203.
*
* */

@Autonomous(name = "BLUE")
public class _AutonomousBlue_ extends LinearOpMode{

    //LẤY BÓNG TỪ SÂN.
    public class Intake{

        private DcMotorEx GoBildaLauncher, REVLauncher;
        private DcMotor GoBilda3_Intake;
        private DcMotor REV_Loader;

        public double TPR = 28;

        public Intake(HardwareMap hardwareMap){
            GoBildaLauncher = hardwareMap.get(DcMotorEx.class, "GoBildaLauncher");
            GoBildaLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            GoBildaLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            GoBildaLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncher = hardwareMap.get(DcMotorEx.class, "REVLauncher");
            REVLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            GoBilda3_Intake = hardwareMap.get(DcMotor.class, "GoBildaIntake");
            GoBilda3_Intake.setDirection(DcMotorSimple.Direction.REVERSE);
            GoBilda3_Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REV_Loader = hardwareMap.get(DcMotor.class, "REVLoader");
            REV_Loader.setDirection(DcMotorSimple.Direction.REVERSE);
            REV_Loader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //NHẶT BÓNG.
        public class Take_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBilda3_Intake.setPower(1);
                REV_Loader.setPower(0.35);
                return false;
            }
        }
        public Action takeArtefact(){
            return new Take_Artefact();
        }

        //NHẢ BÓNG.
        public class Discard_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBilda3_Intake.setPower(-1);
                REV_Loader.setPower(-0.79);
                GoBildaLauncher.setVelocity((-100.0 / 60.0) * TPR);
                REVLauncher.setVelocity((-100.0 / 60.0) * TPR);
                return false;
            }
        }
        public Action discardArtefact(){
            return new Discard_Artefact();
        }

        //GIỮ BÓNG.
        public class Keep_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBilda3_Intake.setPower(0);
                REV_Loader.setPower(0);
                return false;
            }
        }
        public Action keepArtefact(){
            return new Keep_Artefact();
        }
    }

    public class Launcher{

        private DcMotorEx GoBildaLauncher, REVLauncher;

        private DcMotor REVLoader, GoBildaIntake;

        private Servo Aimer;

        private double LaunchSpeed = 2350;

        public double TPR = 28;

        public ElapsedTime timer = new ElapsedTime();

        /*
         *
         * Gia tốc tiếp tuyến bánh bắn:
         *
         *       v = r * o.
         *
         *       Với v là vận tốc, r là bán kính, o là vận tốc góc.
         *
         * Ta có khoảng cách đến điểm rơi bóng tính bằng công thức:
         *
         *       v^2 * sin(2 * z) / g.
         *
         *       Với v là vận tốc bắn, z là góc bắn, và g là gia tốc trọng trường.
         *
         * Servo của rô-bốt sẽ chỉnh góc bắn trong khoảng 20 độ đến 60 độ so với phương ngang.
         *
         * Mã sử dụng các công thức ở khoảng dòng 200.
         *
         * */

        private double ServoMaxAngle = 60;
        private double ServoMinAngle = 20;
        public double AngularSpeed;
        public double FlywheelRadius = 4.5;

        public double AverageLauncherVelocity;
        public double LandingDistance = 28.284;         //KHOẢNG CÁCH CHO SẴN.
        public double g = 9.8;                          //GIA TỐC TRỌNG TRƯỜNG.
        public double ServoPosition = 0;

        public double AngularOffset = 0.1;

        public Launcher (HardwareMap hardwareMap){
            GoBildaLauncher = hardwareMap.get(DcMotorEx.class, "GoBildaLauncher");
            GoBildaLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            GoBildaLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            GoBildaLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncher = hardwareMap.get(DcMotorEx.class, "REVLauncher");
            REVLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            GoBildaIntake = hardwareMap.get(DcMotor.class, "GoBildaIntake");
            GoBildaIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            GoBildaIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLoader = hardwareMap.get(DcMotor.class, "REVLoader");
            REVLoader.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            Aimer = hardwareMap.get(Servo.class, "ServoAimer");
            Aimer.setPosition(0);
        }

        /*
        * Cả mô-tơ GoBilda và REV đều đọc ra 28 xung mỗi vòng quay.
        * Tốc độ mong muốn là khoảng 2700RPM, tức là 450RPS.
        * Nhân vòng mỗi giây với số xung mỗi vòng ta có 12600 xung mỗi giây.
        * */

        public class Spool_Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBildaLauncher.setVelocity((LaunchSpeed / 60) * TPR);
                REVLauncher.setVelocity((LaunchSpeed / 60) * TPR);
                return (GoBildaLauncher.getVelocity() + REVLauncher.getVelocity()) / 2 < (LaunchSpeed / 60) * TPR - 10;
            }
        }

        public Action spoolUp() {
            return new Spool_Up();
        }

        public class PushBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                REVLoader.setPower(-0.5);
                GoBildaIntake.setPower(-0.3);
                REVLoader.setPower(0);
                GoBildaIntake.setPower(0);
                return false;
            }
        }

        public Action pushBack(){
            return new PushBack();
        }

        public class Launch_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry) {
                GoBildaIntake.setPower(1);
                REVLoader.setPower(1);
                sleep(150);
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                sleep(150);
                GoBildaIntake.setPower(1);
                REVLoader.setPower(1);
                sleep(150);
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                sleep(150);
                GoBildaIntake.setPower(1);
                REVLoader.setPower(1);
                sleep(150);
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                return false;
                /*
                AverageLauncherVelocity = 0.5 * (GoBildaLauncher.getVelocity() + REVLauncher.getVelocity());
                AngularSpeed = FlywheelRadius * (AverageLauncherVelocity / 28) * (2 * FlywheelRadius * Math.PI);
                ServoPosition = (0.5 * Math.toDegrees(Math.asin((LandingDistance * 2 * g) / Math.pow(AngularSpeed, 2))) - 20) / 40;
                if ((timer.time(TimeUnit.MILLISECONDS) > 300 & timer.time(TimeUnit.MILLISECONDS) < 400)
                    | (timer.time(TimeUnit.MILLISECONDS) > 600 & timer.time(TimeUnit.MILLISECONDS) < 700)
                    | (timer.time(TimeUnit.MILLISECONDS) > 900 & timer.time(TimeUnit.MILLISECONDS) < 1000)) {
                    GoBildaIntake.setPower(0);
                    REVLoader.setPower(0);
                    if (ServoPosition > ServoMaxAngle) {
                        if (Aimer.getPosition() + AngularOffset > 0 | Aimer.getPosition() - AngularOffset < 0){
                            Aimer.setPosition(Aimer.getPortNumber());
                        } else {
                            Aimer.setPosition(0);
                        }
                    } else if (ServoPosition < ServoMinAngle) {
                        if (Aimer.getPosition() + AngularOffset > 1 | Aimer.getPosition() - AngularOffset < 1){
                            Aimer.setPosition(Aimer.getPosition());
                        } else {
                            Aimer.setPosition(1);
                        }
                    } else {
                        if (Aimer.getPosition() + AngularOffset > ServoPosition | Aimer.getPosition() - AngularOffset < ServoPosition){
                            Aimer.setPosition(Aimer.getPosition());
                        } else {
                            Aimer.setPosition(ServoPosition);
                        }
                    }
                    GoBildaIntake.setPower(1);
                    REVLoader.setPower(1);
                    return timer.time(TimeUnit.MILLISECONDS) < 1500;
                }
                 */
            }
        }

        public Action launchArtefact() {
            return new Launch_Artefact();
        }

        public class Halt implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket telemetry){
                GoBildaLauncher.setPower(0);
                REVLauncher.setPower(0);
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                return false;
            }
        }

        public Action halt() {
            return new Halt();
        }

    }

    //ĐƯỜNG ĐI.

    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d Location = new Pose2d(60, -15, Math.toRadians(180));
        MecanumDrive Base = new MecanumDrive(hardwareMap, Location);
        Intake intake = new Intake(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        waitForStart();
        Action ReadMotif = Base.actionBuilder(Location)
                //ĐỌC MÃ BẰNG LIMELIGHT CHO VÀO ĐÂY.
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(220))
                .build();
        Action ToRow1 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(220)))
                .strafeToSplineHeading(new Vector2d(-12, -20), Math.toRadians(270))
                .build();
        Action GetRow1 = Base.actionBuilder(new Pose2d(-12, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(-12, -52))
                .build();
        Action LeaveRow1 = Base.actionBuilder(new Pose2d(-12, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24,-24), Math.toRadians(220))
                .build();
        Action ToRow2 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(220)))
                .strafeToSplineHeading(new Vector2d(12, -20), Math.toRadians(270))
                .build();
        Action GetRow2 = Base.actionBuilder(new Pose2d(12, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, -52))
                .build();
        Action LeaveRow2 = Base.actionBuilder(new Pose2d(12, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24,-24), Math.toRadians(220))
                .build();
        Action ToRow3 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(220)))
                .strafeToSplineHeading(new Vector2d(36, -20), Math.toRadians(270))
                .build();
        Action GetRow3 = Base.actionBuilder(new Pose2d(36, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(36, -52))
                .build();
        Action LeaveRow3 = Base.actionBuilder(new Pose2d(36, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(220))
                .build();
        Action End = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(220)))
                .strafeToSplineHeading(new Vector2d(60, -15), Math.toRadians(180))
                .build();

        Actions.runBlocking(
                new SequentialAction(

                        ReadMotif,                  //ĐỌC MÃ GHI THỨ TỰ BÓNG VÀ ĐẾN VỊ TRÍ BẮN BÓNG.

                        launcher.pushBack(),
                        launcher.spoolUp(),         //KHỞI ĐỘNG MÁY BẮN.
                        launcher.launchArtefact(),  //BẮN BÓNG.
                        launcher.halt(),            //DỪNG MÁY BẮN.
                        intake.discardArtefact(),   //NHẢ BÓNG.

                        ToRow1,                     //ĐI ĐẾN HÀNG BÓNG THỨ NHẤT.
                        intake.takeArtefact(),      //LẤY BÓNG.
                        GetRow1,
                        intake.keepArtefact(),      //GIỮ BÓNG.
                        LeaveRow1,                  //QUAY LẠI VỊ TRÍ BẮN.

                        launcher.pushBack(),
                        launcher.spoolUp(),         //KHỞI ĐỘNG MÁY BẮN.
                        launcher.launchArtefact(),  //BẮN BÓNG.
                        launcher.halt(),            //DỪNG MÁY BẮN.
                        intake.discardArtefact(),   //NHẢ BÓNG.

                        ToRow2,                     //ĐI ĐẾN HÀNG HAI.
                        intake.takeArtefact(),      //LẤY BÓNG.
                        GetRow2,
                        intake.keepArtefact(),      //GIỮ BÓNG.
                        LeaveRow2,                  //VỀ VỊ TRÍ BẮN.

                        launcher.pushBack(),
                        launcher.spoolUp(),         //KHỞI ĐỘNG MÁY BẮN.
                        launcher.launchArtefact(),  //BẮN BÓNG.
                        launcher.halt(),            //DỪNG MÁY BẮN.
                        intake.discardArtefact(),   //NHẢ BÓNG.

                        ToRow3,                     //ĐI ĐẾN HÀNG THỨ BA.
                        intake.takeArtefact(),      //LẤY BÓNG.
                        GetRow3,
                        intake.keepArtefact(),      //GIỮ BÓNG.
                        LeaveRow3,                  //VỀ VỊ TRÍ BẮN.

                        launcher.pushBack(),
                        launcher.spoolUp(),         //KHỞI ĐỘNG MÁY BẮN.
                        launcher.launchArtefact(),  //BẮN BÓNG.
                        launcher.halt(),            //DỪNG MÁY BẮN.
                        intake.discardArtefact(),   //NHẢ BÓNG.
                        intake.keepArtefact(),

                        End
                )
        );
    }
}
