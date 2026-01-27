package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

/*
* Before anything, we have a few notes:
*
* Positive direction is clockwise from the perspective opposite to the motor axially.
*
* We are using REV 41 and GoBilda 5203 for the launcher because we are out of resources.
*
* Both motor types count 28 ticks per revolution.
*
* Latest measurement gives 12150 ticks per 23.8125 in, for our tiles are not perfect.
* */

@Autonomous(name = "BLUE")
public class _AutonomousBlue_ extends LinearOpMode{

    public static class ArtefactHandler{

        private final DcMotorEx REVLauncherR, REVLauncherL;
        private final DcMotor REVLoader, GoBildaIntake;
        Servo Aimer;
        double LaunchSpeed = 2500;

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
        * Launcher function: y = 0.0521006 x^2 + 0.881022 x + 2200.11667
        *
        * Aimer function: y = -0.000173587 x^2 + 0.04419 x - 2.19982
        *
        * Each field tile is 24in by 24in.
        *
        * Launch lines: x = |y| + 48 and x = -|y|.
        * */

        public double TPR = 28;

        //Overclocking time!
        PIDFCoefficients PIDF = new PIDFCoefficients(1000000, 1, 0, 19);

        private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        public ArtefactHandler (HardwareMap hardwareMap){
            REVLauncherR = hardwareMap.get(DcMotorEx.class, "REVR");
            REVLauncherR.setDirection(DcMotorSimple.Direction.FORWARD);
            REVLauncherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
            REVLauncherL = hardwareMap.get(DcMotorEx.class, "REVL");
            REVLauncherL.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLauncherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLauncherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            REVLauncherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

            GoBildaIntake = hardwareMap.get(DcMotor.class, "GoBildaIntake");
            GoBildaIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            GoBildaIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            REVLoader = hardwareMap.get(DcMotor.class, "REVTransfer");
            REVLoader.setDirection(DcMotorSimple.Direction.REVERSE);
            REVLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            Aimer = hardwareMap.get(Servo.class, "Hood");
            Aimer.setPosition(0);
        }

        /*
        * Both GoBilda and REV have their motors running on 28 ticks each revolution.
        * Desired speed in rotation per minute will be divided by 60 to get per second rotation.
        * Multiply that with 28 we have the desired angular speed.
        * */

        //INTAKE.

        public class Take_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBildaIntake.setPower(1);
                REVLoader.setPower(0.35);
                REVLauncherR.setVelocity((-1100.0 / 60.0) * TPR);
                REVLauncherL.setVelocity((-1100.0 / 60.0) * TPR);
                return false;
            }
        }
        public Action takeArtefact(){
            return new Take_Artefact();
        }

        public class Discard_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBildaIntake.setPower(-1);
                REVLoader.setPower(-0.79);
                return false;
            }
        }
        public Action discardArtefact(){
            return new Discard_Artefact();
        }

        public class Keep_Artefact implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                return false;
            }
        }
        public Action keepArtefact(){
            return new Keep_Artefact();
        }

        //TURRET.

        public class Spool_Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                Aimer.setPosition(0);
                REVLauncherR.setVelocity((LaunchSpeed / 60) * TPR);
                REVLauncherL.setVelocity((LaunchSpeed / 60) * TPR);
                //Here a tolerance of 10 ticks will be allowed because we have had enough.
                return (REVLauncherR.getVelocity() + REVLauncherL.getVelocity()) < (LaunchSpeed / 60) * TPR - 10;
            }
        }
        public Action spoolUp() {
            return new Spool_Up();
        }

        public class Launch_Artefact implements Action {
            private boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetry){
                if (!init) {
                    timer.reset();
                    init = true;
                }
                GoBildaIntake.setPower(1);
                REVLoader.setPower(1);
                return timer.time(TimeUnit.MILLISECONDS) < 1300;
            }}

        public Action launchArtefact() {
            return new Launch_Artefact();
        }

        public class Halt implements Action {
            private boolean init = false;
            @Override
            public boolean run (@NonNull TelemetryPacket telemetry){
                if (!init) {
                    timer.reset();
                    init = true;
                }
                REVLauncherR.setVelocity(0);
                REVLauncherL.setVelocity(0);
                REVLauncherR.setPower(0);
                REVLauncherL.setPower(0);
                GoBildaIntake.setPower(0);
                REVLoader.setPower(0);
                return timer.time(TimeUnit.MILLISECONDS) < 500;
            }
        }

        public Action halt() {
            return new Halt();
        }

    }

    //PATH.

    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d Location = new Pose2d(-61, -6, Math.toRadians(0));
        MecanumDrive Base = new MecanumDrive(hardwareMap, Location);
        ArtefactHandler artefactHandler = new ArtefactHandler(hardwareMap);
        waitForStart();
        Action Path1 = Base.actionBuilder(Location)
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(225))
                .build();
        Action Path2 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(-8, -20), Math.toRadians(270))
                .build();
        Action Path3 = Base.actionBuilder(new Pose2d(-8, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(-8, -52))
                .build();
        Action Path4 = Base.actionBuilder(new Pose2d(-8, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(225))
                .build();
        Action Path5 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(12, -20), Math.toRadians(270))
                .build();
        Action Path6 = Base.actionBuilder(new Pose2d(12, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, -52))
                .build();
        Action Path7 = Base.actionBuilder(new Pose2d(12, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(225))
                .build();
        Action Path8 = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(36, -20), Math.toRadians(270))
                .build();
        Action Path9 = Base.actionBuilder(new Pose2d(36, -20, Math.toRadians(270)))
                .strafeTo(new Vector2d(36, -52))
                .build();
        Action PathA = Base.actionBuilder(new Pose2d(36, -52, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-24, -24), Math.toRadians(220))
                .build();
        Action PathB = Base.actionBuilder(new Pose2d(-24, -24, Math.toRadians(220)))
                .strafeToSplineHeading(new Vector2d(0, -24), Math.toRadians(0))
                .build();

        ParallelAction Spool1 = new ParallelAction(
                Path1,
                artefactHandler.spoolUp()
        );
        ParallelAction Defer2 = new ParallelAction(
                Path2,
                artefactHandler.discardArtefact()
        );
        ParallelAction Refer3 = new ParallelAction(
                Path3,
                artefactHandler.takeArtefact()
        );
        ParallelAction Spool4 = new ParallelAction(
                Path4,
                artefactHandler.spoolUp(),
                artefactHandler.keepArtefact()
        );
        ParallelAction Defer5 = new ParallelAction(
                Path5,
                artefactHandler.discardArtefact()
        );
        ParallelAction Refer6 = new ParallelAction(
                Path6,
                artefactHandler.takeArtefact()
        );
        ParallelAction Spool7 = new ParallelAction(
                Path7,
                artefactHandler.spoolUp(),
                artefactHandler.keepArtefact()
        );
        ParallelAction Defer8 = new ParallelAction(
                Path8,
                artefactHandler.discardArtefact()
        );
        ParallelAction Refer9 = new ParallelAction(
                Path9,
                artefactHandler.takeArtefact()
        );
        ParallelAction SpoolA = new ParallelAction(
                PathA,
                artefactHandler.spoolUp(),
                artefactHandler.keepArtefact()
        );

        SequentialAction Blue = new SequentialAction(
                Spool1,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                Defer2,
                Refer3,
                artefactHandler.keepArtefact(),
                Spool4,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                Defer5,
                Refer6,
                artefactHandler.keepArtefact(),
                Spool7,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                Defer8,
                Refer9,
                artefactHandler.keepArtefact(),
                SpoolA,
                artefactHandler.launchArtefact(),
                artefactHandler.halt(),
                PathB
        );

        Actions.runBlocking(Blue);
    }
}

//ENDING POSITION: (+60;-12); HEADING 216.
