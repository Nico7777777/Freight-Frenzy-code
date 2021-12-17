//package org.firstinspires.ftc.teamcode.drive.advanced;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.apache.commons.math3.analysis.function.Pow;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.opmode.OpenCV;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.Arrays;
//
//import static org.openftc.easyopencv.OpenCvCameraFactory.*;
//
//@Autonomous
//public class AutoRed_Special extends LinearOpMode {
//
//    //drivetrain
//    SampleMecanumDrive drive;
//
//    //servo
//    Servo manuta,wobbleServo,antena,sabie;
//
//    //motoare
//    DcMotor shooter1,shooter2,wobble,intake;
//
//    //opencv
//    private OpenCvCamera webcam;
//    private UltimateGoalPipeline pipeline;
//
//    //opmode
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //init
//        initHardware();
//        initOpenCv();
//        while(!opModeIsActive() && !isStopRequested()){
//            telemetry.addData("case: " , pipeline.getAnalysis());
//            telemetry.update();
//        }
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        makeCase();
//    }
//
//    //alege cazul
//    private void makeCase() {
//
//        if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.FOUR)
//            cazulFOUR();
//
//        else if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.ONE)
//            cazulONE();
//
//        else if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.ZERO)
//            cazulZERO();
//    }
//
//
//
//    //cazuri
//    private void cazulZERO(){
//
//        Trajectory powerShots = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .lineToLinearHeading(new Pose2d(60, 24.5, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory powerShots2 = drive.trajectoryBuilder(new Pose2d(60, 24.5, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, 35, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//        Trajectory powerShots3 = drive.trajectoryBuilder(new Pose2d(60, 35, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, 42, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60, 42, Math.toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(80, 8,Math.toRadians(-90)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory prepareBounceBack1 = drive.trajectoryBuilder(new Pose2d(80, 8, Math.toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(70, 14, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory prepareBounceBack2 = drive.trajectoryBuilder(new Pose2d(70, 14, 0))
//                .lineToLinearHeading(new Pose2d(116, 14,Math.toRadians(45)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory bounceBack = drive.trajectoryBuilder(new Pose2d(116, 14, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(116, 50,Math.toRadians(90)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.2, () -> {
//                    intake.setPower(0.7);
//                })
//                .addTemporalMarker(1.8, () -> {
//                    intake.setPower(0.5);
//                })
//                .build();
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(116, 50, 0))
//                .lineToLinearHeading(new Pose2d(60, 15, Math.toRadians(5)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .addTemporalMarker(0.2, () -> {
//                    wobble.setPower(-1);
//                })
//                .addTemporalMarker(1.8, () -> {
//                    wobble.setPower(0);
//                })
//                .addTemporalMarker(1, () -> {
//                    shooter(0.89);
//                })
//                .build();
//
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(60, 15, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(30.8, 27.7, Math.toRadians(-10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(30.8 , 27.7, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(77, 20, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .addTemporalMarker(0.1, () -> {
//                    wobble.setPower(1);
//                })
//                .addTemporalMarker(0.3, () -> {
//                    wobble.setPower(0);
//                })
//                .build();
//
//
//        shooter(0.84);
//        drive.followTrajectory(powerShots);
//
//        //1
//        sleep(300);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //2
//        drive.followTrajectory(powerShots2);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //3
//        drive.followTrajectory(powerShots3);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(100);
//
//
//        drive.followTrajectory(dropWobble);
//        shooter(0);
//        wobbleServo.setPosition(0.72);
//        sleep(200);
//
//
//        drive.followTrajectory(prepareBounceBack1);
//        shooter(-0.3);
//        intake.setPower(1);
//        drive.followTrajectory(prepareBounceBack2);
//        intake.setPower(1);
//        drive.followTrajectory(bounceBack);
//        shooter(0);
//
//
//
//        drive.followTrajectory(shootHigh);
//        intake.setPower(0);
//        wobble.setPower(0);
//        //1
//        sleep(1000);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //2
//        sleep(300);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //3
//        sleep(300);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//
//
//        drive.followTrajectory(getWobble);
//        shooter(0);
//        sleep(100);
//        wobbleServo.setPosition(0);
//        sleep(400);
//
//        drive.followTrajectory(dropSecondWobble);
//        drive.turn(Math.toRadians(95));
//        wobbleServo.setPosition(0.72);
//        sleep(300);
//        wobble.setPower(1);
//        sleep(500);
//
//
//
//
//
//
//
//    }
//
//    private void cazulONE(){
//
//        Trajectory prepareDropWobble = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(69, 0, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(68, 0, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(80, 14, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory powerShots = drive.trajectoryBuilder(new Pose2d(80, 14, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, 26.5, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(30)
//                )
//                .addTemporalMarker(0.02, () -> {
//                    wobble.setPower(-1);
//                })
//                .build();
//
//
//
//        Trajectory powerShots2 = drive.trajectoryBuilder(new Pose2d(60, 26.5, Math.toRadians(-1)))
//                .lineToLinearHeading(new Pose2d(60, 35, Math.toRadians(-1)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//        Trajectory powerShots3 = drive.trajectoryBuilder(new Pose2d(60, 35, Math.toRadians(-1)))
//                .lineToLinearHeading(new Pose2d(60, 40.5, Math.toRadians(-1)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(60, 40.5, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(32, 30,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .build();
//
//        Trajectory prepareForRings = drive.trajectoryBuilder(new Pose2d(32, 30, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(17, 21.5,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//        Trajectory getRing = drive.trajectoryBuilder(new Pose2d(17, 21.5,Math.toRadians(-10)))
//                .lineToLinearHeading(new Pose2d(53.4, 23, Math.toRadians(-10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .addTemporalMarker(0.1, () -> {
//                    wobble.setPower(1);
//                })
//                .addTemporalMarker(0.4, () -> {
//                    wobble.setPower(0);
//                })
//                .build();
//
//        Trajectory prepareForWobble = drive.trajectoryBuilder(new Pose2d(53.4, 23,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(54, 33, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(54, 33,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(116, 33,Math.toRadians(25)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory bounceBack = drive.trajectoryBuilder(new Pose2d(116, 33 ,Math.toRadians(60)))
//                .lineToLinearHeading(new Pose2d(116, 60,Math.toRadians(70)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(116 , 60, Math.toRadians(30)))
//                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(-31)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .addTemporalMarker(0.7, () -> {
//                    shooter(0.93);
//                })
//                .build();
//
//        Trajectory park = drive.trajectoryBuilder(new Pose2d(60, 60 ,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(70, 60,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//
//
//
//
//
//
//
//        drive.followTrajectory(prepareDropWobble);
//        drive.followTrajectory(dropWobble);
//        wobbleServo.setPosition(0.72);
//        shooter(0.80);
//        sleep(500);
//
//
//        drive.followTrajectory(powerShots);
//
//        wobble.setPower(0);
//
//        //1
//        sleep(300);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //2
//        drive.followTrajectory(powerShots2);
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //3
//        drive.followTrajectory(powerShots3);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//
//        sleep(200);
//        shooter(0);
//
//        drive.followTrajectory(getWobble);
//        wobbleServo.setPosition(0);
//        sleep(150);
//        intake.setPower(1);
//
//
//        drive.followTrajectory(prepareForRings);
//        shooter(0.92);
//        drive.followTrajectory(getRing);
//
//        sleep(1200);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//
//        drive.followTrajectory(prepareForWobble);
//        shooter(0);
//        shooter(-0.3);
//
//
//        drive.followTrajectory(dropSecondWobble);
//        wobbleServo.setPosition(0.72);
//        sleep(300);
//        wobble.setPower(1);
//        sleep(200);
//        wobble.setPower(0);
//        drive.turn(Math.toRadians(90));
//        drive.followTrajectory(bounceBack);
//        shooter(0);
//
//
//
//        drive.followTrajectory(shootHigh);
//
//        drive.turn(Math.toRadians(-26.6));
//
//        //1
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(200);
//        manuta.setPosition(0);
//        //2
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(200);
//        manuta.setPosition(0);
////        //3
////        sleep(300);
////        manuta.setPosition(0.5);
////        sleep(100);
////        manuta.setPosition(0);
//
//
//        drive.followTrajectory(park);
//
//
//
//
//
//
//    }
//
//
//    private void cazulFOUR() {
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .lineToLinearHeading(new Pose2d(108, -7.5,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//        Trajectory powerShots = drive.trajectoryBuilder(new Pose2d(108, -7.5, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, 27, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .addTemporalMarker(0.02, () -> {
//                    wobble.setPower(-1);
//                })
//                .addTemporalMarker(1.7, () -> {
//                    wobble.setPower(0);
//                })
//                .build();
//
//        Trajectory powerShots2 = drive.trajectoryBuilder(new Pose2d(60, 27, Math.toRadians(-1)))
//                .lineToLinearHeading(new Pose2d(60, 35, Math.toRadians(-1)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//        Trajectory powerShots3 = drive.trajectoryBuilder(new Pose2d(60, 35, Math.toRadians(-1)))
//                .lineToLinearHeading(new Pose2d(60, 40, Math.toRadians(-1)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(60, 40, Math.toRadians(-1)))
//                .lineToLinearHeading(new Pose2d(33, 35, Math.toRadians(10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory prepareForRings = drive.trajectoryBuilder(new Pose2d(33, 35 , Math.toRadians(10)))
//                .lineToLinearHeading(new Pose2d(20, 29, Math.toRadians(-8)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//        Trajectory pushRings = drive.trajectoryBuilder(new Pose2d(20, 29,Math.toRadians(-8)))
//                .lineToLinearHeading(new Pose2d(32.3, 23, Math.toRadians(-8)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .addTemporalMarker(0.1, () -> {
//                    wobble.setPower(1);
//                })
//                .addTemporalMarker(0.3, () -> {
//                    wobble.setPower(0);
//                })
//                .build();
//
//
//        Trajectory getRings = drive.trajectoryBuilder(new Pose2d(32.3, 23, Math.toRadians(-8)))
//                .lineToLinearHeading(new Pose2d(57, 23, Math.toRadians(-8)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(6, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(5)
//                )
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(57, 23, Math.toRadians(20)))
//                .lineToLinearHeading(new Pose2d(120, 16, Math.toRadians(115)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory bounceBacks = drive.trajectoryBuilder(new Pose2d(120, 16, Math.toRadians(30)))
//                .lineToLinearHeading(new Pose2d(120, 50, Math.toRadians(45)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(120, 50, Math.toRadians(45)))
//                .lineToLinearHeading(new Pose2d(63, 31, Math.toRadians(-27)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .addTemporalMarker(0.7, () -> {
//                    shooter(0.95);
//                })
//                .build();
//
//        Trajectory park = drive.trajectoryBuilder(new Pose2d(63, 31, Math.toRadians(-27)))
//                .lineToLinearHeading(new Pose2d(72, 31, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//
//        shooter(0.81);
//        drive.followTrajectory(dropWobble);
//        wobbleServo.setPosition(0.72);
//        drive.followTrajectory(powerShots);
//
//        wobble.setPower(0);
//
//        //1
//
//        sleep(150);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //2
//        drive.followTrajectory(powerShots2);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //3
//        drive.followTrajectory(powerShots3);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//
//        sleep(200);
//        shooter(0);
//
//        drive.followTrajectory(getWobble);
//        wobbleServo.setPosition(0);
//        sleep(300);
//        intake.setPower(1);
//
//
//        drive.followTrajectory(prepareForRings);
//        shooter(0.93);
//        drive.followTrajectory(pushRings);
//
//        intake.setPower(1);
//        sleep(1000);
//
//        manuta.setPosition(0.5);
//        sleep(200);
//        manuta.setPosition(0);
//
//        shooter(0.91);
//        drive.followTrajectory(getRings);
//
//        //1
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(200);
//        manuta.setPosition(0);
//        //2
//        sleep(200);
//        manuta.setPosition(0.5);
//        sleep(200);
//        manuta.setPosition(0);
//        //3
//        sleep(200);
//        manuta.setPosition(0.5);
//        sleep(200);
//        manuta.setPosition(0);
//
//        shooter(0);
//
//
//        drive.followTrajectory(dropSecondWobble);
//        wobbleServo.setPosition(0.72);
//        sleep(100);
//
//        drive.followTrajectory(bounceBacks);
//        drive.followTrajectory(shootHigh);
//        intake.setPower(0);
//
//        //1
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //2
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //3
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//
//        drive.followTrajectory(park);
//    }
//
//
//    //inituri
//    private void initOpenCv(){
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam =  getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"),cameraMonitorViewId);
//        pipeline = new UltimateGoalPipeline(telemetry);
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
//    }
//    private void initHardware(){
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        manuta = hardwareMap.get(Servo.class,"servo");
//     //   antena = hardwareMap.get(Servo.class,"antena");
//     //   sabie = hardwareMap.get(Servo.class,"sabie");
//        wobbleServo = hardwareMap.get(Servo.class, "wobble_servo");
//        wobble= hardwareMap.get(DcMotor.class, "wobble");
//        shooter1 = hardwareMap.get(DcMotor.class,"shooter1");
//        shooter2 = hardwareMap.get(DcMotor.class,"shooter2");
//        intake = hardwareMap.get(DcMotor.class,"intake");
//
//
//
//        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//    //    antena.setDirection(Servo.Direction.REVERSE);
////        wobbleServo.setDirection(Servo.Direction.REVERSE);
//
//
//        wobbleServo.setPosition(0);
//        manuta.setPosition(0);
//    }
//
//
//    //shooter
//    private void shooter(double speed){
//        shooter1.setPower(speed);
//        shooter2.setPower(speed);
//    }
//
//
//
//    //clasa openCv
//    public static class UltimateGoalPipeline extends OpenCvPipeline
//    {
//        Telemetry telemetry;
//
//        public UltimateGoalPipeline(Telemetry telemetry){
//            this.telemetry = telemetry;
//
//        }
//
//        public enum UltimateGoalRings
//        {
//            FOUR,
//            ONE,
//            ZERO
//        }
//
//
//        static final Scalar RED = new Scalar(255, 0, 0);
//
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(461,531);
//
//        static final int REGION_WIDTH = 120;
//        static final int REGION_HEIGHT = 80;
//        static final int FOUR_RING_THRESHOLD = 105; // 93    124 for ZERO
//        static final int ONE_RING_THRESHOLD = 115; //104
//
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg;
//
//        private volatile UltimateGoalRings position = UltimateGoalRings.ZERO;
//
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb );
//            Core.extractChannel(YCrCb, Cb, 2);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            inputToCb(firstFrame);
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            inputToCb(input);
//
//            avg = (int) Core.mean(region1_Cb).val[0];
//            telemetry.addData("avg: ",avg);
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//
//            if(avg <= FOUR_RING_THRESHOLD)
//            {
//                position = UltimateGoalRings.FOUR;
//
//            }
//            else if(avg <= ONE_RING_THRESHOLD)
//            {
//                position = UltimateGoalRings.ONE;
//
//            }
//            else
//            {
//                position = UltimateGoalRings.ZERO;
//            }
//
//            return input;
//        }
//
//        public UltimateGoalRings getAnalysis()
//        {
//            return position;
//        }
//    }
//}