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
//public class AutoBlue_Special extends LinearOpMode {
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
////        makeCase();
//        cazulFOUR();
//
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
//                .lineToLinearHeading(new Pose2d(60, -41, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(30)
//                )
//                .build();
//
//        Trajectory powerShots2 = drive.trajectoryBuilder(new Pose2d(60, -41, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, -48 , Math.toRadians(-2)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//        Trajectory powerShots3 = drive.trajectoryBuilder(new Pose2d(60, -48, Math.toRadians(-2)))
//                .lineToLinearHeading(new Pose2d(60, -56, Math.toRadians(-3)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60, -56, Math.toRadians(-3)))
//                .lineToLinearHeading(new Pose2d(75, -7,Math.toRadians(90)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory prepareBounceBack1 = drive.trajectoryBuilder(new Pose2d(75, -7, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(70,-10,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory prepareBounceBack2 = drive.trajectoryBuilder(new Pose2d(70, -10, 0))
//                .lineToLinearHeading(new Pose2d(113, -10,Math.toRadians(-45)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .build();
//
//        Trajectory bounceBack = drive.trajectoryBuilder(new Pose2d(113, -10, Math.toRadians(-45)))
//                .lineToLinearHeading(new Pose2d(115, -50,Math.toRadians(-90)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .build();
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(115, -50, 0))
//                .lineToLinearHeading(new Pose2d(58, -31,Math.toRadians(2)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.7, () -> {
//                    wobble.setPower(-1);
//                })
//                .build();
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(58, -31, Math.toRadians(2)))
//                .lineToLinearHeading(new Pose2d(31, -23.7,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(31, -23.7,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(77, -13,Math.toRadians(-110)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .addTemporalMarker(0.2, () -> {
//                    wobble.setPower(1);
//                })
//                .addTemporalMarker(0.35, () -> {
//                    wobble.setPower(0);
//                })
//                .build();
//
//
//
//        shooter(0.68);
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
//        sleep(100);
//        drive.followTrajectory(prepareBounceBack1);
//
//        intake.setPower(1);
//        drive.followTrajectory(prepareBounceBack2);
//        drive.followTrajectory(bounceBack);
//
//
//        drive.followTrajectory(shootHigh);
//        wobble.setPower(0);
//        shooter(0.84);
//        sleep(2500);
//        intake.setPower(0);
//        //1
//        sleep(400);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //2
//        sleep(400);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //3
//        sleep(400);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//
//
//        drive.followTrajectory(getWobble);
//        shooter(0);
//        wobbleServo.setPosition(0);
//        sleep(300);
//        drive.followTrajectory(dropSecondWobble);
//        sleep(200);
//        wobbleServo.setPosition(0.72);
//        sleep(300);
//        wobble.setPower(1);
//        sleep(400);
//        wobble.setPower(0);
//
//    }
//    private void cazulONE(){
//
//        Trajectory prepareDropWobble = drive.trajectoryBuilder(new Pose2d(0, 0,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, 0,Math.toRadians(0)),
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
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60, 0,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(78, -10.2,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory powerShots = drive.trajectoryBuilder(new Pose2d(78, -10.2,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, -43, Math.toRadians(0)),
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
//        Trajectory powerShots2 = drive.trajectoryBuilder(new Pose2d(60, -43, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(60, -49, Math.toRadians(-2)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//        Trajectory powerShots3 = drive.trajectoryBuilder(new Pose2d(60, -49, Math.toRadians(-2)))
//                .lineToLinearHeading(new Pose2d(60, -54, Math.toRadians(-4)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(60, -54,Math.toRadians(-4)))
//                .lineToLinearHeading(new Pose2d(30.3, -28.5,Math.toRadians(-10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .build();
//
//        Trajectory prepareForRings = drive.trajectoryBuilder(new Pose2d(30.3, -28.5,Math.toRadians(-10)))
//                .lineToLinearHeading(new Pose2d(20, -7,Math.toRadians(-15)),
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
//        Trajectory getRing = drive.trajectoryBuilder(new Pose2d(20, -7,Math.toRadians(-15)))
//                .lineToLinearHeading(new Pose2d(55, -3, Math.toRadians(-15)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory prepareForWobble = drive.trajectoryBuilder(new Pose2d(55, -3,Math.toRadians(-15)))
//                .lineToLinearHeading(new Pose2d(55, -37, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(55, -37,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(113, -27,Math.toRadians(-40)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory bounceBack = drive.trajectoryBuilder(new Pose2d(113, -27,Math.toRadians(-40)))
//                .lineToLinearHeading(new Pose2d(115, -55,Math.toRadians(-90)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(115, -55,Math.toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(58, -30,Math.toRadians(4)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .addTemporalMarker(0.3, () -> {
//                    shooter(0.84);
//                })
//                .build();
//
//        Trajectory park = drive.trajectoryBuilder(new Pose2d(58, -30,Math.toRadians(4)))
//                .lineToLinearHeading(new Pose2d(66, -20,Math.toRadians(0)),
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
//        drive.followTrajectory(prepareDropWobble);
//        drive.followTrajectory(dropWobble);
//        wobbleServo.setPosition(0.72);
//        shooter(0.68);
//        sleep(500);
//
//
//        drive.followTrajectory(powerShots);
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
//        sleep(300);
//        wobbleServo.setPosition(0);
//        sleep(100);
//        intake.setPower(1);
//
//
//        drive.followTrajectory(prepareForRings);
//        shooter(0.82);
//        drive.followTrajectory(getRing);
//
//        sleep(1500);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//
//        drive.followTrajectory(prepareForWobble);
//        shooter(0);
//
//        drive.followTrajectory(dropSecondWobble);
//        wobbleServo.setPosition(0.72);
//        wobble.setPower(1);
//        sleep(300);
//        wobble.setPower(0);
//        drive.followTrajectory(bounceBack);
//
//        drive.followTrajectory(shootHigh);
//
//        //1
//        sleep(700);
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
//        sleep(300);
//        drive.followTrajectory(park);
//
//    }
//    private void cazulFOUR() {
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .lineToLinearHeading(new Pose2d(113, -6,Math.toRadians(45)),
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
//        Trajectory powerShots = drive.trajectoryBuilder(new Pose2d(113, -6,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(56, -37, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .addTemporalMarker(0.5, () -> {
//                    wobble.setPower(-1);
//                })
//                .build();
//
//
//        Trajectory powerShots2 = drive.trajectoryBuilder(new Pose2d(56, -37,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(56, -48, Math.toRadians(-2)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//        Trajectory powerShots3 = drive.trajectoryBuilder(new Pose2d(56, -48, Math.toRadians(-2)))
//                .lineToLinearHeading(new Pose2d(56, -52, Math.toRadians(-4)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(56, -52,Math.toRadians(-4)))
//                .lineToLinearHeading(new Pose2d(28.8, -21,Math.toRadians(-10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .build();
//
//        Trajectory prepareForRings = drive.trajectoryBuilder(new Pose2d(28.8, -21,Math.toRadians(-10)))
//                .lineToLinearHeading(new Pose2d(15.5, -11,Math.toRadians(-10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .addTemporalMarker(0.4, () -> {
//                    wobble.setPower(1);
//                })
//                .addTemporalMarker(0.6, () -> {
//                    wobble.setPower(0);
//                })
//                .build();
//
//
//        Trajectory pushRings = drive.trajectoryBuilder(new Pose2d(15.5 , -11,Math.toRadians(-10)))
//                .lineToLinearHeading(new Pose2d(30, -4, Math.toRadians(-10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory getRings = drive.trajectoryBuilder(new Pose2d(30, -4, Math.toRadians(-9)))
//                .lineToLinearHeading(new Pose2d(43, -4, Math.toRadians(-9)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(6, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(7)
//                )
//                .addTemporalMarker(0.02, () -> {
//                    intake.setPower(0.7);
//                })
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(43, -4, Math.toRadians(-50)))
//                .lineToLinearHeading(new Pose2d(130, -20, Math.toRadians(-120)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .addTemporalMarker(0.5, () -> {
//                    shooter(-0.3);
//                })
//                .addTemporalMarker(1, () -> {
//                    shooter(0);
//                })
//                .build();
//
//        Trajectory bounceBacks = drive.trajectoryBuilder(new Pose2d(130, -20, Math.toRadians(-120)))
//                .lineToLinearHeading(new Pose2d(117, -45, Math.toRadians(-90)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
////                .addTemporalMarker(0.0002, () -> {
////                    wobbleServo.setPosition(0.72);
////                })
//                .build();
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(117, -45, Math.toRadians(-90)))
//                .lineToLinearHeading(new Pose2d(59, -15,Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory park = drive.trajectoryBuilder(new Pose2d(59, -15,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(66, -15,Math.toRadians(0)),
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
//        drive.followTrajectory(dropWobble);
//        wobbleServo.setPosition(0.72);
//        shooter(0.68);
//        drive.followTrajectory(powerShots);
//
//        wobble.setPower(0);
//
//        //1
//
//        sleep(300);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        shooter(0.64);
//
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
//        shooter(0.83);
//        drive.followTrajectory(pushRings);
//
//        sleep(1400);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//
//        drive.followTrajectory(getRings);
//
//        //1
//        sleep(300);
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
//        drive.followTrajectory(dropSecondWobble);
//        wobbleServo.setPosition(0.72);
//        sleep(100);
//
//        drive.followTrajectory(bounceBacks);
//        shooter(0.81);
//        drive.followTrajectory(shootHigh);
//        intake.setPower(0);
//
//        //1
//        sleep(200);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //2
//        sleep(200);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        //3
////        sleep(200);
////        manuta.setPosition(0.5);
////        sleep(100);
////        manuta.setPosition(0);
//
//        drive.followTrajectory(park);
//
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
//        antena = hardwareMap.get(Servo.class,"antena");
//        sabie = hardwareMap.get(Servo.class,"sabie");
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
//        antena.setDirection(Servo.Direction.REVERSE);
////        wobbleServo.setDirection(Servo.Direction.REVERSE);
//
//
//        wobbleServo.setPosition(0);
//        manuta.setPosition(0);
//
//        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
//    //?
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