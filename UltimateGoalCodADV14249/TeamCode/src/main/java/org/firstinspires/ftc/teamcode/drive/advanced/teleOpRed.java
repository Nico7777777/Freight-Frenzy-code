 package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;
@TeleOp(group = "advanced")
public class teleOpRed extends LinearOpMode{

    MechanismRed mechanismRed;
    GAMEPAD GAMEPAD1;
    GAMEPAD GAMEPAD2;

    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    @Override
    public void runOpMode() throws InterruptedException {
        GAMEPAD1 = new GAMEPAD(this.gamepad1,telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2,telemetry);
        //mechanism = new Mechanism(hardwareMap,GAMEPAD1,telemetry);
        mechanismRed = new MechanismRed(hardwareMap,GAMEPAD1,GAMEPAD2,telemetry);


        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            GAMEPAD1.run();
            GAMEPAD2.run();
            mechanismRed.mechanism();
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees( poseEstimate.getHeading()));
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    if(gamepad1.left_bumper){
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.3,
                                        -gamepad1.left_stick_x * 0.3,
                                        -gamepad1.right_stick_x * 0.3
                                )
                        );

                    }   else if (gamepad1.x){
                        new Pose2d(
                                gamepad1.left_stick_y * 1.0,
                                gamepad1.left_stick_x * 1.0,
                                gamepad1.right_stick_x * 1.0
                        );

                    }   else   {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );
                    }


//                   else if (gamepad1.b) {
//                        // If the B button is pressed on gamepad1, we generate a splineTo()
//                        // trajectory on the fly and follow it
//                        // We switch the state to AUTOMATIC_CONTROL
//
//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .splineTo(targetAVector, targetAHeading)
//                                .build();
//
//                        drive.followTrajectoryAsync(traj1);
//
//                        currentMode = Mode.AUTOMATIC_CONTROL;
                   /* } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }/* else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;*/
//                case AUTOMATIC_CONTROL:
//                    // If x is pressed, we break out of the automatic following
//                    if (gamepad1.x) {
//                        drive.cancelFollowing();
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}