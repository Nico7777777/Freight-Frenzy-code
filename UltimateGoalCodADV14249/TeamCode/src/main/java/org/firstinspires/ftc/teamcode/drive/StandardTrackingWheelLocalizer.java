package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     __     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
//    public static double TICKS_PER_REV = 1440;
//    public static double WHEEL_RADIUS = 0.75 ; // in  1.4960/2
//    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
//
//    public static double LATERAL_DISTANCE = 13.7632405;    //13.38-acto; // in; distance between the left and right wheels  12.9498361
//    public static double FORWARD_OFFSET = 1.3781;       //3.74;-acto // in; offset of the lateral wheel
//
//    public static double X_MULTIPLIER = 0.98293799;
//    public static double Y_MULTIPLIER = 0.98416;

    //robot v.1.3

    //REV
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.98; //LADY 2
    public static double GEAR_RATIO = 1;

    public static double LATERAL_DISTANCE = 15.5894093;//LADY 16.1925238574218 *2; // Fan 17.5177964606;
    public static double FORWARD_OFFSET = 1.3781;

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER =1;


    private Encoder leftEncoder, rightEncoder, frontEncoder;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        //5843
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "wobble"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "shooter1"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));

        //LADY
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "wobble"));//intake
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "shooter1"));//wobble
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));//shooter1

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        // TODO: la rev de comentat
        //14249
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        frontEncoder.setDirection(Encoder.Direction.REVERSE);

        //LADY
//        frontEncoder.setDirection(Encoder.Direction.REVERSE);



    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method


//REV
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );

//        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getRawVelocity()) * X_MULTIPLIER,
//                encoderTicksToInches(rightEncoder.getRawVelocity()) * X_MULTIPLIER,
//                encoderTicksToInches(frontEncoder.getRawVelocity()) * Y_MULTIPLIER
//        );
        //  v1.3
    }
}