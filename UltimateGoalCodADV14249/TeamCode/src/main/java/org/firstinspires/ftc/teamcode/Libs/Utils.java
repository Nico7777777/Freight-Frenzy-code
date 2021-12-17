package org.firstinspires.ftc.teamcode.Libs;

/**
 * Created by edidi on 14.10.2016.
 */
public class Utils{

    // Cut method truncates a value between min_value and max_value limits
    public static Double cut(Double value, Double min_value, Double max_value){
        if(min_value > max_value){
            Double something = min_value;
            min_value = max_value;
            max_value = something;
        }

        if(value < min_value)
            value = min_value;
        else if(value > max_value)
            value = max_value;

        return value;
    }

    // This function scales a value between min_value and max_value to another value between scale_min and scale_max

    public static Double scale(Double value, Double min_value, Double max_value, Double scale_min, Double scale_max){
        /*if(min_value > max_value){
            Double something = min_value;
            min_value = max_value;
            max_value = something;
        }*/

        return (scale_min*(value - max_value) + scale_max*(min_value - value)) / (min_value - max_value);
    }

    // Same as scale except that the returned value will not exceed min_value or max_value

    public static Double range(Double value, Double min_value, Double max_value, Double range_min, Double range_max){
        Double scale = scale(value, min_value, max_value, range_min, range_max);
        Double range = cut(scale, min_value, max_value);

        return range;
    }

}
