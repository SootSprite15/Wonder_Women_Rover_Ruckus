package org.firstinspires.ftc.teamcode;



public class imuAngle {

    private double angle; //angle must always be between -180 and 180

    public imuAngle(double input){
        angle = (input > 180) ? input - 360 : input;
        angle = (angle <= -180) ? angle + 360 : angle;

    }

    public static imuAngle add(imuAngle angle1, imuAngle angle2){
            double result = angle1.angle + angle2.angle;
           return new imuAngle(result);
    }

    public static imuAngle subtract(imuAngle angle1, imuAngle angle2){
        double result = angle1.angle - angle2.angle;
        return new imuAngle(result);
    }

    public static double distance(imuAngle angle1, imuAngle angle2){
        imuAngle result = subtract(angle1, angle2);
        double dist = Math.abs(result.angle);
        return dist;
    }

    public static double toDouble(imuAngle angle1){
        return(angle1.angle);
    }

}
