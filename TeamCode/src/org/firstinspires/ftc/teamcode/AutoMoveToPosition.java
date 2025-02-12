package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "AutoMoveToPosition")
public class AutoMoveToPosition extends LinearOpMode {

    public DcMotor motorRightFront = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorLeftFront = null;
    public SparkFunOTOS myOtos;
    public Position desiredPosition = new Position(DistanceUnit.INCH, 37.58, 37.53, 0, 0);
    public YawPitchRollAngles desiredOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0.74, 0, 0, 0);
    public Position desiredPosition2 = new Position(DistanceUnit.INCH, 43.02, -33.46, 0, 0);
    public YawPitchRollAngles desiredOrientation2 = new YawPitchRollAngles(AngleUnit.RADIANS, -1.07, 0, 0, 0);

    // Tuning variables for position control - go too fast with too tight of tolerances and things will get wacky
    final double[] Kp = {0.3, 0.3, 0.5};
    final double[] Ki = {0,0,0};//{0.0001, 0.0001, 0}; // 0.01;
    final double[] MaxIntegral = {1000, 1000, 10};
    final double[] Kd = {0.005, 0.005, 0}; // 0.005;
    public double MAX_AUTO_Y = 1.0;
    public double MAX_AUTO_X = 1.0;
    public double MAX_AUTO_R  = 1.0;
    final double Y_TOLERANCE = 0.5;
    final double X_TOLERANCE = 0.5;
    final double R_TOLERANCE = 0.25;
    // PID error terms
    double xIntegral = 0;
    double yIntegral = 0;
    double rIntegral = 0;
    double xPreviousError = 0;
    double yPreviousError = 0;
    double rPreviousError = 0;

    public int state = 0;
    public int nextState = 0;

    @Override
    public void runOpMode() {

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        motorLeftBack = hardwareMap.dcMotor.get("back_left_motor");
        motorLeftFront = hardwareMap.dcMotor.get("front_left_motor");
        motorRightFront = hardwareMap.dcMotor.get("front_right_motor");
        motorRightBack = hardwareMap.dcMotor.get("back_right_motor");

        // Calling setPower with a positive value should rotate the wheel forward
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        configureOtos();

        waitForStart();

        while (opModeIsActive()) {
            if (state==0){
                if (inPosition(desiredPosition, desiredOrientation)){
                    resetIntegrals();
                    nextState = 1;
                } else {
                    driveTowards(desiredPosition, desiredOrientation);
                }
            } else if (state==1){
                if (inPosition(desiredPosition2, desiredOrientation2)){
                    resetIntegrals();
                    nextState = 0;
                } else {
                    driveTowards(desiredPosition2, desiredOrientation2);
                }
            }

            state = nextState;

            telemetry.addLine("State: " + state);
            telemetry.update();
            sleep(20);  // Sleeping here allows the CPU to catch up with other tasks
        }
    }

    public boolean inPosition(Position desiredPosition, YawPitchRollAngles desiredOrientation){
        double  xError = desiredPosition.x - myOtos.getPosition().x;
        double  yError = desiredPosition.y - myOtos.getPosition().y;
        double  rError = desiredOrientation.getYaw(AngleUnit.RADIANS) - myOtos.getPosition().h;

        return (Math.abs(xError) < X_TOLERANCE) && (Math.abs(yError) < Y_TOLERANCE) && (Math.abs(rError) < R_TOLERANCE);
    }

    public void driveTowards(Position desiredPosition, YawPitchRollAngles desiredOrientation){
        double heading = myOtos.getPosition().h;

        double  xError = desiredPosition.x - myOtos.getPosition().x;
        double  yError = desiredPosition.y - myOtos.getPosition().y;
        double  rError = desiredOrientation.getYaw(AngleUnit.RADIANS) - heading;

        // Proportional term
        double xProportional = xError;
        double yProportional = yError;
        double rProportional = rError;

        // Integral term
        xIntegral += xError;
        yIntegral += yError;
        rIntegral += rError;
        xIntegral = Range.clip(xIntegral, -MaxIntegral[0], MaxIntegral[0]);
        yIntegral = Range.clip(yIntegral, -MaxIntegral[1], MaxIntegral[1]);
        rIntegral = Range.clip(rIntegral, -MaxIntegral[2], MaxIntegral[2]);

        // Derivative term
        double xDerivative = xError - xPreviousError;
        double yDerivative = yError - yPreviousError;
        double rDerivative = rError - rPreviousError;

        // PID output
        double xOutput = (Kp[0] * xProportional) + (Ki[0] * xIntegral) + (Kd[0] * xDerivative);
        double yOutput = (Kp[1] * yProportional) + (Ki[1] * yIntegral) + (Kd[1] * yDerivative);
        double rOutput = (Kp[2] * rProportional) + (Ki[2] * rIntegral) + (Kd[2] * rDerivative);

        // Update previous errors
        xPreviousError = xError;
        yPreviousError = yError;
        rPreviousError = rError;

        // Rotate the output values to account for the robot's heading
        double xRotated = xOutput * Math.cos(heading) - yOutput * Math.sin(heading);
        double yRotated = xOutput * Math.sin(heading) + yOutput * Math.cos(heading);

        // Clip the outputs to the maximum allowed values
        xRotated = Range.clip(xRotated, -MAX_AUTO_X, MAX_AUTO_X);
        yRotated = Range.clip(yRotated, -MAX_AUTO_Y, MAX_AUTO_Y);
        rOutput = Range.clip(rOutput, -MAX_AUTO_R, MAX_AUTO_R);

        if (Math.abs(rError) < R_TOLERANCE) {
            moveRobot(xRotated, yRotated, 0);
        } else {
            moveRobot(0, 0, rOutput);
        }

        telemetry.addLine("xProportional: " + xProportional);
        telemetry.addLine("xIntegral: " + xIntegral);
        telemetry.addLine("xDerivative: " + xDerivative);
        telemetry.addLine();
        telemetry.addLine("yProportional: " + yProportional);
        telemetry.addLine("yIntegral: " + yIntegral);
        telemetry.addLine("yDerivative: " + yDerivative);
        telemetry.addLine();
        telemetry.addLine("rProportional: " + rProportional);
        telemetry.addLine("rIntegral: " + rIntegral);
        telemetry.addLine("rDerivative: " + rDerivative);
        telemetry.addLine();
    }

    public void resetIntegrals(){
        xIntegral = 0;
        yIntegral = 0;
        rIntegral = 0;
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motorLeftFront.setPower(leftFrontPower);
        motorRightFront.setPower(rightFrontPower);
        motorLeftBack.setPower(leftBackPower);
        motorRightBack.setPower(rightBackPower);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 2, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.update();
    }

    private void stopMotors() {
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
    }
}
