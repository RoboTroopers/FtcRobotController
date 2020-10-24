package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.MathMethods;
import org.firstinspires.ftc.teamcode.util.UtilMethods;

import static java.lang.Math.abs;
import static java.lang.Math.min;

public class OpmodeBase extends OpMode
{
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;

    public BNO055IMU imu;

    public DcMotor horizontalEncoder;
    public DcMotor verticalEncoder;
    public DcMotor verticalEncoder2;

    private double worldXPos = 0;
    private double worldYPos = 0;

    private double horizontalEncoderLast = 0;
    private double verticalEncoderLast = 0;


    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        horizontalEncoder = hardwareMap.dcMotor.get("horizontalEncoder");
        verticalEncoder = hardwareMap.dcMotor.get("verticalEncoder1");
        verticalEncoder2 = hardwareMap.dcMotor.get("verticalEncoder2");
    }

    public double getWorldAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        // Since the imu is rotated on its x-axis 90 degrees, the x-axis is the one that points up.
        return Math.toRadians(angles.firstAngle);
    }

    public double getWorldXPos()
    {
        return worldXPos;
    }

    public double getWorldYPos()
    {
        return worldYPos;
    }

    public void updatePos()
    {
        double horizontalReading = horizontalEncoder.getCurrentPosition();
        double verticalReading = (verticalEncoder.getCurrentPosition() - verticalEncoder2.getCurrentPosition())/2.0;
        double angleReading = getWorldAngle();

        double horizontalChange = horizontalReading - horizontalEncoderLast;
        double verticalChange =  verticalReading   - verticalEncoderLast;

        // Position where the robot would be if the robot had not strafed
        double forwardShiftX = verticalChange * Math.cos(angleReading);
        double forwardShiftY = verticalChange * Math.sin(angleReading);

        // How far the robot's position has shifted as a result of strafing
        double strafeShiftX = horizontalChange * Math.cos(angleReading);
        double strafeShiftY = horizontalChange * Math.sin(angleReading);

        worldXPos += forwardShiftX + strafeShiftX;
        worldYPos += forwardShiftY + strafeShiftY;

        horizontalEncoderLast = horizontalReading;
        verticalEncoderLast = verticalReading;

        telemetry.addData("world x:", worldXPos);
        telemetry.addData("world y:", worldYPos);
    }


    // Set powers of motors according to direction parameters.
    public void applyMovement(double horizontal, double vertical, double turn)
    {
        // Sideways multiplied by 1.5 because mechanum drive strafes sideways slower than forwards/backwards
        double frontLeftRaw =  vertical + turn - (horizontal*1.5);
        double frontRightRaw = vertical - turn + (horizontal*1.5);
        double rearLeftRaw =   vertical + turn + (horizontal*1.5);
        double rearRightRaw =  vertical - turn - (horizontal*1.5);

        // Find greatest power
        double maxRawPower = Math.max(Math.max(frontLeftRaw, frontRightRaw), Math.max(rearLeftRaw, rearRightRaw));

        double scaleDownFactor = 1.0;
        if (maxRawPower > 1.0)
        {
            // Reciprocal of maxRawPower so that when multiplied by factor, maxPower == 1 (full speed)
            scaleDownFactor = 1.0/maxRawPower;
        }

        // All motor speeds scaled down (if maxRawPower > 1) but vector is preserved.
        frontLeftRaw *= scaleDownFactor;
        frontRightRaw *= scaleDownFactor;
        rearLeftRaw *= scaleDownFactor;
        rearRightRaw *= scaleDownFactor;

        frontLeft.setPower(frontLeftRaw);
        frontRight.setPower(frontRightRaw);
        rearLeft.setPower(rearLeftRaw);
        rearRight.setPower(rearRightRaw);
    }


    double clampPower(double power, double minSpeed, double maxSpeed)
    {
        if (power > 0)
        {
            power = MathMethods.clamp(power, minSpeed, maxSpeed);
        }
        else if (power < 0)
        {
            power = MathMethods.clamp(power, -maxSpeed, -minSpeed);
        }
        return power;
    }
    
    
    public void glideToPos(double xPos, double yPos, double xSpeed, double ySpeed, double preferredAngle, double turnSpeed)
    {
        double absoluteDist = Math.hypot(xPos - worldXPos, yPos - worldYPos);
        double absoluteAngle = Math.atan2(yPos - worldYPos, xPos - worldXPos);
        double relativeAngle = MathMethods.angleWrap(absoluteAngle - (getWorldAngle() - Math.toRadians(90)));

        double relativeX = Math.cos(relativeAngle) * absoluteDist;
        double relativeY = Math.sin(relativeAngle) * absoluteDist;

        double denominator =  Math.abs(relativeX) + Math.abs(relativeY);

        double xPower = relativeX / denominator;
        double yPower = relativeY / denominator;

        double turnPower = relativeAngle - Math.toRadians(180) + preferredAngle;
        turnPower = MathMethods.clamp(turnPower * 2, -1, 1);

        final double turnSlowThreshold = 10;
        if (absoluteDist < turnSlowThreshold)
        {
            turnPower *= absoluteDist / turnSlowThreshold;
        }
        turnPower = clampPower(turnPower, 0.1, turnSpeed);

        applyMovement(xPower * xSpeed, yPower * ySpeed, turnPower * turnSpeed);
    }
    
    
    public void glideRelative(double xDist, double yDist, double xSpeed, double ySpeed, double turnAngle, double turnSpeed)
    {
        final double minSpeed = 0.05;
        final double minTurnSpeed = 0.05;
        
        double xError = xDist - getWorldXPos();
        double yError = yDist - getWorldYPos();
        double turnError = MathMethods.angleWrap(turnAngle - getWorldAngle());

        if (abs(turnError) > 180)
        {
            turnError = 360 - turnError;
        }
        
        double xPower = clampPower(xError, minSpeed, xSpeed);
        double yPower = clampPower(yError, minSpeed, ySpeed);
        double turnPower = clampPower(turnError, minTurnSpeed, turnSpeed);

        applyMovement(xPower * xSpeed, yPower * ySpeed, turnPower * turnSpeed);
    }

    @Override
    public void loop()
    {
        updatePos();
        UtilMethods.delay(15);
    }
}
