package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
double degreesToEncoderTicks(double angle) {
    double percentage = (angle / 360);
    return (percentage * 1 / ((15.0 / 32.0) * (10.0 / 60.0)) * 2048);
}   

    GenericEntry entry = Shuffleboard.getTab("Testing").add("value", 0).getEntry();
    GenericEntry entry5 = Shuffleboard.getTab("cancoder").add("cancoder5", 0).getEntry();
    GenericEntry entry3 = Shuffleboard.getTab("cancoder").add("cancoder3", 0).getEntry();
    GenericEntry entry2 = Shuffleboard.getTab("cancoder").add("cancoder2", 0).getEntry();
    GenericEntry entry4 = Shuffleboard.getTab("cancoder").add("cancoder4", 0).getEntry();

    TalonFX frontleft = new TalonFX(17);
    TalonFX frontright = new TalonFX(14);
    TalonFX backleft = new TalonFX(11);
    TalonFX backright = new TalonFX(15);

    TalonFX frontleft2 = new TalonFX(18);
    TalonFX frontright2 = new TalonFX(13);
    TalonFX backleft2 = new TalonFX(12);
    TalonFX backright2 = new TalonFX(16);

    CANCoder frontleft3 = new CANCoder(5);
    CANCoder frontright3 = new CANCoder(3);
    CANCoder backleft3 = new CANCoder(2);
    CANCoder backright3 = new CANCoder(4);

    double offset5 = 74.619140625;
    double offset4 = 103.447265625;
    double offset3 = 165.5859375;
    double offset2 = -76.552734375;

    XboxController joystick = new XboxController(0);

    double currentAngle = 0;

    public Motor() {

        initializeCANcoder(frontleft3);
        initializeCANcoder(frontright3);
        initializeCANcoder(backleft3);
        initializeCANcoder(backright3);

        double CANangle5 = frontleft3.getAbsolutePosition() - offset5;
        double CANangle3 = frontright3.getAbsolutePosition() - offset3;
        double CANangle2 = backleft3.getAbsolutePosition() - offset2;
        double CANangle4 = backright3.getAbsolutePosition() - offset4;

        frontleft2.setNeutralMode(NeutralMode.Brake);
        frontright2.setNeutralMode(NeutralMode.Brake);
        backleft2.setNeutralMode(NeutralMode.Brake);
        backright2.setNeutralMode(NeutralMode.Brake);
        

        frontleft2.config_kP(0, .1);
        frontleft2.config_kD(0, .2);
        frontleft2.config_kI(0, 0);
        frontleft2.setSelectedSensorPosition(degreesToEncoderTicks(CANangle5));


        frontright2.config_kP(0, .1);
        frontright2.config_kD(0, .2);
        frontright2.config_kI(0, 0);
        frontright2.setSelectedSensorPosition(degreesToEncoderTicks(CANangle3));

        backleft2.config_kP(0, .1);
        backleft2.config_kD(0, .2);
        backleft2.config_kI(0, 0);
        backleft2.setSelectedSensorPosition(degreesToEncoderTicks(CANangle2));

        backright2.config_kP(0, .1);
        backright2.config_kD(0, .2);
        backright2.config_kI(0, 0);
        backright2.setSelectedSensorPosition(degreesToEncoderTicks(CANangle4));

    }

    private void initializeCANcoder(CANCoder canCoder) {
        for(int i = 0; i<10; i++){
            ErrorCode code = canCoder.configAllSettings(new CANCoderConfiguration());
            if(code == ErrorCode.OK){
                break;
            }
            else {
                System.out.println(":( depression, code failed");
            }
        }
    }

    public double optimizeAngle(double current, double set) {
        int angleRevo = (int)(current / 360);

        current %= 360;
        set %= 360;

        double change = current - set;

        if (change > 270) {
            set += 360;
        } else if (change < -270) {
            set -= 360;
        }

        set += 360 * angleRevo;

        return set;
    }

    @Override
    public void periodic() {

        double CANangle5 = frontleft3.getAbsolutePosition() - offset5;
        double CANangle3 = frontright3.getAbsolutePosition() - offset3;
        double CANangle2 = backleft3.getAbsolutePosition() - offset2;
        double CANangle4 = backright3.getAbsolutePosition() - offset4;

        double joystickx = joystick.getRightX();
        double leftspeed = joystick.getLeftY();
        double joysticky = joystick.getRightY();

        if (joystickx < 0.05 && joystickx > -0.05) {
            joystickx = 0;
        }

        if (joysticky < 0.05 && joysticky > -0.05) {
            joysticky = 0;
        }

        double angle = Math.toDegrees(Math.atan2(joystickx, joysticky)) + 180;

        angle = optimizeAngle(currentAngle, angle);
        currentAngle = angle;
        // double percentage = (angle / 360);
         double gearatio = degreesToEncoderTicks(angle);
        // System.out.println(gearatio);

        // System.out.println(CANangle);

        frontleft.set(TalonFXControlMode.PercentOutput, leftspeed * -.25);
        frontright.set(TalonFXControlMode.PercentOutput, leftspeed * -.25);
        backleft.set(TalonFXControlMode.PercentOutput, leftspeed * -.25);
        backright.set(TalonFXControlMode.PercentOutput, leftspeed * -.25);
        frontleft2.set(TalonFXControlMode.Position, gearatio);
        frontright2.set(TalonFXControlMode.Position, gearatio);
        backleft2.set(TalonFXControlMode.Position, gearatio);
        backright2.set(TalonFXControlMode.Position, gearatio);

        entry5.setDouble(CANangle5);
        entry3.setDouble(CANangle3);
        entry2.setDouble(CANangle2);
        entry4.setDouble(CANangle4);
    }
}
