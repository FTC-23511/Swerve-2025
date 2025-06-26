package com.seattlesolvers.solverslib.drivebase.swerve.coaxial;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.drivebase.RobotDrive;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

public class CoaxialSwerveDrivetrain extends RobotDrive {
    private final CoaxialSwerveModule[] modules = new CoaxialSwerveModule[4];
    // Left-right c2c distance between pods
    private final double trackWidth;
    // Front-back c2c distance between pods
    private final double wheelBase;
    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    /**
     * The constructor for a standard coaxial swerve drivetrain in FTC, that has 4 motors and 4 Axon servos with their absolute encoders.
     *
     * @param trackWidth left-right center-to-center distance between pods
     * @param wheelBase front-back center-to-center distance between pods
     * @param maxSpeed the max speed of the robot in inches/second
     * @param swervoPIDFCoefficients the coefficients in order (P, I, D, F) for the swervo PIDF controller
     * @param motors the motors corresponding with the individual modules, starting from front-right going counterclockwise
     * @param swervos the swervos corresponding with the individual modules, starting from front-right going counterclockwise, and properly constructed with absolute encoders and their offsets so that the wheel/pod moves the robot forward with a positive motor power when the returned angle is 0
     */
    public CoaxialSwerveDrivetrain(double trackWidth, double wheelBase, double maxSpeed, PIDFCoefficients swervoPIDFCoefficients, MotorEx[] motors, CRServoEx[] swervos) {
        setMaxSpeed(maxSpeed);
        if (motors.length != 4 || swervos.length != 4) {
            throw new IllegalArgumentException("Hardware lists for swerve modules must have exactly 4 objects each");
        }
        if (trackWidth <= 0 || wheelBase <= 0 || maxSpeed <= 0) {
            throw new IllegalArgumentException("trackWidth, wheelBase, and maxSpeed must have positive values");
        }

        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;

        for (Motor motor : motors) {
            motor.setRunMode(Motor.RunMode.RawPower);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        this.modules[0] = new CoaxialSwerveModule(motors[0], swervos[0], new Vector2d(trackWidth/2, wheelBase/2), maxSpeed, swervoPIDFCoefficients);
        this.modules[1] = new CoaxialSwerveModule(motors[1], swervos[1], new Vector2d(-trackWidth/2, wheelBase/2), maxSpeed, swervoPIDFCoefficients);
        this.modules[2] = new CoaxialSwerveModule(motors[2], swervos[2], new Vector2d(-trackWidth/2, -wheelBase/2), maxSpeed, swervoPIDFCoefficients);
        this.modules[3] = new CoaxialSwerveModule(motors[3], swervos[3], new Vector2d(trackWidth/2, -wheelBase/2), maxSpeed, swervoPIDFCoefficients);
    }

    public CoaxialSwerveDrivetrain setCachingTolerance(double motorCachingTolerance, double swervoCachingTolerance) {
        for (CoaxialSwerveModule module : modules) {
            module.setCachingTolerance(motorCachingTolerance, swervoCachingTolerance);
        }
        return this;
    }

    public void setTargetVelocity(ChassisSpeeds targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void update() {
        Vector2d[] moduleVelocities = new Vector2d[modules.length];
        // Copy of the module velocities but with just magnitudes, to be normalized with the normalize method
        double[] moduleVelocitiesMagnitude = new double[moduleVelocities.length];

        for (int i = 0; i < modules.length; i++) {
            moduleVelocities[i] = modules[i].calculateVectorRobotCentric(targetVelocity);
            moduleVelocitiesMagnitude[i] = moduleVelocities[i].magnitude();
        }

        normalize(moduleVelocitiesMagnitude);

        for (int i = 0; i < modules.length; i++) {
            // Scale the actual module velocities by the scale the normalization did to the magnitudes
            moduleVelocities[i].scale(moduleVelocitiesMagnitude[i] / moduleVelocities[i].magnitude());
            // Update the module itself
            modules[i].updateModuleWithVelocity(moduleVelocities[i]);
        }
    }

    public void updateWithTargetVelocity(ChassisSpeeds targetVelocity) {
        setTargetVelocity(targetVelocity);
        update();
    }

    @Override
    public void stop() {
        for (CoaxialSwerveModule module : modules) {
            module.stop();
        }
    }
}
