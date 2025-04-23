/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.seattlesolvers.solverslib.util;

import java.math.BigDecimal;
import java.math.RoundingMode;

public final class MathUtils {
    private MathUtils() {
        throw new AssertionError("utility class");
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     */
    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns value rounded to specified places
     *
     * @param number Number to round
     * @param places The number of decimal places to round to
     */
    public static double round(double number, int places) {
        return new BigDecimal((String.valueOf(number))).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    /**
     *
     * @param angleRadians The angle the be normalized (in radians)
     * @return Returns the angle passed as the parameter normalized between -pi and pi
     */
    public static double normalizeRadians(double angleRadians) {
        double angle = angleRadians % (2 * Math.PI);
        if (angle < 0) {
            return angle + 2 * Math.PI;
        }
        return angle;
    }
}