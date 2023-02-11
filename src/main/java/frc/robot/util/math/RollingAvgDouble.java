// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.math;

import java.util.ArrayList;

/** Add your docs here. */
public class RollingAvgDouble {
  ArrayList<Double> lastVals = new ArrayList<Double>();
  public int maxItems;

  public RollingAvgDouble(int maxItems) {
    this.maxItems = maxItems;
  }

  public RollingAvgDouble(double maxTime) {
    this.maxItems = (int) (50 * maxTime);
  }

  public void clearExtras() {
    while (this.lastVals.size() > this.maxItems) {
      this.lastVals.remove(0);
    }
  }

  public void add(double inputVal) {
    this.clearExtras();
    this.lastVals.add(inputVal);
  }

  public boolean full() {
    this.clearExtras();
    return (this.lastVals.size() == this.maxItems);
  }

  public void emptyValues() {
    this.lastVals.clear();
  }

  public double getAvg() {
    // ensures we only look at the last n samples, set via robot loop logic timing.
    this.clearExtras();
    while (this.lastVals.size() > this.maxItems) {
      this.lastVals.remove(0);
    }

    // very hacky toolkit to get totals, doubles are IEEE
    double total = 0;
    for (Double item : this.lastVals) {
      total += item;
    }
    return total / this.lastVals.size();
  }
  /* targetTolerance is the ±n of the goalTarget.

  This function returns whether the rolling average is on target.
  */
  public boolean withinTolerance(double targetTolerance, double goalTarget) {
    this.clearExtras();

    double currentAvg = getAvg();

    return (currentAvg > goalTarget - targetTolerance)
        && (currentAvg < goalTarget + targetTolerance);
  }

  /* This is to verify if all of the last values we saw within the target window are within tolerance */
  public boolean lastValuesWithinTolerance(double targetTolerance, double goalTarget) {
    this.clearExtras();

    for (double val : this.lastVals) {
      if (!((val > goalTarget - targetTolerance) && val < goalTarget + targetTolerance)) {
        return false;
      }
    }
    return true;
  }

  /* BE SURE TO EXPRESS AS PERCENT VALUE.

  THIS WOULD BE LIKE 5% = .05

  This might be used like the independent rollers to ensure they're at speed. */

  public boolean withinTolerancePercent(double targetTolerancePercent, double goalTarget) {
    double unitsTolerance = goalTarget * targetTolerancePercent;
    return withinTolerance(unitsTolerance, goalTarget);
  }
}