// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Arm;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.module.SimpleModule;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.node.IntNode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.io.File;
import java.io.IOException;
import java.util.Map;

/** Represents all of the arm config data shared between the robot code and solver. */
public class ArmConfig {

    private Translation2d origin;
    private JointConfig shoulder;
    private JointConfig elbow;
    private SolverConfig solver;
    private Map<String, Constraint> constraints;

    public ArmConfig(
      Translation2d origin,
      JointConfig shoulder,
      JointConfig elbow,
      SolverConfig solver,
      Map<String, Constraint> constraints) {

      this.origin = origin;
      this.shoulder = shoulder;
      this.elbow = elbow;
      this.solver = solver;
      this.constraints = constraints;
    }

    public Translation2d origin() {
      return this.origin;
    }

    public JointConfig shoulder() {
      return this.shoulder;
    }

    public JointConfig elbow() {
      return this.elbow;
    }
  
    public SolverConfig solver() {
      return this.solver;
    }

    public Map<String, Constraint> constraints() {
      return constraints;
    }
    

  /** Physics constants for a single joint. */
  public static class JointConfig{
    private double mass;
    private double length;
    private double moi;
    private double cgRadius;
    private double minAngle;
    private double maxAngle;
    private double reduction;
    private DCMotor motor;

    public JointConfig(
      double mass,
      double length,
      double moi,
      double cgRadius,
      double minAngle,
      double maxAngle,
      double reduction,
      DCMotor motor
    ) {

        this.mass = mass;
        this.length = length;
        this.moi = moi;
        this.cgRadius = cgRadius;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.reduction = reduction;
        this.motor = motor;
      }

      public double mass() {
        return this.mass;
      }

      public double length() {
        return this.length;
      }

      public double moi() {
        return this.moi;
      }

      public double cgRadius() {
        return this.cgRadius;
      }

      public double minAngle() {
        return this.minAngle;
      }

      public double maxAngle() {
        return this.maxAngle;
      }

      public double reduction() {
        return this.reduction;
      }
      
      public DCMotor motor() {
        return this.motor;
      }
  }

  /** Config fields for solver. */
  public static class SolverConfig {
    private int interiorPoints;
    private double maxVoltage;

    public SolverConfig(
      int interiorPoints,
      double maxVoltage
    ) {

        this.interiorPoints = interiorPoints;
        this.maxVoltage = maxVoltage;
    }

    public int interiorPoints() {
      return this.interiorPoints;
    }

    public double maxVoltage() {
      return this.maxVoltage;
    }
  }

  /** Arbitrary solver constraint. */
  public static class Constraint {
    private String type;
    private double[] args;

    public Constraint(
      String type,
      double[] args
    ) {
      this.type = type;
      this.args = args;
    }

    public String type() {
      return type;
    }

    public double[] args() {
      return args;
    }

  }

  /** Converts double array to Translation2d instance. */
  private static class Translation2dDeserializer extends StdDeserializer<Translation2d> {
    public Translation2dDeserializer() {
      this(null);
    }

    public Translation2dDeserializer(Class<?> vc) {
      super(vc);
    }

    @Override
    public Translation2d deserialize(JsonParser jp, DeserializationContext ctxt)
        throws IOException, JsonProcessingException {
      ArrayNode node = jp.getCodec().readTree(jp);
      double x = (Double) node.get(0).numberValue();
      double y = (Double) node.get(1).numberValue();
      return new Translation2d(x, y);
    }
  }

  /** Converts motor type, count, and reduction to DCMotor instance. */
  private static class DCMotorDeserializer extends StdDeserializer<DCMotor> {
    public DCMotorDeserializer() {
      this(null);
    }

    public DCMotorDeserializer(Class<?> vc) {
      super(vc);
    }

    @Override
    public DCMotor deserialize(JsonParser jp, DeserializationContext ctxt)
        throws IOException, JsonProcessingException {
      JsonNode node = jp.getCodec().readTree(jp);
      String type = node.get("type").asText();
      int count = (Integer) ((IntNode) node.get("count")).numberValue();
      double reduction = (Double) ((DoubleNode) node.get("reduction")).numberValue();

      switch (type) {
        case "neo":
          return DCMotor.getNEO(count).withReduction(reduction);
        case "neo550":
          return DCMotor.getNeo550(count).withReduction(reduction);
        case "falcon500":
          return DCMotor.getFalcon500(count).withReduction(reduction);
        default:
          return null;
      }
    }
  }

  /** Generates a config instance by reading from a JSON file. */
  public static ArmConfig loadJson(File source) {
    // Set up object mapper
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    SimpleModule module = new SimpleModule();
    module.addDeserializer(Translation2d.class, new Translation2dDeserializer());
    module.addDeserializer(DCMotor.class, new DCMotorDeserializer());
    mapper.registerModule(module);

    // Read config data
    ArmConfig config;
    try {
      config = mapper.readValue(source, ArmConfig.class);
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse arm config JSON");
    }

    // Return result
    return config;
  }
}
