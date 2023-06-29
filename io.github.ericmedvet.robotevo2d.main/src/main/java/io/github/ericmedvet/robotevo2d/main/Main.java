package io.github.ericmedvet.robotevo2d.main;

import io.github.ericmedvet.jgea.experimenter.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.tasks.locomotion.Locomotion;

import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

@SuppressWarnings({"unused", "unchecked"})
public class Main {
  protected static final NamedBuilder<Object> nb = PreparedNamedBuilder.get();
  protected static final String bipedBody = "sim.agent.vsr.gridBody(" +
          "shape = %s;" +
          "sensorizingFunction = sim.agent.vsr.sensorizingFunction.empty()" +
          ")";
  protected static final String robotMapper = "er.mapper.numericalParametrizedHomoBrains(" +
          "target = sim.agent.centralizedNumGridVSR(" +
          "body = %s;" +
          "function = %s" +
          ")" +
          ")";
  protected static final String sin = "dynamicalSystem.numerical.sin(" +
          "p = s.range(min = -1.57; max = 1.57);" +
          "f = s.range(min = 0.3; max = 2);" +
          "a = s.range(min = 1; max = 1);" +
          "b = s.range(min = 1; max = 1)" +
          ")";

  public static void main(String[] args) {
    VSRValidation();
  }

  public static void VSRValidation() {
    final Locomotion locomotion = (Locomotion) nb.build("sim.task.locomotion(duration = 10)");
    InvertibleMapper<List<Double>, EmbodiedAgent> mapper;
    String actualRobotMapper = String.format(robotMapper, String.format(bipedBody, "sim.agent.vsr.shape.free(s = %s)"), sin);
    final Random rg = new Random();
    for (int i = 0; i < 10; ++i) {
      mapper = (InvertibleMapper<List<Double>, EmbodiedAgent>) nb.build(String.format(actualRobotMapper, buildStringShape(i)));
    }
  }

  public static String buildStringShape(int nOfRigids) {
    boolean[][] rigids = new boolean[4][3];
    for (int i = 0; i < 12; ++i) {
      rigids[i / 3][i % 3] = false;
    }
    for (int i = 0; i < nOfRigids && i < 6; ++i) {
      rigids[3 * (i % 2)][2 - i / 2] = true;
    }
    for (int i = 0; i < nOfRigids - 6 && i < 4; ++i) {
      rigids[1 + (i % 2)][i / 2] = true;
    }
    StringBuilder rigidsString = new StringBuilder();
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 4; ++j) {
        rigidsString.append(rigids[j][i] ? "r" : "s");
      }
      rigidsString.append("-");
    }
    rigidsString.append(rigids[0][2] ? "r" : "s");
    rigidsString.append(".".repeat(2));
    rigidsString.append(rigids[3][2] ? "r" : "s");
    return rigidsString.toString();
  }
}
