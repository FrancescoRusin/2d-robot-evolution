package io.github.ericmedvet.robotevo2d.main;

import io.github.ericmedvet.jgea.experimenter.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.jumping.Jumping;
import io.github.ericmedvet.mrsim2d.core.tasks.locomotion.Locomotion;

import java.io.*;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Supplier;
import java.util.stream.IntStream;

@SuppressWarnings({"unused", "unchecked"})
public class Main {
  protected static final NamedBuilder<Object> nb = PreparedNamedBuilder.get();
  protected static final Supplier<Engine> engine = () -> ServiceLoader.load(Engine.class).findFirst().orElseThrow();
  protected static final String robotBody = "sim.agent.vsr.gridBody(" +
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

  private enum Shape {BIPED, WORM, T}

  public static void main(String[] args) throws IOException {
    locomotionValidation(Shape.T, 12, "FL_T_locomotion.csv");
    jumpingValidation(Shape.T, 12,"FL_T_jumping.csv");
  }

  public static void locomotionValidation(Shape shape, int nOfShapes, String fileName) throws IOException {
    final Locomotion locomotion = (Locomotion) nb.build("sim.task.locomotion(duration = 10)");
    String actualRobotMapper = String.format(robotMapper, String.format(robotBody, "sim.agent.vsr.shape.free(s = \"%s\")"), sin);
    Future<Double> baseResult;
    List<List<Double>> genotypes;
    List<Future<Double>> results;
    final Random rg = new Random();
    final ExecutorService executorService = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    final FileWriter writer = new FileWriter(fileName);
    writer.write("rigids;point;segment;genotype;fitness\n");
    final double SEGMENTLENGTH = 0.5;
    final int NOFPOINTS = 20;
    final int NOFTRIALS = 20;
    final int FRAGMENTATIONS = 500;
    for (int rigids = 0; rigids < nOfShapes; ++rigids) {
      InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper =
              (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(String.format(actualRobotMapper, buildStringShape(shape, rigids)));
      for (int point = 0; point < NOFPOINTS; ++point) {
        List<Double> baseGenotype = IntStream.range(0, 20).mapToDouble(i -> rg.nextDouble(-1, 1)).boxed().toList();
        genotypes = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        results = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        baseResult = executorService.submit(() -> locomotion.run(mapper.apply(baseGenotype), engine.get()).firstAgentXVelocity());
        for (int trial = 0; trial < NOFTRIALS; ++trial) {
          List<Double> randomVector = IntStream.range(0, 20).mapToDouble(i -> rg.nextGaussian()).boxed().toList();
          double norm = Math.sqrt(randomVector.stream().mapToDouble(i -> Math.pow(i, 2)).sum());
          randomVector = randomVector.stream().mapToDouble(i -> SEGMENTLENGTH * i / norm).boxed().toList();
          for (int iter = 1; iter < FRAGMENTATIONS + 1; ++iter) {
            double tick = iter / (double) FRAGMENTATIONS;
            List<Double> placeholder1 = new ArrayList<>(randomVector);
            List<Double> placeholder2 = IntStream.range(0, 20).mapToDouble(i -> baseGenotype.get(i) + tick * placeholder1.get(i)).boxed().toList();
            genotypes.add(placeholder2);
            results.add(executorService.submit(() -> locomotion.run(mapper.apply(placeholder2), engine.get()).firstAgentXVelocity()));
          }
        }
        try {
          writer.write(String.format("%d;%d;%d;%s;", rigids, point, -1, serialize(baseGenotype)) + baseResult.get() + "\n");
          for (int counter = 0; counter < results.size(); ++counter) {
            writer.write(
                    String.format("%d;%d;%d;%s;", rigids, point, counter / FRAGMENTATIONS, serialize(genotypes.get(counter))) +
                            results.get(counter).get() + "\n");
          }
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
    writer.close();
    executorService.shutdown();
  }

  public static void jumpingValidation(Shape shape, int nOfShapes, String fileName) throws IOException {
    final Jumping jumping = (Jumping) nb.build("sim.task.jumping(duration = 10)");
    String actualRobotMapper = String.format(robotMapper, String.format(robotBody, "sim.agent.vsr.shape.free(s = \"%s\")"), sin);
    Future<Double> baseResult;
    List<List<Double>> genotypes;
    List<Future<Double>> results;
    final Random rg = new Random();
    final ExecutorService executorService = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    final FileWriter writer = new FileWriter(fileName);
    writer.write("rigids;point;segment;genotype;fitness\n");
    final double SEGMENTLENGTH = 0.5;
    final int NOFPOINTS = 20;
    final int NOFTRIALS = 20;
    final int FRAGMENTATIONS = 500;
    for (int rigids = 0; rigids < nOfShapes; ++rigids) {
      InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper =
              (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(String.format(actualRobotMapper, buildStringShape(shape, rigids)));
      for (int point = 0; point < NOFPOINTS; ++point) {
        List<Double> baseGenotype = IntStream.range(0, 20).mapToDouble(i -> rg.nextDouble(-1, 1)).boxed().toList();
        genotypes = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        results = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        baseResult = executorService.submit(() -> jumping.run(mapper.apply(baseGenotype), engine.get()).firstAgentXVelocity());
        for (int trial = 0; trial < NOFTRIALS; ++trial) {
          List<Double> randomVector = IntStream.range(0, 20).mapToDouble(i -> rg.nextGaussian()).boxed().toList();
          double norm = Math.sqrt(randomVector.stream().mapToDouble(i -> Math.pow(i, 2)).sum());
          randomVector = randomVector.stream().mapToDouble(i -> SEGMENTLENGTH * i / norm).boxed().toList();
          for (int iter = 1; iter < FRAGMENTATIONS + 1; ++iter) {
            double tick = iter / (double) FRAGMENTATIONS;
            List<Double> placeholder1 = new ArrayList<>(randomVector);
            List<Double> placeholder2 = IntStream.range(0, 20).mapToDouble(i -> baseGenotype.get(i) + tick * placeholder1.get(i)).boxed().toList();
            genotypes.add(placeholder2);
            results.add(executorService.submit(() -> jumping.run(mapper.apply(placeholder2), engine.get()).firstAgentMaxRelativeJumpHeight()));
          }
        }
        try {
          writer.write(String.format("%d;%d;%d;%s;", rigids, point, -1, serialize(baseGenotype)) + baseResult.get() + "\n");
          for (int counter = 0; counter < results.size(); ++counter) {
            writer.write(
                    String.format("%d;%d;%d;%s;", rigids, point, counter / FRAGMENTATIONS, serialize(genotypes.get(counter))) +
                            results.get(counter).get() + "\n");
          }
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
    writer.close();
    executorService.shutdown();
  }

  public static String buildStringShape(Shape shape, int nOfRigids) {
    StringBuilder rigidsString = new StringBuilder();
    if (shape == Shape.BIPED) {
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
    } else if (shape == Shape.WORM) {
      boolean[][] rigids = new boolean[5][2];
      for (int i = 0; i < 10; ++i) {
        rigids[i / 2][i % 2] = false;
      }
      for (int i = 0; i < nOfRigids && i < 5; ++i) {
        rigids[i][i % 2] = true;
      }
      if (nOfRigids > 5) {
        rigids[1][0] = true;
      }
      if (nOfRigids > 6) {
        rigids[2][1] = true;
      }
      for (int i = 0; i < 5; ++i) {
        rigidsString.append(rigids[i][0] ? "r" : "s");
      }
      rigidsString.append("-");
      for (int i = 0; i < 5; ++i) {
        rigidsString.append(rigids[i][1] ? "r" : "s");
      }
      return rigidsString.toString();
    } else if (shape == Shape.T) {
      boolean[][] rigids = new boolean[2][5];
      for (int i = 0; i < 10; ++i) {
        rigids[i / 5][i % 5] = false;
      }
      for (int i = 0; i < nOfRigids && i < 5; ++i) {
        rigids[i % 2][i] = true;
      }
      for (int i = 0; i < nOfRigids - 5 && i < 5; ++i) {
        rigids[1 - i % 2][i] = true;
      }
      for (int i = 0; i < 4; ++i) {
        rigidsString.append(String.format(".%s%s.-", rigids[0][i] ? "r" : "s", rigids[1][i] ? "r" : "s"));
      }
      rigidsString.append(String.format("%s%s%ss", nOfRigids < 11 ? "s" : "r", rigids[0][4] ? "r" : "s", rigids[1][4] ? "r" : "s"));
      return rigidsString.toString();
    }
    return null;
  }

  public static String serialize(List<Double> list) {
    try (ByteArrayOutputStream baos = new ByteArrayOutputStream(); ObjectOutputStream oos = new ObjectOutputStream(baos)) {
      oos.writeObject(list);
      oos.flush();
      return Base64.getEncoder().encodeToString(baos.toByteArray());
    } catch (Throwable t) {
      return null;
    }
  }

  private static Object deserialize(String serialized) {
    byte[] bytes = Base64.getDecoder().decode(serialized);
    try (ObjectInputStream oois = new ObjectInputStream(new ByteArrayInputStream(bytes))) {
      return oois.readObject();
    } catch (IOException | ClassNotFoundException e) {
      throw new RuntimeException(e);
    }
  }
}
