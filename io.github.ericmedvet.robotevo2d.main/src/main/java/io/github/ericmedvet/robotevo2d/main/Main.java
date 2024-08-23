package io.github.ericmedvet.robotevo2d.main;

import io.github.ericmedvet.jgea.experimenter.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.Outcome;
import io.github.ericmedvet.mrsim2d.core.tasks.Task;
import io.github.ericmedvet.mrsim2d.core.tasks.jumping.Jumping;
import io.github.ericmedvet.mrsim2d.core.tasks.locomotion.Locomotion;
import io.github.ericmedvet.mrsim2d.viewer.Drawer;
import io.github.ericmedvet.mrsim2d.viewer.FramesImageBuilder;
import io.github.ericmedvet.robotevo2d.main.builders.Misc;

import javax.imageio.ImageIO;
import java.io.*;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.IntStream;

@SuppressWarnings({"unused", "unchecked"})
public class Main {
  protected static final NamedBuilder<Object> nb = PreparedNamedBuilder.get();
  protected static final Supplier<Engine> engine = () -> ServiceLoader.load(Engine.class).findFirst().orElseThrow();
  protected static final String gridBody = "sim.agent.vsr.gridBody(" +
          "shape = %s;" +
          "sensorizingFunction = %s" +
          ")";
  protected static final String robotMapper = "er.mapper.numericalParametrizedHomoBrains(" +
          "target = sim.agent.centralizedNumGridVSR(" +
          "body = %s;" +
          "function = %s" +
          ")" +
          ")";

  protected static final String noSensors = "sim.agent.vsr.sensorizingFunction.empty()";

  protected static final String standardSensors = "sim.agent.vsr.sensorizingFunction.directional(" +
          "nSensors = [sim.sensor.ar(); sim.sensor.rv(a = 0.0); sim.sensor.rv(a = 90.0)];" +
          "eSensors = [sim.sensor.ar(); sim.sensor.rv(a = 0.0); sim.sensor.rv(a = 90.0)];" +
          "sSensors = [sim.sensor.ar(); sim.sensor.rv(a = 0.0); sim.sensor.rv(a = 90.0); sim.sensor.c()];" +
          "wSensors = [sim.sensor.ar(); sim.sensor.rv(a = 0.0); sim.sensor.rv(a = 90.0)];" +
          "headSensors = []" +
          ")";
  protected static final String sin = "dynamicalSystem.numerical.sin(" +
          "p = s.range(min = -1.57; max = 1.57);" +
          "f = s.range(min = 0; max = 2);" +
          "a = s.range(min = 1; max = 1);" +
          "b = s.range(min = 1; max = 1)" +
          ")";

  protected static final String mlp = "dynamicalSystem.numerical.outStepped(" +
          "stepT = 0.2;" +
          "inner = dynamicalSystem.numerical.mlp(" +
          "innerLayerRatio = %s" +
          ")" +
          ")";

  protected static final Locomotion locomotion = (Locomotion) nb.build("sim.task.locomotion(duration = 10)");

  protected static final Jumping jumping = (Jumping) nb.build("sim.task.jumping(duration = 10)");

  private enum Shape {BIPED, WORM, T, PLUS}

  public static void main(String[] args) throws IOException {
    int NOFRIGIDS;
    List<Shape> shapes;
    for (String arg : args) {
      String[] argSplit = arg.split("-");
      if (argSplit.length > 2) {
        shapes = new ArrayList<>();
        for (int i = 2; i < argSplit.length; ++i) {
          shapes.add(switch (argSplit[2]) {
            case "biped" -> Shape.BIPED;
            case "worm" -> Shape.WORM;
            case "t" -> Shape.T;
            case "plus" -> Shape.PLUS;
            default -> null;
          });
        }
      } else {
        shapes = Arrays.stream(Shape.values()).toList();
      }
      for (Shape shape : shapes) {
        NOFRIGIDS = switch (shape) {
          case BIPED, WORM -> 7;
          case T -> 9;
          case PLUS -> 17;
        };
        switch (argSplit[1]) {
          case "loc":
            System.out.printf("Executing %s locomotion experiment for %s\n", argSplit[0], shape.name());
            if (argSplit[0].equals("sin")) {
              sinControllerLandscape(locomotion, Outcome::firstAgentXVelocity, shape, NOFRIGIDS, noSensors,
                      String.format("FL_%s_controller_sin_locomotion_refactored.csv", shape.name().toLowerCase()), false);
            } else if (argSplit[0].equals("mlp")) {
              mlpControllerLandscape(locomotion, Outcome::firstAgentXVelocity, shape, NOFRIGIDS, standardSensors,
                      String.format("FL_%s_controller_mlp_locomotion_refactored.csv", shape.name().toLowerCase()), false);
            }
            break;
          case "jmp":
            System.out.printf("Executing %s jumping experiment for %s\n", argSplit[0], shape.name());
            if (argSplit[0].equals("sin")) {
              sinControllerLandscape(jumping, Outcome::firstAgentMaxY, shape, NOFRIGIDS, noSensors,
                      String.format("FL_%s_controller_sin_jumping_refactored.csv", shape.name().toLowerCase()), false);
            } else if (argSplit[0].equals("mlp")) {
              mlpControllerLandscape(jumping, Outcome::firstAgentMaxY, shape, NOFRIGIDS, standardSensors,
                      String.format("FL_%s_controller_mlp_jumping_refactored.csv", shape.name().toLowerCase()), false);
            }
            break;
        }
      }
    }
  }

  public static void robotsDraw() throws IOException {
    List<InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>> mappers = new ArrayList<>();
    for (int i = 0; i < 10; ++i) {
      mappers.add((InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(String.format("er.mapper.numericalParametrizedHomoBrains(target = sim.agent.centralizedNumGridVSR(body = sim.agent.vsr.gridBody(shape = sim.agent.vsr.shape.free(s = \"%s\"); sensorizingFunction = sim.agent.vsr.sensorizingFunction.empty()); function = %s))",
              buildStringShape(Shape.T, i), sin)));
    }
    List<Double> genotype = new ArrayList<>();
    for (int i = 0; i < 24; ++i) {
      genotype.add(-1d);
    }
    FramesImageBuilder drawer;
    for (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper : mappers) {
      drawer = new FramesImageBuilder(
              Misc.FILE_VIDEO_W, Misc.FILE_VIDEO_H, 1, 2, .1, FramesImageBuilder.Direction.HORIZONTAL, true,
              ((Function<String, Drawer>) nb.build("sim.drawer(miniWorld = false; miniAgents = NONE; engineProfiling = false; actions = false; nfc = false)")).apply(""));
      final int parameters = mapper.exampleInput().size();
      locomotion.run(mapper.apply(genotype), engine.get(), drawer);
      ImageIO.write(drawer.get(), "png", new File("t_%d_rigids.png".formatted(mappers.indexOf(mapper))));
    }
  }

  public static void sinControllerLandscape(Task<Supplier<EmbodiedAgent>, Outcome> task, Function<Outcome, Double> fitness, Shape shape, int maxNOfRigids, String sensors,
                                            String fileName, boolean saveG) throws IOException {
    String actualRobotMapper = String.format(robotMapper, String.format(gridBody, "sim.agent.vsr.shape.free(s = \"%s\")", sensors), sin);
    Future<Double> baseResult;
    List<double[]> genotypes;
    List<Future<Double>> results;
    final Random rg = new Random();
    final ExecutorService executorService = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    final FileWriter writer = new FileWriter(fileName);
    if (saveG) {
      writer.write("rigids;point;segment;genotype;fitness\n");
    } else {
      writer.write("rigids;point;segment;fitness\n");
    }
    final double SEGMENTLENGTH = 0.5;
    final int NOFPOINTS = 20;
    final int NOFTRIALS = 20;
    final int FRAGMENTATIONS = 500;
    for (int rigids = 0; rigids <= maxNOfRigids; ++rigids) {
      String stringShape = buildStringShape(shape, rigids);
      InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper =
              (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(String.format(actualRobotMapper, buildStringShape(shape, rigids)));
      int NOFPARAMS = mapper.exampleInput().size() - 2 * rigids;
      for (int point = 0; point < NOFPOINTS; ++point) {
        double[] baseGenotype = IntStream.range(0, NOFPARAMS).mapToDouble(i -> rg.nextDouble(-1, 1)).toArray();
        genotypes = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        results = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        baseResult = executorService.submit(() ->
                fitness.apply(task.run(mapper.apply(Arrays.stream(sinCAdjustParams(baseGenotype, stringShape)).boxed().toList()), engine.get())));
        for (int trial = 0; trial < NOFTRIALS; ++trial) {
          double[] randomVector = IntStream.range(0, NOFPARAMS).mapToDouble(i -> rg.nextGaussian()).toArray();
          double norm = Math.sqrt(Arrays.stream(randomVector).boxed().mapToDouble(i -> Math.pow(i, 2)).sum());
          for (int i = 0; i < randomVector.length; ++i) {
            randomVector[i] *= (SEGMENTLENGTH / norm);
          }
          for (int iter = 1; iter < FRAGMENTATIONS + 1; ++iter) {
            double tick = iter / (double) FRAGMENTATIONS;
            double[] placeholder = new double[randomVector.length];
            for (int i = 0; i < randomVector.length; ++i) {
              placeholder[i] = baseGenotype[i] + tick * randomVector[i];
            }
            genotypes.add(placeholder);
            results.add(executorService.submit(
                    () -> fitness.apply(task.run(mapper.apply(Arrays.stream(sinCAdjustParams(placeholder, stringShape)).boxed().toList()), engine.get()))));
          }
        }
        try {
          if (saveG) {
            writer.write(String.format("%d;%d;%d;%s;", rigids, point, -1, serialize(Arrays.stream(baseGenotype).boxed().toList())) + baseResult.get() + "\n");
            for (int counter = 0; counter < results.size(); ++counter) {
              writer.write(
                      String.format("%d;%d;%d;%s;", rigids, point, counter / FRAGMENTATIONS, serialize(Arrays.stream(genotypes.get(counter)).boxed().toList())) +
                              results.get(counter).get() + "\n");
            }
          } else {
            writer.write(String.format("%d;%d;%d;", rigids, point, -1) + baseResult.get() + "\n");
            for (int counter = 0; counter < results.size(); ++counter) {
              writer.write(String.format("%d;%d;%d;", rigids, point, counter / FRAGMENTATIONS) + results.get(counter).get() + "\n");
            }
          }
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
    writer.close();
    executorService.shutdown();
  }

  public static void mlpControllerLandscape(Task<Supplier<EmbodiedAgent>, Outcome> task, Function<Outcome, Double> fitness, Shape shape, int maxNOfRigids, String sensors,
                                            String fileName, boolean saveG) throws IOException {
    String actualRobotMapper = String.format(robotMapper, String.format(gridBody, "sim.agent.vsr.shape.free(s = \"%s\")", sensors), String.format(mlp, 1));
    Future<Double> baseResult;
    List<double[]> genotypes;
    List<Future<Double>> results;
    final Random rg = new Random();
    final ExecutorService executorService = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    final FileWriter writer = new FileWriter(fileName);
    if (saveG) {
      writer.write("rigids;point;segment;genotype;fitness\n");
    } else {
      writer.write("rigids;point;segment;fitness\n");
    }
    final double SEGMENTLENGTH = 0.5;
    final int NOFPOINTS = 20;
    final int NOFTRIALS = 20;
    final int FRAGMENTATIONS = 500;
    for (int rigids = 0; rigids <= maxNOfRigids; ++rigids) {
      String stringShape = buildStringShape(shape, rigids);
      InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper =
              (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(String.format(actualRobotMapper, buildStringShape(shape, rigids)));
      int NOFPARAMS = mapper.exampleInput().size();
      for (int point = 0; point < NOFPOINTS; ++point) {
        double[] baseGenotype = IntStream.range(0, NOFPARAMS).mapToDouble(i -> rg.nextDouble(-1, 1)).toArray();
        genotypes = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        results = new ArrayList<>(NOFTRIALS * FRAGMENTATIONS);
        baseResult = executorService.submit(() ->
                fitness.apply(task.run(mapper.apply(Arrays.stream(baseGenotype).boxed().toList()), engine.get())));
        for (int trial = 0; trial < NOFTRIALS; ++trial) {
          double[] randomVector = IntStream.range(0, NOFPARAMS).mapToDouble(i -> rg.nextGaussian()).toArray();
          double norm = Math.sqrt(Arrays.stream(randomVector).boxed().mapToDouble(i -> Math.pow(i, 2)).sum());
          for (int i = 0; i < randomVector.length; ++i) {
            randomVector[i] *= (SEGMENTLENGTH / norm);
          }
          for (int iter = 1; iter < FRAGMENTATIONS + 1; ++iter) {
            double tick = iter / (double) FRAGMENTATIONS;
            double[] placeholder = new double[randomVector.length];
            for (int i = 0; i < randomVector.length; ++i) {
              placeholder[i] = baseGenotype[i] + tick * randomVector[i];
            }
            genotypes.add(placeholder);
            results.add(executorService.submit(
                    () -> fitness.apply(task.run(mapper.apply(Arrays.stream(placeholder).boxed().toList()), engine.get()))));
          }
        }
        try {
          if (saveG) {
            writer.write(String.format("%d;%d;%d;%s;", rigids, point, -1, serialize(Arrays.stream(baseGenotype).boxed().toList())) + baseResult.get() + "\n");
            for (int counter = 0; counter < results.size(); ++counter) {
              writer.write(
                      String.format("%d;%d;%d;%s;", rigids, point, counter / FRAGMENTATIONS, serialize(Arrays.stream(genotypes.get(counter)).boxed().toList())) +
                              results.get(counter).get() + "\n");
            }
          } else {
            writer.write(String.format("%d;%d;%d;", rigids, point, -1) + baseResult.get() + "\n");
            for (int counter = 0; counter < results.size(); ++counter) {
              writer.write(String.format("%d;%d;%d;", rigids, point, counter / FRAGMENTATIONS) + results.get(counter).get() + "\n");
            }
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
    boolean[][] rigids;
    switch (shape) {
      case BIPED:
        rigids = new boolean[4][3];
        for (int i = 0; i < 12; ++i) {
          rigids[i / 3][i % 3] = false;
        }
        for (int i = 0; i < nOfRigids && i < 6; ++i) {
          rigids[3 * (i % 2)][2 - i / 2] = true;
        }
        for (int i = 0; i < nOfRigids - 6 && i < 4; ++i) {
          rigids[1 + (i % 2)][i % 2] = true;
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
        break;
      case WORM:
        rigids = new boolean[5][2];
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
        break;
      case T:
        rigids = new boolean[2][5];
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
        break;
      case PLUS:
        rigids = new boolean[6][6];
        for (int i = 0; i < 36; ++i) {
          rigids[i / 6][i % 6] = false;
        }
        for (int i = 0; i < nOfRigids && i < 6; ++i) {
          rigids[i][2 + i % 2] = true;
        }
        for (int i = 0; i < nOfRigids - 6 && i < 2; ++i) {
          rigids[2 + i % 2][i] = true;
        }
        for (int i = 0; i < nOfRigids - 8 && i < 2; ++i) {
          rigids[2 + i % 2][5 - i] = true;
        }
        for (int i = 0; i < nOfRigids - 10 && i < 2; ++i) {
          rigids[i][3 - i % 2] = true;
        }
        for (int i = 0; i < nOfRigids - 12 && i < 2; ++i) {
          rigids[3 - i % 2][i] = true;
        }
        for (int i = 0; i < nOfRigids - 14 && i < 2; ++i) {
          rigids[3 - i % 2][5 - i] = true;
        }
        for (int i = 0; i < nOfRigids - 16 && i < 2; ++i) {
          rigids[5 - i][2 + i % 2] = true;
        }
        for (int i = 0; i < 2; ++i) {
          rigidsString.append(String.format("..%s%s..-", rigids[i][2] ? "r" : "s", rigids[i][3] ? "r" : "s"));
        }
        for (int i = 2; i < 4; ++i) {
          rigidsString.append(String.format("%s%s%s%s%s%s-", rigids[i][0] ? "r" : "s", rigids[i][1] ? "r" : "s",
                  rigids[i][2] ? "r" : "s", rigids[i][3] ? "r" : "s", rigids[i][4] ? "r" : "s", rigids[i][5] ? "r" : "s"));
        }
        rigidsString.append(String.format("..%s%s..-", rigids[4][2] ? "r" : "s", rigids[4][3] ? "r" : "s"));
        rigidsString.append(String.format("..%s%s..", rigids[5][2] ? "r" : "s", rigids[5][3] ? "r" : "s"));
        break;
    }
    return rigidsString.toString();
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

  public static double[] sinCAdjustParams(double[] params, String stringShape) {
    int nOfVoxels = stringShape.replace(".", "").replace("-", "").length();
    int nOfSofts = stringShape.replace(".", "").replace("-", "").replace("r", "").length();
    double[] adjustedParams = new double[nOfVoxels * 2];
    int fullArrayIndex = 0;
    int conditionalIndex = 0;
    String[] splitStringShape = stringShape.split("-");
    for (int i = 0; i < splitStringShape.length * splitStringShape[0].length(); ++i) {
      switch (splitStringShape[splitStringShape.length - i % splitStringShape.length - 1].charAt(i / splitStringShape.length)) {
        case 'r':
          adjustedParams[fullArrayIndex] = 0;
          adjustedParams[fullArrayIndex + nOfVoxels] = 0;
          ++fullArrayIndex;
          break;
        case 's':
          adjustedParams[fullArrayIndex] = params[conditionalIndex];
          adjustedParams[fullArrayIndex + nOfVoxels] = params[conditionalIndex + nOfSofts];
          ++conditionalIndex;
          ++fullArrayIndex;
          break;
        default:
      }
    }
    return adjustedParams;
  }
}