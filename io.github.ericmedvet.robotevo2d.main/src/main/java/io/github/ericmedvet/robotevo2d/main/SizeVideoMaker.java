/*-
 * ========================LICENSE_START=================================
 * robotevo2d-main
 * %%
 * Copyright (C) 2018 - 2025 Eric Medvet
 * %%
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =========================LICENSE_END==================================
 */
package io.github.ericmedvet.robotevo2d.main;

import io.github.ericmedvet.jgea.core.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.jviz.core.drawer.VideoBuilder;
import io.github.ericmedvet.jviz.core.util.VideoUtils;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.AbstractIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.NumIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.locomotion.PrebuiltIndependentLocomotion;
import io.github.ericmedvet.mrsim2d.viewer.Drawer;
import io.github.ericmedvet.mrsim2d.viewer.TaskVideoBuilder;

import java.io.*;
import java.util.*;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.logging.ConsoleHandler;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

@SuppressWarnings("unchecked")
public class SizeVideoMaker {
  final static NamedBuilder<Object> nb = NamedBuilder.fromDiscovery();
  final static String path = "C:/Users/Francesco/Desktop/Università/Dottorato/Ricerca/Size/";
  private static final Logger L = Logger.getLogger(SizeVideoMaker.class.getName());

  private static Object base64Deserializer(String serialized) {
    byte[] bytes = Base64.getDecoder().decode(serialized);
    try (ObjectInputStream oois = new ObjectInputStream(new ByteArrayInputStream(bytes))) {
      return oois.readObject();
    } catch (IOException | ClassNotFoundException e) {
      throw new RuntimeException(e);
    }
  }

  public static void main(String[] args) throws IOException {
    L.setUseParentHandlers(false);
    ConsoleHandler handler = new ConsoleHandler();
    handler.setLevel(Level.ALL);
    handler.setFormatter(new Formatter() {
      public String format(LogRecord r) {
        return String.format(
            "[%1$tm-%1$td %1$tH:%1$tM:%1$tS] %4$4.4s %5$s%n",
            new java.util.Date(r.getMillis()),
            r.getSourceClassName(),
            r.getLoggerName(),
            r.getLevel().getName(),
            r.getMessage()
        );
      }
    });
    L.addHandler(handler);
    L.setLevel(Level.ALL);

    allVideos("flat-20-30");
    allVideos("downhill20-20-30");
    for (String task : List.of("downhill10", "downhill20", "flat", "hilly2")) {
      allVideos("%s-20-60".formatted(task));
    }
  }

  public static void allVideos(String exp) throws IOException {
    final BufferedReader reader = new BufferedReader(new FileReader(path + "Csv/%s-finals.csv".formatted(exp)));
    final Function<List<Double>, Supplier<NumIndependentVoxel>> mapper = ((InvertibleMapper<List<Double>, Supplier<NumIndependentVoxel>>) nb
        .build(
            "er.m.dsToNIV(" + "  sensors = [" + "    s.sensors.sin(); s.sensors.a(); s.sensors.ar(); s.sensors.rv(a = 0); s.sensors.rv(a = 90);" + "    s.sensors.d(a = 0; r = 5); s.sensors.d(a = 45; r = 5); s.sensors.d(a = 90; r = 5); s.sensors.d(a = 135; r = 5);" + "    s.sensors.d(a = 180; r = 5); s.sensors.d(a = 225; r = 5); s.sensors.d(a = 270; r = 5); s.sensors.d(a = 315; r = 5);" + "    s.sensors.sc(s = N); s.sensors.sc(s = E); s.sensors.sc(s = S); s.sensors.sc(s = W);" + "    s.sensors.sa(s = N); s.sensors.sa(s = E); s.sensors.sa(s = S); s.sensors.sa(s = W);" + "    s.sensors.c()" + "  ];" + "  function = ds.num.stepped(" + "    stepT = 0.2;" + "    inner = ds.num.mlp(" + "      nOfInnerLayers = 2;" + "      innerLayerRatio = 2" + "    )" + "  )" + ")"
        )).mapperFor(null);
    final String task = getTask(exp);
    final PrebuiltIndependentLocomotion taskRunner = (PrebuiltIndependentLocomotion) nb.build(task);
    final Supplier<Engine> engineSupplier = (Supplier<Engine>) nb.build("sim.engine()");
    final Supplier<Drawer> drawerSupplier = () -> ((Function<String, Drawer>) nb.build("sim.drawer()")).apply("");
    String line = reader.readLine();
    String[] splitLine = line.split(";");
    final int fitnessIndex = Arrays.stream(splitLine).toList().indexOf("best→quality→all.agents.final.avg.w");
    final int genotypeIndex = Arrays.stream(splitLine).toList().indexOf("best→genotype→to.base64");
    int counter = 0;
    while (Objects.nonNull(line = reader.readLine())) {
      splitLine = line.split(";");
      final List<Double> genotype = (List<Double>) base64Deserializer(splitLine[genotypeIndex]);
      final double fitness = Double.parseDouble(splitLine[fitnessIndex]);
      L.info(
          "Fitness: %.4f vs %.4f".formatted(
              fitness,
              taskRunner.run(() -> mapper.apply(genotype).get(), engineSupplier.get()).allAgentsFinalAverageWidth()
          )
      );
      TaskVideoBuilder<Supplier<AbstractIndependentVoxel>> taskVideoBuilder = new TaskVideoBuilder<>(
          taskRunner,
          s -> drawerSupplier.get(),
          engineSupplier,
          "",
          0,
          30,
          30
      );
      taskVideoBuilder.save(
          new VideoBuilder.VideoInfo(800, 600, VideoUtils.EncoderFacility.JCODEC),
          new File(path + "Videos/%s-%d.mp4".formatted(exp, ++counter)),
          mapper.apply(genotype)::get
      );
    }
  }

  private static String getTask(String exp) {
    String[] expSplit = exp.split("-");
    int param1 = Integer.parseInt("0" + expSplit[0].replaceAll("[a-z]", ""));
    int nOfAgents = Integer.parseInt(expSplit[1]);
    int duration = Integer.parseInt(expSplit[2].replaceAll("[a-z]", ""));
    return "s.task.prebuiltIndependentLocomotion(" + "duration = %d;".formatted(duration) + switch (exp.split("-")[0]
        .replaceAll("[0-9]", "")) {
      case "downhill" -> "terrain = sim.terrain.downhill(a = %d);".formatted(param1);
      case "hilly" -> "terrain = sim.terrain.hilly(seed = %d);".formatted(param1);
      case "flat" -> "terrain = sim.terrain.flat();";
      default -> "";
    } + "shape = s.a.vsr.shape.free(s = \"%s\")".formatted(posConfig(nOfAgents)) + ")";
  }

  private static String posConfig(int n) {
    return switch (n) {
      case 20 -> "sssss-sssss-sssss-sssss";
      default -> "";
    };
  }
}