/*-
 * ========================LICENSE_START=================================
 * robotevo2d-main
 * %%
 * Copyright (C) 2018 - 2024 Eric Medvet
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
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.AbstractIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.geometry.Point;
import io.github.ericmedvet.mrsim2d.core.tasks.AgentsObservation;
import io.github.ericmedvet.mrsim2d.core.tasks.locomotion.PrebuiltIndependentLocomotion;
import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

public class AltPerformanceMeasurer {
  static final String path =
      "C:/Users/Francesco/Desktop/Università/Dottorato/Ricerca/Cooperation/Csv/Processed/Genotypes/";
  static final NamedBuilder<Object> nb = NamedBuilder.fromDiscovery();

  private record Individual(String s, List<Double> genotype, Double fitness) {}

  @SuppressWarnings("unchecked")
  public static void main(String[] args) throws IOException {
    final Function<String, Object> fromBase64 = (Function<String, Object>) nb.build("f.fromBase64()");
    final Function<List<Double>, Supplier<AbstractIndependentVoxel>> mapper = ((InvertibleMapper<
                List<Double>, Supplier<AbstractIndependentVoxel>>)
            nb.build(
                "er.m.dsToNpHomoBrains(target = s.a.numIndependentVoxel(sensors = [s.sensors.sin(); s.sensors.a(); s.sensors.ar(); s.sensors.rv(a = 0); s.sensors.rv(a = 90);s.sensors.sc(s = N); s.sensors.sc(s = E); s.sensors.sc(s = S); s.sensors.sc(s = W);s.sensors.sa(s = N); s.sensors.sa(s = E); s.sensors.sa(s = S); s.sensors.sa(s = W);s.sensors.c(); s.sensors.d(a = 0; r = 5)];nOfNFCChannels = 1;"
                    + "function = ds.num.stepped(inner = ds.num.mlp(nOfInnerLayers = 3); stepT = 0.1)))"))
        .mapperFor(null);
    final Supplier<Engine> engine = (Supplier<Engine>) nb.build("sim.engine()");
    for (int hole : List.of(1, 2, 3)) {
      for (String fitness : List.of("xmax", "xavg", "xmin")) {
        for (String pos : List.of("", "-closeh")) {
          Function<String, PrebuiltIndependentLocomotion> taskBuilder =
              s -> (PrebuiltIndependentLocomotion) nb.build(
                  "s.task.prebuiltIndependentLocomotion(terrain = s.terrain.holed(startW = %d; holeWs = [%d.05]); shape = s.a.vsr.shape.free(s = \"%s\"))"
                      .formatted(pos.isEmpty() ? 20 : 12, hole, s));
          final BufferedReader reader = new BufferedReader(
              new FileReader(path + "coop-%dh-%s%s_best_fg.csv".formatted(hole, fitness, pos)));
          List<Individual> individuals = new ArrayList<>();
          String line;
          while (Objects.nonNull(line = reader.readLine())) {
            String[] splitLine = line.split(";");
            individuals.add(new Individual(
                splitLine[0],
                (List<Double>) fromBase64.apply(splitLine[1]),
                Double.parseDouble(splitLine[2])));
          }
          reader.close();
          final BufferedWriter writer = new BufferedWriter(
              new FileWriter(path + "Extras/coop-%dh-%s%s-extras.csv".formatted(hole, fitness, pos)));
          for (Individual i : individuals) {
            final AgentsObservation finalObs = taskBuilder
                .apply(i.s)
                .run(mapper.apply(i.genotype), engine.get())
                .snapshots()
                .lastEntry()
                .getValue();
            writer.write("%s;%f".formatted(i.s, i.fitness));
            for (Point p : finalObs.getCenters()) {
              writer.write(";%f;%f".formatted(p.x(), p.y()));
            }
            writer.write('\n');
          }
          writer.close();
        }
      }
    }
  }
}
