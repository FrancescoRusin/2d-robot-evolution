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
import io.github.ericmedvet.mrsim2d.core.tasks.AgentsObservation;
import io.github.ericmedvet.mrsim2d.core.tasks.AgentsOutcome;
import io.github.ericmedvet.mrsim2d.core.tasks.locomotion.PrebuiltIndependentLocomotion;
import io.github.ericmedvet.mrsim2d.viewer.TaskVideoBuilder;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

public class Videomaker {
  static final String path = "C:/Users/Francesco/Desktop/Università/Dottorato/Ricerca/Cooperation";
  static final NamedBuilder<Object> nb = NamedBuilder.fromDiscovery();

  private record Individual(String s, List<Double> genotype, Double fitness) {
  }

  @SuppressWarnings("unchecked")
  public static void main(String[] args) throws IOException {
    final Function<String, Object> fromBase64 = (Function<String, Object>) nb.build("f.fromBase64()");
    final Function<List<Double>, Supplier<AbstractIndependentVoxel>> mapper = ((InvertibleMapper<
            List<Double>, Supplier<AbstractIndependentVoxel>>)
            nb.build(
                    "er.m.dsToNpHomoBrains(target = s.a.numIndependentVoxel(sensors = [s.sensors.sin(); s.sensors.a(); s.sensors.ar(); s.sensors.rv(a = 0); s.sensors.rv(a = 90);s.sensors.sc(s = N); s.sensors.sc(s = E); s.sensors.sc(s = S); s.sensors.sc(s = W);s.sensors.sa(s = N); s.sensors.sa(s = E); s.sensors.sa(s = S); s.sensors.sa(s = W);s.sensors.c(); s.sensors.d(a = 0; r = 5)];nOfNFCChannels = 1;"
                            + "function = ds.num.stepped(inner = ds.num.mlp(nOfInnerLayers = 3); stepT = 0.1)))"))
            .mapperFor(null);
    for (int hole : List.of(1, 2, 3)) {
      for (String fitness : List.of("xmax", "xavg", "xmin")) {
        Function<String, TaskVideoBuilder<Supplier<AbstractIndependentVoxel>>> videoBuilderBuilder =
                s -> (TaskVideoBuilder<Supplier<AbstractIndependentVoxel>>) nb.build(
                        "s.taskVideoBuilder(task = s.task.prebuiltIndependentLocomotion(terrain = s.terrain.holed(startW = 12; holeWs = [%d.05]); shape = s.a.vsr.shape.free(s = \"%s\")))"
                                .formatted(hole, s));
        final BufferedReader reader = new BufferedReader(
                new FileReader(path + "/Csv/coop-%dh-%s_best_fg.csv".formatted(hole, fitness)));
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
        Function<String, Function<Supplier<AbstractIndependentVoxel>, AgentsOutcome<AgentsObservation>>> PIL = s ->
                (Function<Supplier<AbstractIndependentVoxel>, AgentsOutcome<AgentsObservation>>) nb.build(
                        ("s.taskRunner(" +
                                "task = s.task.prebuiltIndependentLocomotion(" +
                                "terrain = s.terrain.holed(startW = 12; holeWs = [%d.05]); " +
                                "shape = s.a.vsr.shape.free(s = \"%s\")" +
                                ")" +
                                ")")
                                .formatted(hole, s));
        int index = -1;
        List<Function<AgentsOutcome<AgentsObservation>, Double>> fitnessFunctions = List.of(
                (Function<AgentsOutcome<AgentsObservation>, Double>) nb.build("sim.f.outcome.aaFinalMaxW()"),
                (Function<AgentsOutcome<AgentsObservation>, Double>) nb.build("sim.f.outcome.aaFinalAvgW()"),
                (Function<AgentsOutcome<AgentsObservation>, Double>) nb.build("sim.f.outcome.aaFinalMinW()")
        );
        for (Individual i : individuals) {
          ++index;
          for (String s : List.of("ssssssssss", "ssssssss", "ssssss", "ssss")) {
            AgentsOutcome<AgentsObservation> outcome = PIL.apply(s).apply(mapper.apply(i.genotype));
            System.out.printf("%s, %s, %d; %f\n", s, fitnessFunctions.stream().map(f -> f.apply(outcome)).toList(), index, i.fitness);
          }
          /*Video video = videoBuilderBuilder.apply(i.s).apply(mapper.apply(i.genotype));
          File file = new File(path + "/Videos/Hole%d_%s_%d_minih_vid%d.mp4".formatted(hole, fitness, i.s.length(), (++index) % 10));
          Files.write(file.toPath(), video.data(), StandardOpenOption.WRITE, StandardOpenOption.CREATE);*/
        }
      }
    }
  }
}