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

import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.jsdynsym.core.composed.OutStepped;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import io.github.ericmedvet.jviz.core.drawer.VideoBuilder;
import io.github.ericmedvet.jviz.core.util.VideoUtils;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.AbstractIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.NumIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;
import io.github.ericmedvet.mrsim2d.viewer.TaskVideoBuilder;
import java.io.*;
import java.util.*;
import java.util.function.Supplier;
import java.util.stream.IntStream;

@SuppressWarnings("unchecked")
public class NoiseTester {
  final static NamedBuilder<Object> nb = NamedBuilder.fromDiscovery();
  final static String path = "";

  public static void main(String[] args) {
    final int minDim = Integer.parseInt(args[0]);
    final int maxDim = Integer.parseInt(args[1]);
    final int stepDim = Integer.parseInt(args[2]);
    for (int i = minDim; i <= maxDim; i += stepDim) {
      noiseTest(i, i);
    }
  }

  public static void noiseTest(long seed, double size) {
    Random rng = new Random(seed);
    final boolean[] switcher = new boolean[5];
    Supplier<NumIndependentVoxel> supplier;
    TaskVideoBuilder<Supplier<NumIndependentVoxel>> taskVideoBuilder = (TaskVideoBuilder<Supplier<NumIndependentVoxel>>) nb
        .build(
            "sim.taskVideoBuilder(task = s.task.prebuiltIndependentLocomotion(shape = s.a.vsr.shape.free(s = \"sssss\")))"
        );
    for (int i = 0; i < 32; ++i) {
      for (int j = 0; j < 5; ++j) {
        switcher[j] = (i & (1 << j)) != 0;
      }
      final int[] index = {0};
      supplier = () -> {
        boolean actualSwitcher = switcher[index[0]];
        ++index[0];
        final double length = AbstractIndependentVoxel.VOXEL_SIDE_LENGTH * (actualSwitcher ? size : 1 / size);
        return new NumIndependentVoxel(
            new Voxel.Material(),
            length,
            AbstractIndependentVoxel.VOXEL_MASS,
            List.of(),
            NumIndependentVoxel.AreaActuation.SIDES,
            true,
            0,
            NumericalDynamicalSystem.from(
                new OutStepped<>(
                    NumericalStatelessSystem.from(
                        0,
                        8,
                        (a, d) -> IntStream.range(0, 8).mapToDouble(k -> 2 * rng.nextDouble() - 1).toArray()
                    ),
                    .2
                ),
                0,
                8
            )
        );
      };
      taskVideoBuilder.save(
          new VideoBuilder.VideoInfo(800, 600, VideoUtils.EncoderFacility.DEFAULT),
          new File(path + "Dimtest/size-%1.1f-seed-%d-%d.mp4".formatted(size, seed, i)),
          supplier
      );
    }
  }
}