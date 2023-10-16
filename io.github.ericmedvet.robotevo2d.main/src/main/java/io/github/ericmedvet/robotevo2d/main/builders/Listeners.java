/*-
 * ========================LICENSE_START=================================
 * robotevo2d-main
 * %%
 * Copyright (C) 2022 - 2023 Eric Medvet
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
package io.github.ericmedvet.robotevo2d.main.builders;

import io.github.ericmedvet.jgea.core.listener.AccumulatorFactory;
import io.github.ericmedvet.jgea.core.listener.ListenerFactory;
import io.github.ericmedvet.jgea.core.solver.state.POSetPopulationState;
import io.github.ericmedvet.jgea.experimenter.Experiment;
import io.github.ericmedvet.jgea.experimenter.Run;
import io.github.ericmedvet.jnb.core.Param;
import java.io.File;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.function.BiFunction;

public class Listeners {

  private Listeners() {}

  @SuppressWarnings("unused")
  public static <A>
      BiFunction<
              Experiment,
              ExecutorService,
              ListenerFactory<POSetPopulationState<?, A, ?>, Run<?, ?, A, ?>>>
          videoSaver(
              @Param("videos")
                  List<AccumulatorFactory<POSetPopulationState<?, A, ?>, File, Run<?, ?, A, ?>>>
                      accumulators) {
    return (experiment, executorService) ->
        ListenerFactory.all(accumulators.stream().map(AccumulatorFactory::withAutoGet).toList())
            .deferred(executorService);
  }
}
