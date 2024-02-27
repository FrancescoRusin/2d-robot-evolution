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

import io.github.ericmedvet.jgea.core.listener.NamedFunction;
import io.github.ericmedvet.jgea.experimenter.InvertibleMapper;
import io.github.ericmedvet.jnb.core.Discoverable;
import io.github.ericmedvet.jnb.core.Param;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.Task;
import io.github.ericmedvet.robotevo2d.main.builders.PlayConsumers;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

@Discoverable(prefixTemplate = "evorobots|er")
public record Play<G, S, O>(
    @Param(value = "name", dS = "") String name,
    @Param(value = "genotype", dNPM = "ea.f.identity()") Function<G, G> genotype,
    @Param("mapper") InvertibleMapper<G, S> mapper,
    @Param("task") Task<S, O> task,
    @Param(value = "engine", dNPM = "sim.engine()") Supplier<Engine> engineSupplier,
    @Param("consumers") List<PlayConsumers.ProducingConsumer> consumers,
    @Param("outcomeFunctions") List<NamedFunction<?, ?>> outcomeFunctions) {}
