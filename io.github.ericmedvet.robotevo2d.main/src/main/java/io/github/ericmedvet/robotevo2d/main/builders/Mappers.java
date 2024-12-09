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

package io.github.ericmedvet.robotevo2d.main.builders;

import io.github.ericmedvet.jgea.core.InvertibleMapper;
import io.github.ericmedvet.jgea.core.representation.NamedMultivariateRealFunction;
import io.github.ericmedvet.jgea.core.representation.sequence.integer.IntString;
import io.github.ericmedvet.jnb.core.*;
import io.github.ericmedvet.jnb.datastructure.*;
import io.github.ericmedvet.jsdynsym.buildable.builders.NumericalDynamicalSystems;
import io.github.ericmedvet.jsdynsym.core.composed.Composed;
import io.github.ericmedvet.jsdynsym.core.numerical.MultivariateRealFunction;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.mrsim2d.buildable.builders.ReactiveVoxels;
import io.github.ericmedvet.mrsim2d.buildable.builders.Sensors;
import io.github.ericmedvet.mrsim2d.core.NumMultiBrained;
import io.github.ericmedvet.mrsim2d.core.Sensor;
import io.github.ericmedvet.mrsim2d.core.actions.Sense;
import io.github.ericmedvet.mrsim2d.core.actions.SenseDistanceToBody;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.CentralizedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.DistributedNumGridVSR;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.GridBody;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.ReactiveGridVSR;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.AbstractIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.NumIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.bodies.Body;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import java.util.random.RandomGenerator;

@Discoverable(prefixTemplate = "evorobots|er.mapper|m")
public class Mappers {

    private Mappers() {}

    private static int argmax(double[] values) {
        if (values.length == 0) {
            throw new IllegalArgumentException("Empty array");
        }
        int i = 0;
        for (int j = 1; j < values.length; j++) {
            if (values[j] > values[i]) {
                i = j;
            }
        }
        return i;
    }

    @SuppressWarnings("unused")
    public static <X> InvertibleMapper<X, Supplier<DistributedNumGridVSR>> bodyBrainHomoDistributedVSR(
            @Param(value = "of", dNPM = "ea.m.identity()")
                    InvertibleMapper<X, Pair<Grid<GridBody.VoxelType>, NumericalDynamicalSystem<?>>> beforeM,
            @Param(value = "w", dI = 10) int w,
            @Param(value = "h", dI = 10) int h,
            @Param(value = "nOfSignals", dI = 1) int nOfSignals,
            @Param(value = "directional", dB = true) boolean directional,
            @Param(value = "sensors") List<Sensor<? super Body>> sensors) {
        Pair<Grid<GridBody.VoxelType>, NumericalDynamicalSystem<?>> ePair = new Pair<>(
                Grid.create(w, h, GridBody.VoxelType.SOFT),
                NumericalDynamicalSystems.Builder.empty()
                        .apply(
                                DistributedNumGridVSR.nOfInputs(sensors, nOfSignals, directional),
                                DistributedNumGridVSR.nOfOutputs(sensors, nOfSignals, directional)));
        return InvertibleMapper.from(
                (supplier, x) -> () -> {
                    Grid<GridBody.VoxelType> grid = GridUtils.largestConnected(
                            beforeM.mapperFor(ePair).apply(x).first(),
                            vt -> !vt.equals(GridBody.VoxelType.NONE),
                            GridBody.VoxelType.NONE);
                    if (grid.values().stream().allMatch(vt -> vt.equals(GridBody.VoxelType.NONE))) {
                        grid = Grid.create(1, 1, GridBody.VoxelType.RIGID);
                    }
                    return new DistributedNumGridVSR(
                            new GridBody(grid.map(vt -> new GridBody.SensorizedElement(
                                    new GridBody.Element(vt, Voxel.DEFAULT_MATERIAL), sensors))),
                            grid.map(vt -> vt.equals(GridBody.VoxelType.NONE)
                                    ? null
                                    : beforeM.mapperFor(ePair).apply(x).second()),
                            nOfSignals,
                            directional);
                },
                supplier -> beforeM.exampleFor(ePair),
                "%s→bodyBrainHomoDistributedVSR[%dx%d;nOfSignals=%d;directional=%s]"
                        .formatted(beforeM, w, h, nOfSignals, directional));
    }

    private static void checkIOSizeConsistency(NumMultiBrained numMultiBrained) {
        List<Integer> inSizes = numMultiBrained.brains().stream()
                .map(NumericalDynamicalSystem::nOfInputs)
                .distinct()
                .toList();
        List<Integer> outSizes = numMultiBrained.brains().stream()
                .map(NumericalDynamicalSystem::nOfOutputs)
                .distinct()
                .toList();
        if (inSizes.size() != 1) {
            throw new IllegalArgumentException("Not all of the %d brains has the same input size: %s sizes found"
                    .formatted(numMultiBrained.brains().size(), inSizes));
        }
        if (outSizes.size() != 1) {
            throw new IllegalArgumentException("Not all of the %d brains has the same output size: %s sizes found"
                    .formatted(numMultiBrained.brains().size(), outSizes));
        }
    }

    private static void checkNumericalParametrizedSizeConsistency(NumMultiBrained numMultiBrained) {
        List<Integer> brainSizes = numMultiBrained.brains().stream()
                .map(b -> ((double[]) Composed.shallowest(b, NumericalParametrized.class)
                                .orElseThrow()
                                .getParams())
                        .length)
                .distinct()
                .toList();
        if (brainSizes.size() != 1) {
            throw new IllegalArgumentException("Not all of the %d brains has the same output size: %s sizes found"
                    .formatted(numMultiBrained.brains().size(), brainSizes));
        }
    }

    private static <T> void checkType(NumMultiBrained numMultiBrained, Class<T> clazz) {
        for (NumericalDynamicalSystem<?> nds : numMultiBrained.brains()) {
            if (Composed.shallowest(nds, clazz).isEmpty()) {
                throw new IllegalArgumentException("Some of the %d brains is not a %s"
                        .formatted(numMultiBrained.brains().size(), clazz.getSimpleName()));
            }
        }
    }

    @SuppressWarnings("unused")
    public static <X, T extends NumMultiBrained> InvertibleMapper<X, Supplier<T>> dsToNpHeteroBrains(
            @Param(value = "of", dNPM = "ea.m.identity()") InvertibleMapper<X, List<Double>> beforeM,
            @Param("target") T target,
            @Param(value = "", injection = Param.Injection.MAP) ParamMap map,
            @Param(value = "", injection = Param.Injection.BUILDER) NamedBuilder<?> builder) {
        checkType(target, NumericalParametrized.class);
        List<Integer> brainSizes = target.brains().stream()
                .map(b -> ((double[]) Composed.shallowest(b, NumericalParametrized.class)
                                .orElseThrow()
                                .getParams())
                        .length)
                .toList();
        int overallBrainSize = brainSizes.stream().mapToInt(i -> i).sum();
        return beforeM.andThen(InvertibleMapper.from(
                (supplier, values) -> {
                    if (values.size() != overallBrainSize) {
                        throw new IllegalArgumentException("Wrong number of params: %d expected, %d found"
                                .formatted(overallBrainSize, values.size()));
                    }
                    return () -> {
                        @SuppressWarnings("unchecked")
                        T t = (T) builder.build((NamedParamMap) map.value("target", ParamMap.Type.NAMED_PARAM_MAP));
                        int c = 0;
                        for (NumericalDynamicalSystem<?> brain : t.brains()) {
                            int brainSize = ((double[]) Composed.shallowest(brain, NumericalParametrized.class)
                                            .orElseThrow()
                                            .getParams())
                                    .length;
                            //noinspection unchecked
                            Composed.shallowest(brain, NumericalParametrized.class)
                                    .orElseThrow()
                                    .setParams(values.subList(c, c + brainSize).stream()
                                            .mapToDouble(d -> d)
                                            .toArray());
                            c = c + brainSize;
                        }
                        return t;
                    };
                },
                supplier -> Collections.nCopies(overallBrainSize, 0d),
                "dsToNpHeteroBrains"));
    }

    @Alias(
            name = "dsToNIV",
            value = // spotless:off
                    """
                        noisedDsToNIV(
                            bodySizeSigma = 0;
                            sensorDistanceSigma = 0;
                            sideContractionSigma = 0;
                            parametersSigma = 0
                        )
                    """
                    // spotless:on
            )
    @SuppressWarnings("unused")
    public static <X> InvertibleMapper<X, Supplier<NumIndependentVoxel>> noisedDsToNIV(
            @Param(value = "of", dNPM = "ea.m.identity()") InvertibleMapper<X, List<Double>> beforeM,
            @Param("sensors") List<Sensor<? super Voxel>> sensors,
            @Param(value = "areaActuation", dS = "sides") NumIndependentVoxel.AreaActuation areaActuation,
            @Param(value = "attachActuation", dB = true) boolean attachActuation,
            @Param(value = "nOfNFCChannels", dI = 1) int nOfNFCChannels,
            @Param("function") NumericalDynamicalSystems.Builder<?, ?> numericalDynamicalSystemBuilder,
            @Param("bodySizeSigma") double sizeNoise,
            @Param("sensorDistanceSigma") double sensorNoise,
            @Param("sideContractionSigma") double contractionNoise,
            @Param("parametersSigma") double paramNoise,
            @Param(value = "randomGenerator", dNPM = "m.defaultRG(seed = -1)") RandomGenerator randomGenerator) {
        NumericalDynamicalSystem<?> controller = numericalDynamicalSystemBuilder.apply(
                MultivariateRealFunction.varNames("x", NumIndependentVoxel.nOfInputs(sensors, nOfNFCChannels)),
                MultivariateRealFunction.varNames(
                        "y", NumIndependentVoxel.nOfOutputs(areaActuation, attachActuation, nOfNFCChannels)));
        return beforeM.andThen(InvertibleMapper.from(
                (supplier, values) -> () -> {
                    final double sideLength =
                            AbstractIndependentVoxel.VOXEL_SIDE_LENGTH * randomGenerator.nextGaussian(1, sizeNoise);
                    final List<Sensor<? super Voxel>> newSensors = new ArrayList<>();
                    for (Sensor<? super Voxel> s : sensors) {
                        Sense<? extends Body> sensorType = s.apply(null);
                        if (sensorType instanceof SenseDistanceToBody sd) {
                            newSensors.add(Sensors.d(
                                    Math.toDegrees(sd.direction()),
                                    sd.distanceRange() * randomGenerator.nextGaussian(1, sensorNoise)));
                        } else {
                            newSensors.add(s);
                        }
                    }
                    final double sideContraction = new DoubleRange(0d, .4)
                            .clip(Voxel.Material.DEFAULT_AREA_RATIO
                                    * randomGenerator.nextGaussian(1, contractionNoise));
                    final double[] newValues = values.stream()
                            .mapToDouble(d -> d * randomGenerator.nextGaussian(1, paramNoise))
                            .toArray();
                    NumIndependentVoxel agent = new NumIndependentVoxel(
                            new Voxel.Material(Voxel.Material.SOFTNESS, sideContraction),
                            sideLength,
                            AbstractIndependentVoxel.VOXEL_MASS,
                            newSensors,
                            areaActuation,
                            attachActuation,
                            nOfNFCChannels,
                            numericalDynamicalSystemBuilder.apply(
                                    MultivariateRealFunction.varNames(
                                            "x", NumIndependentVoxel.nOfInputs(sensors, nOfNFCChannels)),
                                    MultivariateRealFunction.varNames(
                                            "y",
                                            NumIndependentVoxel.nOfOutputs(
                                                    areaActuation, attachActuation, nOfNFCChannels))));
                    Composed.shallowest(agent.brain(), NumericalParametrized.class)
                            .orElseThrow()
                            .setParams(newValues);
                    return agent;
                },
                supplier -> Collections.nCopies(
                        ((double[]) Composed.shallowest(controller, NumericalParametrized.class)
                                        .orElseThrow()
                                        .getParams())
                                .length,
                        0d),
                "noisedDsToNIV"));
    }

    @SuppressWarnings("unused")
    public static <X, T extends NumMultiBrained> InvertibleMapper<X, Supplier<T>> dsToNpHomoBrains(
            @Param(value = "of", dNPM = "ea.m.identity()") InvertibleMapper<X, List<Double>> beforeM,
            @Param("target") T target,
            @Param(value = "", injection = Param.Injection.MAP) ParamMap map,
            @Param(value = "", injection = Param.Injection.BUILDER) NamedBuilder<?> builder) {
        checkType(target, NumericalParametrized.class);
        checkIOSizeConsistency(target);
        checkNumericalParametrizedSizeConsistency(target);
        int brainSize = target.brains().stream()
                .map(b -> ((double[]) Composed.shallowest(b, NumericalParametrized.class)
                                .orElseThrow()
                                .getParams())
                        .length)
                .findFirst()
                .orElseThrow();
        return beforeM.andThen(InvertibleMapper.from(
                (supplier, values) -> {
                    if (values.size() != brainSize) {
                        throw new IllegalArgumentException(
                                "Wrong number of params: %d expected, %d found".formatted(brainSize, values.size()));
                    }
                    return () -> {
                        @SuppressWarnings("unchecked")
                        T t = (T) builder.build((NamedParamMap) map.value("target", ParamMap.Type.NAMED_PARAM_MAP));
                        //noinspection unchecked
                        t.brains().forEach(b -> Composed.shallowest(b, NumericalParametrized.class)
                                .orElseThrow()
                                .setParams(values.stream().mapToDouble(d -> d).toArray()));
                        return t;
                    };
                },
                supplier -> Collections.nCopies(brainSize, 0d),
                "dsToNpHomoBrains"));
    }

    @SuppressWarnings("unused")
    public static <X> InvertibleMapper<X, Supplier<ReactiveGridVSR>> isToReactiveGridVsr(
            @Param(value = "of", dNPM = "ea.m.identity()") InvertibleMapper<X, IntString> beforeM,
            @Param("w") int w,
            @Param("h") int h,
            @Param("availableVoxels") List<Supplier<ReactiveGridVSR.ReactiveVoxel>> availableVoxels) {
        IntString exampleGenotype = new IntString(Collections.nCopies(w * h, 0), 0, availableVoxels.size() + 1);
        return beforeM.andThen(InvertibleMapper.from(
                (supplier, s) -> {
                    Grid<Integer> indexGrid = Grid.create(w, h, s.genes());
                    Grid<ReactiveGridVSR.ReactiveVoxel> body;
                    if (indexGrid.values().stream().max(Integer::compareTo).orElse(0) == 0) {
                        body = Grid.create(1, 1, ReactiveVoxels.ph());
                    } else {
                        body = GridUtils.fit(GridUtils.largestConnected(indexGrid, i -> i > 0, 0), i -> i > 0)
                                .map(i -> i == 0
                                        ? ReactiveVoxels.none()
                                        : availableVoxels.get(i - 1).get());
                    }
                    return () -> new ReactiveGridVSR(body);
                },
                supplier -> exampleGenotype,
                "isToReactiveGridVsr[w=%d;h=%d]".formatted(w, h)));
    }

    @SuppressWarnings("unused")
    public static <X> InvertibleMapper<X, Supplier<CentralizedNumGridVSR>> ndsToFixedBodyCentralizedVSR(
            @Param(value = "of", dNPM = "ea.m.identity()") InvertibleMapper<X, NumericalDynamicalSystem<?>> beforeM,
            @Param("body") GridBody body,
            @Param(value = "", injection = Param.Injection.MAP) ParamMap map,
            @Param(value = "", injection = Param.Injection.BUILDER) NamedBuilder<?> builder) {
        return beforeM.andThen(InvertibleMapper.from(
                (supplier, nds) -> () -> new CentralizedNumGridVSR(body, nds),
                supplier -> NumericalDynamicalSystems.Builder.empty()
                        .apply(CentralizedNumGridVSR.nOfInputs(body), CentralizedNumGridVSR.nOfOutputs(body)),
                "nmrfToCentralizedVSR[body=%s]".formatted(map.value("body"))));
    }

    @SuppressWarnings("unused")
    public static <X> InvertibleMapper<X, Supplier<DistributedNumGridVSR>> ndsToFixedBodyHomoDistributedVSR(
            @Param(value = "of", dNPM = "ea.m.identity()") InvertibleMapper<X, NumericalDynamicalSystem<?>> beforeM,
            @Param("body") GridBody body,
            @Param(value = "nOfSignals", dI = 1) int nOfSignals,
            @Param(value = "directional", dB = true) boolean directional) {
        // check consistency
        List<Integer> inputSizes = body.grid().entries().stream()
                .filter(e -> !e.value().element().type().equals(GridBody.VoxelType.NONE))
                .map(e -> DistributedNumGridVSR.nOfInputs(body, e.key(), nOfSignals, directional))
                .distinct()
                .toList();
        List<Integer> outputSizes = body.grid().entries().stream()
                .filter(e -> !e.value().element().type().equals(GridBody.VoxelType.NONE))
                .map(e -> DistributedNumGridVSR.nOfOutputs(body, e.key(), nOfSignals, directional))
                .distinct()
                .toList();
        if (inputSizes.size() != 1 || outputSizes.size() != 1) {
            throw new IllegalArgumentException(
                    "Invalid input/output sizes, some voxels have different size: inputSizes=%s, outputSizes=%s"
                            .formatted(inputSizes, outputSizes));
        }
        NumericalDynamicalSystem<?> nds =
                NumericalDynamicalSystems.Builder.empty().apply(inputSizes.get(0), outputSizes.get(0));
        return InvertibleMapper.from(
                (supplier, x) -> () -> new DistributedNumGridVSR(
                        body,
                        body.grid()
                                .map(se -> se.element().type().equals(GridBody.VoxelType.NONE)
                                        ? null
                                        : beforeM.mapperFor(nds).apply(x)),
                        nOfSignals,
                        directional),
                supplier -> beforeM.exampleFor(nds),
                "%s→ndsToFixedBodyHomoDistributedVSR[nOfSignals=%d;directional=%s]"
                        .formatted(beforeM, nOfSignals, directional));
    }

    @SuppressWarnings("unused")
    public static <X> InvertibleMapper<X, Supplier<ReactiveGridVSR>> nmrfToReactiveGridVsr(
            @Param(value = "of", dNPM = "ea.m.identity()") InvertibleMapper<X, NamedMultivariateRealFunction> beforeM,
            @Param("w") int w,
            @Param("h") int h,
            @Param("availableVoxels") List<Supplier<ReactiveGridVSR.ReactiveVoxel>> availableVoxels) {
        return beforeM.andThen(InvertibleMapper.from(
                (supplier, nmrf) -> {
                    Grid<Integer> indexGrid = Grid.create(w, h, (x, y) -> {
                        double[] output = nmrf.apply(new double[] {(double) x / (double) w, (double) y / (double) h});
                        int iMax = argmax(output);
                        return output[iMax] > 0 ? iMax + 1 : 0;
                    });
                    Grid<ReactiveGridVSR.ReactiveVoxel> body;
                    if (indexGrid.values().stream().max(Integer::compareTo).orElse(0) == 0) {
                        body = Grid.create(1, 1, ReactiveVoxels.ph());
                    } else {
                        body = GridUtils.fit(GridUtils.largestConnected(indexGrid, i -> i > 0, 0), i -> i > 0)
                                .map(i -> i == 0
                                        ? ReactiveVoxels.none()
                                        : availableVoxels.get(i - 1).get());
                    }
                    return () -> new ReactiveGridVSR(body);
                },
                supplier -> NamedMultivariateRealFunction.from(
                        MultivariateRealFunction.from(vs -> vs, 2, availableVoxels.size()),
                        List.of("x", "y"),
                        MultivariateRealFunction.varNames("v", availableVoxels.size())),
                "nmrfToReactiveGridVsr[w=%d;h=%d]".formatted(w, h)));
    }
}
