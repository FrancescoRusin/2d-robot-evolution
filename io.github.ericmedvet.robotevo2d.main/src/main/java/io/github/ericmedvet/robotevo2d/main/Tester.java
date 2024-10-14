package io.github.ericmedvet.robotevo2d.main;

import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.jnb.datastructure.Grid;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalStatelessSystem;
import io.github.ericmedvet.mrsim2d.buildable.builders.Terrains;
import io.github.ericmedvet.mrsim2d.core.agents.gridvsr.GridBody;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.NumIndependentVoxel;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.locomotion.PrebuiltIndependentLocomotion;
import io.github.ericmedvet.mrsim2d.viewer.Drawers;
import io.github.ericmedvet.mrsim2d.viewer.RealtimeViewer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

@SuppressWarnings("unchecked")
public class Tester {
    public static void main(String[] args) {
        Supplier<Engine> engineSupplier = (Supplier<Engine>) NamedBuilder.fromDiscovery().build("sim.engine()");
        Grid<GridBody.VoxelType> grid = Grid.create(1, 1, new ArrayList<>(Collections.nCopies(1, GridBody.VoxelType.SOFT)));
        PrebuiltIndependentLocomotion PIL =
                new PrebuiltIndependentLocomotion(10,
                        Terrains.holed(20, 10, List.of(3.05), 9, 30, 10, 100),
                        21.8, 0.1, grid);
        RealtimeViewer viewer = new RealtimeViewer(Drawers.basic("").profiled());
        System.out.println(PIL.run(() -> new NumIndependentVoxel(List.of(), NumIndependentVoxel.AreaActuation.OVERALL, true, 0,
                NumericalStatelessSystem.from(0, 5, (a, b) -> new double[]{0, -1, -1, -1, -1})),
                engineSupplier.get(), viewer).allAgentsFinalMaxWidth());
    }
}
