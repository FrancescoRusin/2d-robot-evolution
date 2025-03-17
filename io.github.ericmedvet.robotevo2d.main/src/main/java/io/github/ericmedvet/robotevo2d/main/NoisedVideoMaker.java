package io.github.ericmedvet.robotevo2d.main;

import io.github.ericmedvet.jgea.core.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.jviz.core.drawer.VideoBuilder;
import io.github.ericmedvet.jviz.core.util.VideoUtils;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.agents.independentvoxel.NumIndependentVoxel;
import io.github.ericmedvet.mrsim2d.viewer.TaskVideoBuilder;

import java.io.*;
import java.util.*;
import java.util.function.Function;
import java.util.function.Supplier;

@SuppressWarnings("unchecked")
public class NoisedVideoMaker {
    final static NamedBuilder<Object> nb = NamedBuilder.fromDiscovery();
    final static String path = "C:/Users/Francesco/Desktop/Università/Dottorato/Ricerca/Noise/";

    private static Object base64Deserializer(String serialized) {
        byte[] bytes = Base64.getDecoder().decode(serialized);
        try (ObjectInputStream oois = new ObjectInputStream(new ByteArrayInputStream(bytes))) {
            return oois.readObject();
        } catch (IOException | ClassNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    public static void main(String[] args) throws IOException {
        noiseVideos("noised-01-holed-anch");
    }

    public static void noisePilingVideos(String exp) throws IOException {
        final BufferedReader reader = new BufferedReader(new FileReader(
                path + "Csv/Piling/%s-finals.csv".formatted(exp)));
        String line = reader.readLine();
        String[] splitLine = line.split(";");
        int genotypeIndex = -1;
        int firstParamIndex = -1;
        while (!splitLine[++genotypeIndex].contains("genotype")) {}
        while (!splitLine[++firstParamIndex].contains("bodySizeSigma")) {}
        Map<String, List<List<Double>>> genotypes = new HashMap<>();
        while (Objects.nonNull(line = reader.readLine())) {
            splitLine = line.split(";");
            final String key = Arrays.stream(splitLine).toList().subList(firstParamIndex, firstParamIndex + 4).stream().reduce("%s;%s"::formatted).orElseThrow();
            if (!genotypes.containsKey(key)) {
                genotypes.put(key, new ArrayList<>());
            }
            genotypes.get(key).add((List<Double>) base64Deserializer(splitLine[genotypeIndex]));
        }
        TaskVideoBuilder<Supplier<EmbodiedAgent>> taskVideoBuilder =
                (TaskVideoBuilder<Supplier<EmbodiedAgent>>) nb.build("sim.taskVideoBuilder(task = s.task.standPiling(duration = 20; nOfAgents = 8))");
        for (Map.Entry<String, List<List<Double>>> params : genotypes.entrySet()) {
            System.out.println(params.getKey());
            final String[] paramsSplit = params.getKey().split(";");
            final Double bodySizeSigma = Double.valueOf(paramsSplit[0]);
            final Double sensorDistanceSigma = Double.valueOf(paramsSplit[1]);
            final Double sideContractionSigma = Double.valueOf(paramsSplit[2]);
            final Double parametersSigma = Double.valueOf(paramsSplit[3]);
            final Function<List<Double>, Supplier<EmbodiedAgent>> mapper = ((InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(
                    String.format(Locale.US,
                            "er.m.noisedDsToNIV(" +
                                    "  sensors = [" +
                                    "    s.sensors.sin(); s.sensors.a(); s.sensors.ar(); s.sensors.rv(a = 0); s.sensors.rv(a = 90);" +
                                    "    s.sensors.d(a = 0; r = 5); s.sensors.d(a = 45; r = 5); s.sensors.d(a = 90; r = 5); s.sensors.d(a = 135; r = 5);" +
                                    "    s.sensors.d(a = 180; r = 5); s.sensors.d(a = 225; r = 5); s.sensors.d(a = 270; r = 5); s.sensors.d(a = 315; r = 5);" +
                                    "    s.sensors.sc(s = N); s.sensors.sc(s = E); s.sensors.sc(s = S); s.sensors.sc(s = W);" +
                                    "    s.sensors.sa(s = N); s.sensors.sa(s = E); s.sensors.sa(s = S); s.sensors.sa(s = W);" +
                                    "    s.sensors.c()" +
                                    "  ];" +
                                    "  function = ds.num.stepped(" +
                                    "    stepT = 0.2;" +
                                    "    inner = ds.num.mlp(" +
                                    "      nOfInnerLayers = 1;" +
                                    "      innerLayerRatio = 1" +
                                    "    )" +
                                    "  );" +
                                    "  bodySizeSigma = %.3f;" +
                                    "  sensorDistanceSigma = %.3f;" +
                                    "  sideContractionSigma = %.3f;" +
                                    "  parametersSigma = %.3f" +
                                    ")", bodySizeSigma, sensorDistanceSigma, sideContractionSigma, parametersSigma)
            )).mapperFor(null);
            int counter = -1;
            for (List<Double> l : params.getValue()) {
                taskVideoBuilder.save(new VideoBuilder.VideoInfo(800, 600, VideoUtils.EncoderFacility.DEFAULT),
                        new File(path + "Videos/%s-%s-video-%d.mp4".formatted(exp, params.getKey(), ++counter)),
                        mapper.apply(l));
            }
        }
    }

    public static void noiseVideos(String exp) throws IOException {
        if (exp.contains("piling")) {
            noisePilingVideos(exp);
            return;
        }
        final String task;
        final String taskName;
        if (exp.contains("locomotion")) {
            task = "s.task.prebuiltIndependentLocomotion(duration = 30; shape = s.a.vsr.shape.free(s = \"sss-sss\"))";
            taskName = "locomotion";
        } else if (exp.contains("holed-anch")) {
            task = "s.task.prebuiltIndependentLocomotion(terrain = s.terrain.holed(startW = 20; holeWs = [2.05]); terrainAttachableDistance = 0.0; shape = s.a.vsr.shape.free(s = \"ssssssssss\"))";
            taskName = "holed-anch";
        } else {
            task = "";
            taskName = "";
        }
        final BufferedReader reader = new BufferedReader(new FileReader(
                path + "Csv/%s/%s-finals.csv".formatted(
                        taskName.substring(0, 1).toUpperCase() + taskName.substring(1), exp
                )));
        String line = reader.readLine();
        String[] splitLine = line.split(";");
        int genotypeIndex = -1;
        int firstParamIndex = -1;
        while (!splitLine[++genotypeIndex].contains("genotype")) {}
        while (!splitLine[++firstParamIndex].contains("bodySizeSigma")) {}
        Map<String, List<List<Double>>> genotypes = new HashMap<>();
        while (Objects.nonNull(line = reader.readLine())) {
            splitLine = line.split(";");
            final String key = Arrays.stream(splitLine).toList().subList(firstParamIndex, firstParamIndex + 4).stream().reduce("%s;%s"::formatted).orElseThrow();
            if (!genotypes.containsKey(key)) {
                genotypes.put(key, new ArrayList<>());
            }
            genotypes.get(key).add((List<Double>) base64Deserializer(splitLine[genotypeIndex]));
        }
        TaskVideoBuilder<Supplier<NumIndependentVoxel>> taskVideoBuilder =
                (TaskVideoBuilder<Supplier<NumIndependentVoxel>>) nb.build("sim.taskVideoBuilder(task = %s)".formatted(task));
        for (Map.Entry<String, List<List<Double>>> params : genotypes.entrySet()) {
            System.out.println(params.getKey());
            final String[] paramsSplit = params.getKey().split(";");
            final Double bodySizeSigma = Double.valueOf(paramsSplit[0]);
            final Double sensorDistanceSigma = Double.valueOf(paramsSplit[1]);
            final Double sideContractionSigma = Double.valueOf(paramsSplit[2]);
            final Double parametersSigma = Double.valueOf(paramsSplit[3]);
            final Function<List<Double>, Supplier<NumIndependentVoxel>> mapper = ((InvertibleMapper<List<Double>, Supplier<NumIndependentVoxel>>) nb.build(
                    String.format(Locale.US,
                            "er.m.noisedDsToNIV(" +
                            "  sensors = [" +
                            "    s.sensors.sin(); s.sensors.a(); s.sensors.ar(); s.sensors.rv(a = 0); s.sensors.rv(a = 90);" +
                            "    s.sensors.d(a = 0; r = 5); s.sensors.d(a = 45; r = 5); s.sensors.d(a = 90; r = 5); s.sensors.d(a = 135; r = 5);" +
                            "    s.sensors.d(a = 180; r = 5); s.sensors.d(a = 225; r = 5); s.sensors.d(a = 270; r = 5); s.sensors.d(a = 315; r = 5);" +
                            "    s.sensors.sc(s = N); s.sensors.sc(s = E); s.sensors.sc(s = S); s.sensors.sc(s = W);" +
                            "    s.sensors.sa(s = N); s.sensors.sa(s = E); s.sensors.sa(s = S); s.sensors.sa(s = W);" +
                            "    s.sensors.c()" +
                            "  ];" +
                            "  function = ds.num.stepped(" +
                            "    stepT = 0.2;" +
                            "    inner = ds.num.mlp(" +
                            "      nOfInnerLayers = 1;" +
                            "      innerLayerRatio = 1" +
                            "    )" +
                            "  );" +
                            "  bodySizeSigma = %.3f;" +
                            "  sensorDistanceSigma = %.3f;" +
                            "  sideContractionSigma = %.3f;" +
                            "  parametersSigma = %.3f" +
                            ")", bodySizeSigma, sensorDistanceSigma, sideContractionSigma, parametersSigma)
            )).mapperFor(null);
            int counter = -1;
            for (List<Double> l : params.getValue()) {
                taskVideoBuilder.save(new VideoBuilder.VideoInfo(800, 600, VideoUtils.EncoderFacility.DEFAULT),
                        new File(path + "Videos/%s-%s-video-%d.mp4".formatted(exp, params.getKey(), ++counter)),
                        mapper.apply(l));
            }
        }
    }
}
