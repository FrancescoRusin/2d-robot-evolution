package io.github.ericmedvet.robotevo2d.main.noise;

import io.github.ericmedvet.mrsim2d.core.Snapshot;
import io.github.ericmedvet.mrsim2d.viewer.Drawer;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.Consumer;

public class SingleFrames implements Consumer<Snapshot> {
    private final Drawer drawer;
    private final String path;
    private final long[] toAccept;
    private final int w;
    private final int h;
    private long counter;
    private int index;

    public SingleFrames(Drawer drawer, String path, long[] toAccept, int w, int h) {
        this.drawer = drawer;
        this.path = path;
        this.toAccept = new long[toAccept.length];
        this.w = w;
        this.h = h;
        System.arraycopy(toAccept, 0, this.toAccept, 0, toAccept.length);
        counter = 0;
        index = 0;
    }
    @Override
    public void accept(Snapshot snapshot) {
        if (counter == toAccept[index]) {
            ++index;
            BufferedImage image = new BufferedImage(w, h, BufferedImage.TYPE_3BYTE_BGR);
            Graphics2D g = image.createGraphics();
            g.setClip(0, 0, image.getWidth(), image.getHeight());
            drawer.draw(List.of(snapshot), g);
            try {
                ImageIO.write(image, "png", new File(path + "-%d.png".formatted(counter)));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        ++counter;
    }
}
