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
package io.github.ericmedvet.robotevo2d.main.noise;

import io.github.ericmedvet.mrsim2d.core.Snapshot;
import io.github.ericmedvet.mrsim2d.viewer.Drawer;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.Consumer;
import javax.imageio.ImageIO;

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
    if (index < toAccept.length) {
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
}
