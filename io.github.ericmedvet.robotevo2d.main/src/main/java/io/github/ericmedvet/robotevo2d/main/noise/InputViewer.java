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
import java.awt.image.BufferStrategy;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import javax.swing.*;

public class InputViewer extends JFrame implements Consumer<Snapshot> {
  private static final long WAIT_MILLIS = 10;
  private static final int INIT_WIN_WIDTH = 1800;
  private static final int INIT_WIN_HEIGHT = 900;

  private final Drawer drawer;

  private final Canvas canvas;
  private final List<Snapshot> snapshots;
  private Instant startingInstant;
  private boolean advance;

  public InputViewer(Drawer drawer) {
    super("Realtime simulation viewer");
    this.drawer = drawer;
    // create/set ui components
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    Dimension dimension = new Dimension(INIT_WIN_WIDTH, INIT_WIN_HEIGHT);
    canvas = new Canvas();
    canvas.setPreferredSize(dimension);
    canvas.setMinimumSize(dimension);
    canvas.setMaximumSize(dimension);
    getContentPane().add(canvas, BorderLayout.CENTER);
    JButton advanceButton = new JButton();
    advanceButton.setText("Advance");
    advance = true;
    advanceButton.addActionListener(e -> advance = true);
    getContentPane().add(advanceButton, BorderLayout.PAGE_END);
    // pack
    pack();
    // start
    setVisible(true);
    canvas.setIgnoreRepaint(true);
    canvas.createBufferStrategy(2);
    snapshots = new ArrayList<>();
  }

  @SuppressWarnings("BusyWait")
  @Override
  public void accept(Snapshot snapshot) {
    if (startingInstant == null) {
      startingInstant = Instant.now();
    }
    snapshots.add(snapshot);
    // wait
    while (!advance) {
      try {
        Thread.sleep(WAIT_MILLIS);
      } catch (InterruptedException e) {
        throw new RuntimeException(e);
      }
    }
    advance = false;
    // get graphics
    Graphics2D g = (Graphics2D) canvas.getBufferStrategy().getDrawGraphics();
    g.setClip(0, 0, canvas.getWidth(), canvas.getHeight());
    // draw
    drawer.draw(snapshots, g);
    // dispose and encode
    g.dispose();
    BufferStrategy strategy = canvas.getBufferStrategy();
    if (!strategy.contentsLost()) {
      strategy.show();
    }
    Toolkit.getDefaultToolkit().sync();
    snapshots.clear();
  }
}