# Nodi: learn ROS 2 by playing

A game-like ROS 2 course that runs entirely in the browser, in a single web page. It has animated lessons, quick practice rounds, a 2D Robot Lab with a working `ros2` terminal and Python nodes, and **SnoBots Robots**: 3D physics simulators of four real robots.

**Try it:** https://pathinvyas.github.io/nodi-ros2/

| | |
|---|---|
| ![The learning path](docs/screenshots/learn.jpg) | ![An animated lesson scene](docs/screenshots/lesson.jpg) |

## What's inside

| | |
|---|---|
| **Learn** | 12 units, 26 lessons and 153 exercises on ROS 2 Jazzy: the command line, publishers and subscribers, services, custom interfaces, parameters, launch files and TF2, in Python and C++, ending with a driver node for a microcontroller. Lessons are short animated scenes followed by a mix of exercise types (multiple choice, fill in the code, spot the bug, order the steps, drive the turtle). |
| **Robot Lab** | A 2D robot with a live ROS 2 graph. Use `ros2 topic echo`, `ros2 param set` and friends in the terminal, or write Python nodes with an rclpy-style API. Python runs in your browser through Pyodide. |
| **SnoBots Robots** | Physics simulators of a Hiwonder AiNex (24-joint small humanoid), a Unitree R1 (humanoid), an AgileX LIMO (four wheel modes) and a LIMO Cobot (LIMO with a 6-axis arm). Each has missions, a terminal, a Python code tab and a live topic graph. |
| **Library** | Books and guides to read inside the app. It has a PDF reader with a contents panel, zoom, selectable text and a saved place in every book. Readers can also open their own PDFs, which stay in their browser. |
| **Practice, Notebook, Profile** | Review rounds, unit notes, XP, streaks and badges. Progress is saved in the browser. |

![AiNex walking with contact forces and centre of mass shown](docs/screenshots/ainex-physics.jpg)

| | |
|---|---|
| ![Unitree R1 in the Isaac-style scene](docs/screenshots/r1-isaac.jpg) | ![LIMO Cobot holding a cube](docs/screenshots/cobot-pick.jpg) |
| ![LIMO driving to a navigation goal](docs/screenshots/limo-nav.jpg) | ![The Robot Lab](docs/screenshots/robot-lab.jpg) |

## The physics

The simulators run on a small rigid-body engine written for this project, in the spirit of MuJoCo (it is not MuJoCo itself):

- **Articulated dynamics.** Featherstone's recursive Newton–Euler algorithm gives the bias forces and the composite-rigid-body algorithm gives the mass matrix. Every link has mass and inertia.
- **Motors.** Joints are driven by implicit PD servos with torque and speed limits, rotor armature, damping and friction, so a robot can sag, stall or fall.
- **Contacts.** Boxes, spheres, capsules and wheels touch through soft contacts with Coulomb friction and rolling resistance. They are solved with warm-started projected Gauss–Seidel. Mecanum rollers use anisotropic friction.
- **Walking.** The humanoids use a model-based controller: a footstep planner, a linear-inverted-pendulum / DCM centre-of-mass plan, capture-point stepping to catch pushes, and leg inverse kinematics. A virtual hoist stands them up after a fall.
- **Wheels and sensors.** LIMO's `/odom` is integrated from the wheel encoders, so it drifts when the tyres slip, and SLAM or navigation publish the `map → odom` correction. The IMU and odometry have noise.
- **Tools you'd expect from a physics sim.** Pause and single-step, slow motion, Ctrl+drag to push a robot or an object, overlays for contact forces, centre of mass and collision shapes, domain randomization (friction, mass, payload, motor strength, sensor noise), and MuJoCo-, Isaac- and classic-style scenes.
- **Simulated time.** Python nodes in the simulators run on the simulator's clock (`use_sim_time` is true), and message stamps carry simulated seconds, so pausing the sim pauses your timers too.

Honest limits: the real Unitree R1 walks with a learned (reinforcement-learning) policy, not this controller, and the topic names are simplified for teaching. Always follow the vendor documentation before running code on real hardware.

## The Library

The Library shelf is defined in `src/library.js`. A book can be in one of two forms:

- **Shipped with the app.** The PDF lives in `library/` and is published with the site. Only add PDFs you have the right to share publicly, for example your own guides or openly licensed books.
- **Bring your own copy** (`own: true`). This is for copyrighted books. The shelf shows the title and a link to the publisher, and each reader adds their own PDF from their device. It's kept in that browser's storage (IndexedDB) and never uploaded.

To add a shipped book:

1. Copy the PDF into `library/`.
2. Add an entry to `LIBRARY` in `src/library.js`.
3. Optionally, add a cover image in `library/covers/` and refer to it as `'LIBCOVER:<file>'`.

The build inlines covers and copies the PDFs next to `index.html`. `npm test` checks that every listed file exists and is a PDF.

The reader uses [PDF.js](https://mozilla.github.io/pdf.js/), loaded from a CDN when you first open a book.

## Run it locally

You need Python 3 to build. The page loads three.js and Pyodide from public CDNs, so the 3D view and Python need an internet connection.

```bash
python3 build.py                        # inlines src/ into dist/index.html
python3 -m http.server -d dist 8000     # then open http://localhost:8000
```

## Tests

The tests run the real simulator code in Node.js (version 18 or newer), without a browser:

```bash
npm test
```

- `tests/content.test.js` checks the course data: every lesson scene and every exercise answer key.
- `tests/physics.test.js` checks the engine against textbook answers: resting contact, Coulomb sliding distance, rolling without slipping, free fall of an articulated body, pendulum period.
- `tests/library.test.js` checks the Library list: shipped books are real PDFs with their covers, and bring-your-own-copy books ship no file.
- `tests/robots.test.js` drives the four robots:
  - AiNex stands, walks, turns, kicks, catches a push and is stood back up after a hard one
  - R1 walks at 0.4 m/s, collapses in damping mode and holds a joint in low-level PD mode
  - LIMO drives in all four wheel modes, with odometry that slips against a wall
  - LIMO navigates to a goal
  - the Cobot picks and sorts all three cubes

## Project layout

```
build.py            one-file build: src/ -> dist/index.html
src/
  core.js           app state, storage, UI helpers, icons
  anim.js           lesson animations
  content1.js       course content: units, lessons, exercises, notes
  content2.js
  lesson.js         lesson player and exercise types
  lab.js            Robot Lab: 2D robot, ROS 2 graph, terminal, Python runner
  pyworker.js       Pyodide web worker
  pyshim.py         rclpy-style API for the browser
  sno_kin.js        kinematic trees, forward and inverse kinematics
  sno_phys.js       the rigid-body physics engine
  sno_walk.js       biped walking controller (LIPM / DCM)
  sno_core.js       SnoBots 3D view, panels, physics tools
  sno_humanoid.js   humanoid models and simulator
  sno_robots.js     AiNex and R1 specifics
  sno_car.js        LIMO and LIMO Cobot
  sno_defs.js       robot definitions, missions, code templates
  library.js        Library shelf and PDF reader
  app.js            pages and routing
  styles.css        app styles
  sno.css           SnoBots styles
  library.css       Library and reader styles
library/            PDFs published with the app, and their covers
tests/              Node tests (see above)
docs/screenshots/   images for this README
```

## Notes

- The **Ask Nodi** tutor and cloud-synced progress use features of the Claude artifact viewer. Hosted anywhere else, such as GitHub Pages, the tutor is hidden and progress stays in the browser.
- The SnoBots simulators are unofficial educational simulators, made for learning. They are not affiliated with or endorsed by Hiwonder, Unitree, AgileX or Elephant Robotics. Robot names belong to their owners. Models are simplified and behaviour is approximate.
- Third-party code loaded at runtime: [three.js](https://threejs.org) (MIT), [Pyodide](https://pyodide.org) (MPL-2.0), [PDF.js](https://mozilla.github.io/pdf.js/) (Apache-2.0), and fonts from Google Fonts (SIL Open Font License).
- *Patty's Guide to GitHub* in `library/` is by Pathin and is published with the app.

## License

MIT. See [LICENSE](LICENSE).
