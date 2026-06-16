# KorSim

![Alt text](/sim.png)

A silly 6-DOF real-time aircraft flight simulator i built in a couple of afternoons as a side project
Designed for high traceability and straighforward integration with control loop development, leveraging the speed of C++ for dynamic simulation and Godot for visualization and fast iteration cycles

> [!NOTE]
> Notice that the project is minimalistic and still in development. Some features are missing. Many, in fact. Also, Right now there are little quicks such as using Forward Euler instead of something better like Runge Kutta, that are artifacts of the fact that i did this in literally 3 days.

## Requirements

- Godot Engine 4.6 (or forward, shouldn't cause issues if you use the lastest 4.7 release)
- SCons (4.5+ ) & Dependencies
- `eigen3` cause eigen is goated
- A C++ compiler that supports C++17

## Building and Running

The C++ GDextension must be built beforehand.

```bash
git clone https://github.com/colmenasio/KorSim --depth=1
cd KorSim/engine
scons #buldscript automatically exports the .so to sim/bin
```

Note that for now only linux is supported.

``bash
cd ../sim
/path/to/your/godot/install/Godot.4.6 -p .
``
If everything went right, the project loads the Godot loads the extension without exploding, pressing "F5" or clicking the play icon on the top right of the editor should launch you straight into the simulation
