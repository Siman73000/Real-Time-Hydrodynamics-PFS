
# Real-Time Particle Fluid Simulation for Hydrodynamics

PFSs are used to understand how certain unstable incompressible fluids are effected by the world around us.
This project was engineered on a Linux kernel based OS, though, this will also work in an MinGW-w64 Windows terminal or other GCC based console.
See https://www.mingw-w64.org/ for more information.

To build this project perform the following commands in the build directory (with quotes omitted) within a terminal environment (GCC will be needed):

"sudo apt update && sudo apt install -y gcc libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libglfw3-dev"

"gcc -o main main.c -lGL -lGLU -lglut -lm -lglfw -O3 -ffast-math"


## Code Architecture

The vast majority of the program is built using pure C. For resolving gravity based collision calculations, inline x86_64 Assembly is used as it is faster than using Procedural C programming.

The famous Quake III fast inverse square root algorithm is used to efficiently compute square roots at blazingly fast speeds.

To optimize the computation of particle collisions, linear interpolation, 3D spacial partitioning, and verlet integration were implimented which allows for 10,000+ 3D fully shaded and interactive particles to be rendered at over 60 FPS.



## Keyboard Shortcuts to Interact with the PFS

Move forward:      "W"

Move backwards:    "S"

Look Left:         "Left Arrow Key"

Look Right:        "Right Arrow Key"

Look Up:           "Up Arrow Key"

Look Down:         "Down Arrow Key"

Restart Sim:       "Enter"

Open Menu:         "f"
