# RatBot python-tools

Python control software for the RatBot quadruped, based on the Q8bot python-tools.

## Quick Start

See [Software Setup](../docs/instructions/software_setup.md) for details on running from source.

To try the existing GUI and controls without a robot or ESP32C3, run from the
repository root with the Python dependencies installed:

```bash
python3 python-tools/ratbot/operate.py --demo
```

Demo mode displays **DEMO MODE - NO HARDWARE CONNECTED** in the status bar.
It skips serial detection and connection, even if a COM port is supplied.
Movement, gait switching, sitting and rearing use the existing control logic;
commands only update an in-memory controller. No battery telemetry or recorded
hardware data is produced. Omit `--demo` to use the normal hardware connection.

Hold **W/A/S/D** to command forward/left/backward/right movement. The demo shows
the active direction and gait (for example, **FORWARD — TROT**), or **IDLE** when
no gait movement is being commanded, including while a pose blocks movement.
The live **Joint-command frames** counter counts calls accepted by the mock,
including startup, walking, return-to-rest and pose commands; it is not a count
of rendered GUI frames.

In demo mode the central panel animates all four legs from the eight angles sent
to the mock controller, in FL/FR/BL/BR order. **Q/E** also animate the existing
forward-turn commands. Zero-duration gait commands are displayed directly; timed
sitting, rearing and lowering commands interpolate linearly between their angle
targets over the supplied duration. This is a visualization of commands, not
measured servo motion or a simulation of a motor's acceleration profile. On
release, the existing return-to-rest command determines the displayed pose.
Walking keeps the original fixed-chassis animation unchanged. Sitting and
rearing add an illustrative kinematic contact model: rear feet anchor the body
to the ground, with translation and a nose-up pitch derived from front/rear FK
foot positions. Sitting levels both pairs of feet (front contacts may slide);
rearing leaves the front feet free. Two rear contacts do not uniquely determine
body orientation, so this pitch is a visualization convention, not a prediction
of balance, stability, forces or physical body attitude. Contact is acquired and
released smoothly over the first 5 mm of front/rear foot-depth difference;
symmetric folded entry and the final symmetric rest use the original fixed-body
view. The normal hardware GUI retains its static illustrations.

**H** plays the complete greeting without blocking rendering or exit. The demo
records the same `greet()` commands and waits into a schedule, including all
three alternating waves and the return to rest. **J** replays the joint commands
from `firmware/ratbot_robot/src/q8Dynamixel.cpp::jump()` and the angle pairs in
its header: crouch `[-40, 220]` at 0.1 s (500 ms profile), extension `[90, 90]`
at 1.2 s (zero profile), retraction `[30, 150]` at 1.3 s (zero profile), then
idle `[30, 150]` at 6.4 s (500 ms profile). All pairs apply to all four legs.
The firmware sequence finishes at 7.4 s; the demo then returns to the selected
gait over 500 ms. Zero-profile targets display immediately because neither
servo feedback nor a timed trajectory is available. No airborne trajectory,
takeoff or landing dynamics is invented. The hardware GUI's original five-second
jump wait and subsequent command remain unchanged; the demo waits for the full
firmware schedule instead of guessing transport ordering during that routine.
Demo scripts remain exitable, block competing motion, and do not repeat while
H/J is held. No hardware trajectory, gain, timing or calibration is changed.

The demo renderer (`ratbot/kinematic_preview.py`) consumes a link graph from
`ratbot/leg_geometry.py`. Its Q8bot adapter uses the original 19.5 mm motor
spacing, 25 mm upper links, 40 mm lower links and existing solver equations.
For folded poses where the legacy FK starting estimate does not converge, the
adapter retries those same equations with an outward starting estimate and
checks link closure and assembly branch. A future mechanism can replace this
adapter's `solve(q1, q2)` method and returned `LegGeometry` graph without changing
the GUI or command handling.

With the Pygame window focused, press **P** to toggle a static sitting pose:
the front legs extend and the rear legs crouch over one second. Press **P**
again to return to the selected gait's resting position. Holding P does not
repeat the toggle. Walking and other motion routines are paused while sitting;
**B** (battery) and **Esc** (exit and release torque) still work.
The pose heights and transition duration are in `ratbot/sitting_pose.py`.
The sitting targets are 60 mm at the front and 20 mm at the rear. These are
foot distances from the leg mounts in the kinematic model.

Press **U** to enter and hold a static rearing position. It uses the greeting's
entry waypoints but holds both forelegs still in the final stance, without
waving. Press **U** again to lower back to the selected gait. Entry takes about
3.7 seconds; lowering takes about 6.4 seconds. Lowering first extends the front
legs to a 60 mm support target over 2 seconds, keeping the rear targets fixed.
After a 0.2 second pause, the rear legs move to the selected gait over 2.5 seconds
while the fronts stay extended. After another 0.2 second pause, all legs settle
into that gait's resting position over 1.5 seconds. These are timed transitions,
without ground-contact sensing. The GUI displays each phase and remains responsive during
both transitions. **B** and **Esc** work throughout. Return from a static pose
before entering the other one (**P** for sitting, **U** for rearing).

The rearward adjustment subtracts 13 degrees from both rear joint angles
of the greeting support stance, giving rear pairs `[37, 62]` and fixed front
pairs `[-45, 45]`. `REARWARD_ANGLE_OFFSET` in `ratbot/rearing_pose.py` is the tuning
parameter. Balance depends on payload mass/location and floor contact: the
geometry tests do not establish that a weighted robot can hold this pose.
Support the chassis during the first physical trial. No feedback balancing is
implemented; the preview's body angle is illustrative.

The GUI includes a picture for all eight gaits, sitting and rearing. The
picture follows **G**, **P** and **U**, and returns to the selected gait after lowering.
These are illustrations of the commanded geometry, not live measurements.
Regenerate the pictures after changing gait or sitting parameters:

```bash
venv/bin/python ratbot/pose_preview.py
```

Run the hardware-free geometry, input, image and GUI tests:

```bash
venv/bin/python -m unittest discover -s . -p 'test_*.py' -v
```

## Building a Single Executable from Source

Follow these steps to build a standalone executable that can run without Python installed:

### 1. Clone the Repository

```bash
git clone https://github.com/SamedVossberg/RatBot.git
cd RatBot/python-tools
```

### 2. Create a Virtual Environment (Recommended)

**Windows (Command Prompt):**
```cmd
python -m venv venv
venv\Scripts\activate
```

**Windows (PowerShell):**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**macOS/Linux:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

First, install the runtime dependencies:
```bash
pip install -r requirements.txt --force-reinstall
```

Then, install PyInstaller for building the executable:
```bash
pip install pyinstaller
```

The `--force-reinstall` flag ensures all packages are properly installed even if they already exist.

### 4. Build the Executable

Run the appropriate build script for your operating system:

**Windows (Command Prompt):**
```cmd
build_exe.bat
```

**Windows (PowerShell):**
```powershell
.\build_exe.ps1
```

**macOS/Linux:**
```bash
./build_exe.sh
```

The build process will:
- Check that PyInstaller is installed
- Clean previous builds
- Package the application with all dependencies
- Embed instruction images into the executable
- Create a single-file executable

### 5. Find the Executable

The built executable will be located in:

```
dist/
├── ratbot.exe         (Windows)
└── ratbot             (macOS/Linux)
```

### Running the Executable

**Auto-detect COM port:**
```bash
./dist/ratbot
```

**Specify COM port:**
```bash
./dist/ratbot COM3              # Windows
./dist/ratbot /dev/ttyUSB0      # Linux
```

**Enable debug logging:**
```bash
./dist/ratbot --debug
```

**Combine options:**
```bash
./dist/ratbot COM3 --debug
```

## Distribution

The executable in `dist/` is fully self-contained and can be:
- Copied to other computers without Python installed
- Run directly from USB drives
- Distributed to end users

**Note:** The executable is platform-specific (Windows .exe will not run on macOS/Linux and vice versa).
