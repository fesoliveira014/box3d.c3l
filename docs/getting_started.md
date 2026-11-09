# Getting started

[Documentation](index.md) › Getting started

Install the `b3` library, then build one program: a ball dropped onto a static box, stepped at
60 Hz for two seconds, with its height printed and its first contact reported. The program is
walked through in sections; the complete source is at the [end](#complete-program).

## Prerequisites

- `c3c --version` reports 0.8.5.
- A `linux-x64` or `windows-x64` host. These are the only two targets `b3` ships.
- Building from a clone also needs Git with submodule support, CMake and the platform C compiler:
  GCC on Linux, MSVC on Windows.

## Install

Lay the application out like this:

```text
hello_box3d/
├── libs/
│   └── b3.c3l        the release file, or a link to a clone
├── src/
│   └── main.c3
└── project.json
```

### From a release

Download `b3.c3l` from the
[latest release](https://github.com/fesoliveira014/box3d.c3l/releases/latest) and put it in
`libs/`. The file is a packed `.c3l`: a zip archive that c3c reads as it stands. It carries the C3
sources and a prebuilt box3d static library for both targets, so a consumer needs neither CMake nor
a C compiler.

Do not unzip or rename it. c3c unpacks it into the build directory itself, and `c3c project fetch`
looks for the dependency as `<search-path>/b3.c3l`.

### From a clone

Clone the repository, fetch the box3d submodule, and build the native library for the host:

```sh
git clone https://github.com/fesoliveira014/box3d.c3l.git
cd box3d.c3l
git submodule update --init
./scripts/build-box3d.sh
```

The script writes `linked-libs/linux-x64/libbox3d.a` on Linux and
`linked-libs/windows-x64/box3d.lib` on Windows. On Windows, run it from a shell such as Git Bash
inside a Visual Studio developer environment so CMake finds MSVC. There is no cross-compilation:
each target is built on its own platform.

Then make the clone visible to the application as `libs/b3.c3l`. A symbolic link is enough:

```sh
ln -s /path/to/box3d.c3l hello_box3d/libs/b3.c3l
```

The build resolves the package by its manifest's `provides` name, `b3`, whatever the entry is
called, but `c3c project fetch` looks for `<search-path>/<dependency>.c3l`, so name the link
`b3.c3l`. The repository's own test project does the same with `test/libs/b3.c3l`.

### project.json

```json
{
  "langrev": "1",
  "dependency-search-paths": [ "libs" ],
  "dependencies": [ "b3" ],
  "output": "build",
  "wincrt": "static",
  "targets": {
    "hello_box3d": {
      "type": "executable",
      "sources": [ "src/main.c3" ]
    }
  }
}
```

The dependency is `b3`, the manifest's `provides` name, not the repository name `box3d.c3l`.

Keep `"wincrt": "static"` for Windows. box3d builds against the static MSVC runtime, and the
library's manifest declares the same for `windows-x64`. A project built against the dynamic runtime
does not link, and the errors are duplicate symbols that do not name the cause. Linux ignores the
key. `windows-x64` is the MSVC target; c3c's `mingw-x64` is a different ABI and is not shipped.

## The program

### Check the ABI

```c3
b3::require_float_abi()!;
```

The bindings are declared against box3d's single-precision layout. A box3d built with
`BOX3D_DOUBLE_PRECISION` widens every position and would corrupt every call. `require_float_abi`
refuses such a library with `DOUBLE_PRECISION_LIBRARY`. The shipped archives are always single
precision, so this is a cheap guard for a library built some other way.

### World

```c3
b3::WorldDef world_def = b3::default_world_def();
world_def.gravity = { 0.0f, -10.0f, 0.0f };
b3::WorldId world = b3::create_world(&world_def)!;
defer b3::destroy_world(world);
```

Every definition starts from its `default_*_def()` function. The defaults carry a marker that
`create_world` checks; a zeroed or hand-built `WorldDef` is refused with `INVALID_DEFINITION`. Edit
fields on the returned value, then create.

`create_world` returns a `WorldId`, a small value struct. Destroying the world destroys every body,
shape and joint in it, so the one `defer` is the only cleanup the program needs.

### Ground

```c3
b3::BodyDef ground_def = b3::default_body_def();
ground_def.position = { 0.0f, -1.0f, 0.0f };
ground_def.name = "ground";
b3::BodyId ground = b3::create_body(world, &ground_def)!;

b3::BoxHull ground_box = b3::make_box_hull(50.0f, 1.0f, 50.0f);
b3::ShapeDef ground_shape_def = b3::default_shape_def();
b3::create_hull_shape(ground, &ground_shape_def, &ground_box.base)!;
```

A default body is static. The ground is a box 100 x 2 x 100 centred one unit below the origin, so
its top face is at `y = 0`. Bodies carry no geometry of their own: shapes are attached afterwards.

A box is a convex hull. `make_box_hull` returns a `BoxHull` by value with its storage inside it,
and `create_hull_shape` copies the hull, so the stack value can go out of scope after the call.

### Ball

```c3
b3::BodyDef ball_def = b3::default_body_def();
ball_def.type = b3::BodyType.DYNAMIC;
ball_def.position = { 0.0f, 4.0f, 0.0f };
ball_def.name = "ball";
b3::BodyId ball = b3::create_body(world, &ball_def)!;

b3::ShapeDef ball_shape_def = b3::default_shape_def();
ball_shape_def.enable_contact_events = true;
b3::Sphere sphere = { .center = { 0.0f, 0.0f, 0.0f }, .radius = 0.5f };
b3::create_sphere_shape(ball, &ball_shape_def, &sphere)!;
```

The sphere is in body-local space. Its mass comes from the shape density, which defaults to that of
water. Contact events are off by default; one of the two touching shapes asking for them is enough,
so only the ball sets the flag.

### Step

```c3
for (int step = 0; step < STEP_COUNT; step++) {
    b3::world_step(world, TIME_STEP, SUB_STEPS);

    if (step % 20 == 19) {
        b3::Pos p = ball.position();
        io::printfn("t = %.2f s  y = %.3f  awake = %s", (float)(step + 1) * TIME_STEP, p.y, ball.is_awake());
    }
}
```

`world_step` advances by a fixed time step, here 1/60 s, split into four sub-steps. Keep the step
fixed; the [cookbook](cookbook.md#fixed-timestep-loop) shows the accumulator loop for a variable
frame rate. The events section below goes right after the `world_step` call.

`ball.position()` is a method on the `BodyId`. Getters drop C's `Get` (`b3Body_GetPosition` is
`position()`); setters keep `set_`. The [architecture](architecture.md#names) page gives the full
naming rule.

### Contact events

```c3
foreach (event : world.contact_begin_touch_events()) {
    ZString a = event.shape_id_a.body().name();
    ZString b = event.shape_id_b.body().name();
    io::printfn("step %d: %s touched %s", step, a, b);
}
```

Read events after each step. The slice borrows an array the world owns: the next `world_step`
replaces it and `destroy_world` frees it. Copy out anything needed later. End-touch, hit and sensor
events work the same way; see [contact events](cookbook.md#contact-events).

### Run

```sh
c3c run hello_box3d
```

```text
t = 0.33 s  y = 3.438  awake = true
t = 0.67 s  y = 1.764  awake = true
step 50: ground touched ball
t = 1.00 s  y = 0.500  awake = true
t = 1.33 s  y = 0.500  awake = true
t = 1.67 s  y = 0.500  awake = false
t = 2.00 s  y = 0.500  awake = false
```

The ball lands at `y = 0.5`, its radius above the ground, and falls asleep once it stops moving.

## Complete program

```c3
module hello_box3d;

import std::io;
import b3;

const float TIME_STEP = 1.0f / 60.0f;
const int SUB_STEPS = 4;
const int STEP_COUNT = 120;

fn void? simulate() {
    b3::require_float_abi()!;

    b3::WorldDef world_def = b3::default_world_def();
    world_def.gravity = { 0.0f, -10.0f, 0.0f };
    b3::WorldId world = b3::create_world(&world_def)!;
    defer b3::destroy_world(world);

    b3::BodyDef ground_def = b3::default_body_def();
    ground_def.position = { 0.0f, -1.0f, 0.0f };
    ground_def.name = "ground";
    b3::BodyId ground = b3::create_body(world, &ground_def)!;

    b3::BoxHull ground_box = b3::make_box_hull(50.0f, 1.0f, 50.0f);
    b3::ShapeDef ground_shape_def = b3::default_shape_def();
    b3::create_hull_shape(ground, &ground_shape_def, &ground_box.base)!;

    b3::BodyDef ball_def = b3::default_body_def();
    ball_def.type = b3::BodyType.DYNAMIC;
    ball_def.position = { 0.0f, 4.0f, 0.0f };
    ball_def.name = "ball";
    b3::BodyId ball = b3::create_body(world, &ball_def)!;

    b3::ShapeDef ball_shape_def = b3::default_shape_def();
    ball_shape_def.enable_contact_events = true;
    b3::Sphere sphere = { .center = { 0.0f, 0.0f, 0.0f }, .radius = 0.5f };
    b3::create_sphere_shape(ball, &ball_shape_def, &sphere)!;

    for (int step = 0; step < STEP_COUNT; step++) {
        b3::world_step(world, TIME_STEP, SUB_STEPS);

        foreach (event : world.contact_begin_touch_events()) {
            ZString a = event.shape_id_a.body().name();
            ZString b = event.shape_id_b.body().name();
            io::printfn("step %d: %s touched %s", step, a, b);
        }

        if (step % 20 == 19) {
            b3::Pos p = ball.position();
            io::printfn("t = %.2f s  y = %.3f  awake = %s", (float)(step + 1) * TIME_STEP, p.y, ball.is_awake());
        }
    }
}

fn int main() {
    if (catch err = simulate()) {
        io::eprintfn("hello_box3d: %s", err);
        return 1;
    }
    return 0;
}
```

## Next

- [Architecture](architecture.md): modules, naming, identifiers, ownership, faults, threading.
- [Cookbook](cookbook.md): queries, joints, meshes, the character mover, debug draw, recording.
- [Features and limitations](features_and_limitations.md): what is bound, targets, fixed limits.
- [API reference](api/index.md): every public symbol by domain.
