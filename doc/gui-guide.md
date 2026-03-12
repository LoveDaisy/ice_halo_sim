[中文版](gui-guide_zh.md)

# GUI Guide

This document describes the Lumice GUI application — an interactive graphical interface for configuring and previewing ice halo simulations.

## Building and Running

```bash
# Build the GUI application
./build.sh -gj release

# Run
./build/cmake_install/LumiceGUI
```

The GUI requires a display server and GPU with OpenGL 3.2 Core Profile support.

## UI Layout

The interface is divided into several areas:

- **Top Bar**: File operations (New, Open, Save, Save As) and simulation controls (Run, Stop, Revert)
- **Left Panel**: Configuration tabs (Crystal, Scene, Render, Filter)
- **Crystal Preview**: 3D wireframe/shaded preview of the current crystal (in the Crystal tab)
- **Render Preview**: Lens-projected halo simulation result (in the Render tab)
- **Floating Lens Bar**: Quick lens type selector (visible when Render tab is active)
- **Status Bar**: Current file path, simulation state, and dirty indicator

## Configuration Tabs

### Crystal Tab

Configure ice crystal geometry and orientation:

- **Crystal type**: `Prism` (hexagonal prism) or `Pyramid` (hexagonal pyramid)
- **Shape parameters**: Height ratio, face distances, and pyramid-specific parameters (upper/lower heights, Miller indices)
- **Axis distributions**: Zenith, roll, and azimuth — each can be fixed, uniform, or Gaussian
- **Multiple crystals**: Add/delete crystals with confirmation popup
- **3D Preview**: Interactive trackball rotation and zoom; supports 4 render styles:
  - **Wireframe**: Edges only
  - **Hidden Line**: Edges with hidden-line removal (default)
  - **X-Ray**: Transparent with visible internal structure
  - **Shaded**: Solid shading with normal-based coloring

### Scene Tab

Configure simulation parameters:

- **Sun position**: Altitude and azimuth (degrees)
- **Light spectrum**: Standard illuminant (e.g., D65) or custom wavelength-weight pairs
- **Ray count**: Number of rays to trace
- **Max hits**: Maximum crystal interactions per ray
- **Scattering layers**: Define multi-scattering scenarios with crystal proportions and optional filters

### Render Tab

Configure render output:

- **Lens type**: 8 supported types — Linear, Fisheye Equal Area, Fisheye Equidistant, Fisheye Stereographic, Dual Fisheye (3 variants), Rectangular
- **Field of view**: Adjustable FOV in degrees
- **View angles**: Elevation, azimuth, and roll
- **Resolution**: Output image dimensions
- **Colors**: Background color, ray color, and opacity
- **Multiple renderers**: Add/delete render configurations

### Filter Tab

Configure ray path filters:

- **Raypath filter**: Specify face sequence with symmetry options (P/B/D)
- **Entry/exit filter**: Filter by entry and exit faces
- **Multiple filters**: Add/delete filter configurations

## File Operations

### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+N | New project |
| Ctrl+O | Open project |
| Ctrl+S | Save project |
| Ctrl+Shift+S | Save As |
| Ctrl+Enter | Run simulation |
| Ctrl+K | Stop simulation |

### Project File Format (`.lmc`)

Lumice uses a binary project file format (`.lmc`) that stores:

- **Configuration**: All crystal, scene, render, and filter settings as semantic JSON
- **Preview texture**: Optional embedded PNG of the last render result

The format uses a 44-byte header with magic number `LMC\0`, version field, and offset/size pointers to the JSON and texture payloads. Values are stored as human-readable semantic types (e.g., `"prism"` instead of enum indices) for forward compatibility.

### Unsaved Changes

When you have unsaved changes (indicated by the dirty flag in the status bar), the application warns before:
- Creating a new project
- Opening another project
- Closing the application

## Simulation Workflow

1. **Configure**: Set up crystals, scene, renders, and filters in the left panel
2. **Run** (Ctrl+Enter): The current configuration is serialized and submitted to the simulation core
3. **Monitor**: The status bar shows simulation progress; parameters are disabled during simulation
4. **View**: Results appear in the Render preview when available
5. **Stop** (Ctrl+K): Halt the simulation early if needed
6. **Revert**: Restore the last committed configuration (undo parameter changes made after starting)

### Simulation States

- **Idle**: No simulation running; all parameters editable
- **Simulating**: Simulation in progress; parameters disabled
- **Done**: Simulation complete; results available in preview
- **Modified**: Parameters changed after simulation completed

## Related Documentation

- [Configuration Guide](configuration.md) - Detailed configuration reference (JSON format)
- [Architecture Document](architecture.md) - System architecture and GUI module design
- [Developer Guide](developer-guide.md) - GUI testing and development
