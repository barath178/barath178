# SolidWorks 2025 package

SolidWorks .SLDPRT and .SLDASM files can only be written by SolidWorks itself. This folder has everything needed to create them on your PC. Choose **one** of the two methods below. Both give you native part files and an assembly file.

| File | What it is |
|---|---|
| `parts/*.step` | 35 part files, one per unique part (mm, AP214, with colours). Printed parts have plain names; bought parts start with `HW`. |
| `Myoelectric Hand Assembly.step` | The full assembly: 60 components that reference those 35 parts. The 4 fingers are one part used 4 times. |
| `instances.csv` | Position and orientation of each of the 60 components. The macro reads it. |
| `BuildSolidWorksAssembly.bas` | SolidWorks VBA macro for method B. |

## Method A: open the assembly STEP (no macro)

1. In SolidWorks 2025, go to **File → Open** and set the file type to *STEP AP203/214/242 (\*.step;\*.stp)*. Click **Options…** and make sure *Import multiple bodies as parts* is selected, so the file opens as an assembly.
2. Open `Myoelectric Hand Assembly.step`. If you're asked whether to run Import Diagnostics, answer **No**.
3. Go to **File → Save As → Assembly (\*.sldasm)**. When asked about the components, choose **Save all as external files** (or *Save internally* if you prefer a single file). This gives you `Myoelectric Hand Assembly.SLDASM` and 35 `.SLDPRT` files.

## Method B: run the macro (one .SLDPRT per STEP, plus an assembly with every component fixed in place)

1. Go to **Tools → Macro → New…** and save an empty macro, for example `hand.swp`.
2. In the VBA editor that opens, choose **File → Import File…** and select `BuildSolidWorksAssembly.bas`. Then run **Main** (F5).
3. When the folder picker appears, select **this `solidworks` folder**.
4. The macro converts the 35 STEP files into `native_sldworks\*.SLDPRT`. It then builds `native_sldworks\Myoelectric Hand Assembly.SLDASM`, placing all 60 components and fixing each one.

## Notes

- All parts share one coordinate system, and each part sits in its assembly position. That's why every component in the assembly has a zero offset, or a simple translation for repeated parts. The parts line up without any mates.
- Holes, outlines and thicknesses are exactly the same as in `../cad/`. I checked: after reading `Myoelectric Hand Assembly.step` back, all 60 instances match the source parts vertex for vertex (`../source/verify_sw_assembly.py`).
- The imported parts are "dumb" solids with no feature history. You can still add features, cut holes or add mates, and edit faces with *Move Face* or *Delete Face*.
- I couldn't run SolidWorks while preparing this package, so neither method has been tested in SolidWorks itself. Method A uses only standard STEP import, so it's the safer first try. If the macro reports an error, send me the message.
