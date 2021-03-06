(G-CODE GENERATED BY FLATCAM v8.993 - www.flatcam.org - Version Date: 2020/06/05)

(Name: Alignment Drills_2.4_5_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Wednesday, 24 March 2021 at 08:18)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)

(TOOL DIAMETER: 2.4 mm)
(Feedrate_XY: 120.0 mm/min)
(Feedrate_Z: 60.0 mm/min)
(Feedrate rapids 1500.0 mm/min)

(Z_Cut: -5.0 mm)
(Z_Move: 2.0 mm)
(Z Start: None mm)
(Z End: 15.0 mm)
(Steps per circle: 64)
(Preprocessor Geometry: default)

(X range:   25.0000 ...  175.0000  mm)
(Y range:    5.0000 ...  112.9049  mm)

(Spindle Speed: None RPM)
G21
G90
G94



G01 F120.00
G00 Z2.0000

M03
G00 X25.0000 Y5.0000
G01 F60.00
G01 Z-5.0000
G01 F120.00
G01 X25.0000 Y5.0000
G01 X25.0000 Y5.0000
G01 X25.0000 Y5.0000
G01 X25.0000 Y5.0000
G00 Z2.0000
G00 X100.0000 Y5.0000
G01 F60.00
G01 Z-5.0000
G01 F120.00
G01 X100.0000 Y5.0000
G01 X100.0000 Y5.0000
G01 X100.0000 Y5.0000
G01 X100.0000 Y5.0000
G00 Z2.0000
G00 X175.0000 Y5.0000
G01 F60.00
G01 Z-5.0000
G01 F120.00
G01 X175.0000 Y5.0000
G01 X175.0000 Y5.0000
G01 X175.0000 Y5.0000
G01 X175.0000 Y5.0000
G00 Z2.0000
G00 X175.0000 Y112.9049
G01 F60.00
G01 Z-5.0000
G01 F120.00
G01 X175.0000 Y112.9049
G01 X175.0000 Y112.9049
G01 X175.0000 Y112.9049
G01 X175.0000 Y112.9049
G00 Z2.0000
G00 X100.0000 Y112.9049
G01 F60.00
G01 Z-5.0000
G01 F120.00
G01 X100.0000 Y112.9049
G01 X100.0000 Y112.9049
G01 X100.0000 Y112.9049
G01 X100.0000 Y112.9049
G00 Z2.0000
G00 X25.0000 Y112.9049
G01 F60.00
G01 Z-5.0000
G01 F120.00
G01 X25.0000 Y112.9049
G01 X25.0000 Y112.9049
G01 X25.0000 Y112.9049
G01 X25.0000 Y112.9049
G00 Z2.0000
M05
G00 Z2.0000
G00 Z15.00

