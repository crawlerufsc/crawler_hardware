(G-CODE GENERATED BY FLATCAM v8.993 - www.flatcam.org - Version Date: 2020/06/05)

(Name: Holes)
(Type: G-code from Geometry)
(Units: MM)

(Created on Wednesday, 24 March 2021 at 14:17)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)

(TOOL DIAMETER: 2.4 mm)
(Feedrate_XY: 120.0 mm/min)
(Feedrate_Z: 60.0 mm/min)
(Feedrate rapids 1500.0 mm/min)

(Z_Cut: -2.0 mm)
(Z_Move: 2.0 mm)
(Z Start: None mm)
(Z End: 15.0 mm)
(Steps per circle: 64)
(Preprocessor Geometry: default)

(X range:   50.0000 ...  180.0000  mm)
(Y range:   20.0000 ...   97.9050  mm)

(Spindle Speed: None RPM)
G21
G90
G94



G01 F120.00
G00 Z2.0000

M03
G00 X50.0000 Y20.0000
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X50.0000 Y20.0000
G01 X50.0000 Y20.0000
G01 X50.0000 Y20.0000
G01 X50.0000 Y20.0000
G00 Z2.0000
G00 X50.0000 Y97.9050
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X50.0000 Y97.9050
G01 X50.0000 Y97.9050
G01 X50.0000 Y97.9050
G01 X50.0000 Y97.9050
G00 Z2.0000
G00 X180.0000 Y81.9050
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X180.0000 Y81.9050
G01 X180.0000 Y81.9050
G01 X180.0000 Y81.9050
G01 X180.0000 Y81.9050
G00 Z2.0000
G00 X180.0000 Y36.0000
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X180.0000 Y36.0000
G01 X180.0000 Y36.0000
G01 X180.0000 Y36.0000
G01 X180.0000 Y36.0000
G00 Z2.0000
M05
G00 Z2.0000
G00 Z15.00

