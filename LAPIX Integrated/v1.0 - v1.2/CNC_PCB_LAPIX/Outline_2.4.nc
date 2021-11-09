(G-CODE GENERATED BY FLATCAM v8.993 - www.flatcam.org - Version Date: 2020/06/05)

(Name: Outline_2.4)
(Type: G-code from Geometry)
(Units: MM)

(Created on Monday, 22 March 2021 at 10:25)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)

(TOOL DIAMETER: 2.4 mm)
(Feedrate_XY: 120.0 mm/min)
(Feedrate_Z: 60.0 mm/min)
(Feedrate rapids 1500.0 mm/min)

(Z_Cut: -2.0 mm)
(DepthPerCut: 0.6 mm <=>4 passes)
(Z_Move: 2.0 mm)
(Z Start: None mm)
(Z End: 15.0 mm)
(Steps per circle: 64)
(Preprocessor Geometry: default)

(X range:    9.9000 ...  189.5384  mm)
(Y range:    9.9000 ...  108.0050  mm)

(Spindle Speed: None RPM)
G21
G90
G94



G01 F120.00
G00 Z2.0000

M03
G00 X51.2000 Y9.9000
G01 F60.00
G01 Z-0.6000
G01 F120.00
G01 X17.1692 Y9.9000
G01 X17.1216 Y9.9008
G01 X17.0915 Y9.9018
G01 X16.5759 Y9.9256
G01 X16.5160 Y9.9298
G01 X16.4810 Y9.9330
G01 X16.4213 Y9.9400
G01 X15.9101 Y10.0113
G01 X15.8509 Y10.0209
G01 X15.8162 Y10.0274
G01 X15.7575 Y10.0398
G01 X15.2551 Y10.1580
G01 X15.1969 Y10.1731
G01 X15.1631 Y10.1827
G01 X15.1057 Y10.2005
G01 X14.6163 Y10.3645
G01 X14.5598 Y10.3849
G01 X14.5270 Y10.3976
G01 X14.4715 Y10.4206
G01 X13.9993 Y10.6291
G01 X13.9450 Y10.6546
G01 X13.9135 Y10.6703
G01 X13.8603 Y10.6983
G01 X13.4094 Y10.9495
G01 X13.3576 Y10.9799
G01 X13.3277 Y10.9984
G01 X13.2774 Y11.0312
G01 X12.8516 Y11.3229
G01 X12.8028 Y11.3580
G01 X12.7747 Y11.3792
G01 X12.7276 Y11.4165
G01 X12.3306 Y11.7462
G01 X12.2853 Y11.7856
G01 X12.2592 Y11.8094
G01 X12.2158 Y11.8508
G01 X11.8508 Y12.2158
G01 X11.8094 Y12.2592
G01 X11.7856 Y12.2853
G01 X11.7462 Y12.3306
G01 X11.4165 Y12.7276
G01 X11.3792 Y12.7747
G01 X11.3580 Y12.8028
G01 X11.3229 Y12.8516
G01 X11.0312 Y13.2774
G01 X10.9984 Y13.3277
G01 X10.9799 Y13.3576
G01 X10.9495 Y13.4094
G01 X10.6983 Y13.8603
G01 X10.6703 Y13.9135
G01 X10.6546 Y13.9450
G01 X10.6291 Y13.9993
G01 X10.4206 Y14.4715
G01 X10.3976 Y14.5270
G01 X10.3849 Y14.5598
G01 X10.3645 Y14.6163
G01 X10.2005 Y15.1057
G01 X10.1827 Y15.1631
G01 X10.1731 Y15.1969
G01 X10.1580 Y15.2551
G01 X10.0398 Y15.7575
G01 X10.0274 Y15.8162
G01 X10.0209 Y15.8509
G01 X10.0113 Y15.9101
G01 X9.9400 Y16.4213
G01 X9.9330 Y16.4810
G01 X9.9298 Y16.5160
G01 X9.9256 Y16.5759
G01 X9.9018 Y17.0915
G01 X9.9008 Y17.1216
G01 X9.9000 Y17.1692
G01 X9.9000 Y61.2000
G00 X9.9000 Y61.2000
G01 F60.00
G01 Z-1.2000
G01 F120.00
G01 X9.9000 Y17.1692
G01 X9.9008 Y17.1216
G01 X9.9018 Y17.0915
G01 X9.9256 Y16.5759
G01 X9.9298 Y16.5160
G01 X9.9330 Y16.4810
G01 X9.9400 Y16.4213
G01 X10.0113 Y15.9101
G01 X10.0209 Y15.8509
G01 X10.0274 Y15.8162
G01 X10.0398 Y15.7575
G01 X10.1580 Y15.2551
G01 X10.1731 Y15.1969
G01 X10.1827 Y15.1631
G01 X10.2005 Y15.1057
G01 X10.3645 Y14.6163
G01 X10.3849 Y14.5598
G01 X10.3976 Y14.5270
G01 X10.4206 Y14.4715
G01 X10.6291 Y13.9993
G01 X10.6546 Y13.9450
G01 X10.6703 Y13.9135
G01 X10.6983 Y13.8603
G01 X10.9495 Y13.4094
G01 X10.9799 Y13.3576
G01 X10.9984 Y13.3277
G01 X11.0312 Y13.2774
G01 X11.3229 Y12.8516
G01 X11.3580 Y12.8028
G01 X11.3792 Y12.7747
G01 X11.4165 Y12.7276
G01 X11.7462 Y12.3306
G01 X11.7856 Y12.2853
G01 X11.8094 Y12.2592
G01 X11.8508 Y12.2158
G01 X12.2158 Y11.8508
G01 X12.2592 Y11.8094
G01 X12.2853 Y11.7856
G01 X12.3306 Y11.7462
G01 X12.7276 Y11.4165
G01 X12.7747 Y11.3792
G01 X12.8028 Y11.3580
G01 X12.8516 Y11.3229
G01 X13.2774 Y11.0312
G01 X13.3277 Y10.9984
G01 X13.3576 Y10.9799
G01 X13.4094 Y10.9495
G01 X13.8603 Y10.6983
G01 X13.9135 Y10.6703
G01 X13.9450 Y10.6546
G01 X13.9993 Y10.6291
G01 X14.4715 Y10.4206
G01 X14.5270 Y10.3976
G01 X14.5598 Y10.3849
G01 X14.6163 Y10.3645
G01 X15.1057 Y10.2005
G01 X15.1631 Y10.1827
G01 X15.1969 Y10.1731
G01 X15.2551 Y10.1580
G01 X15.7575 Y10.0398
G01 X15.8162 Y10.0274
G01 X15.8509 Y10.0209
G01 X15.9101 Y10.0113
G01 X16.4213 Y9.9400
G01 X16.4810 Y9.9330
G01 X16.5160 Y9.9298
G01 X16.5759 Y9.9256
G01 X17.0915 Y9.9018
G01 X17.1216 Y9.9008
G01 X17.1692 Y9.9000
G01 X51.2000 Y9.9000
G00 X51.2000 Y9.9000
G01 F60.00
G01 Z-1.8000
G01 F120.00
G01 X17.1692 Y9.9000
G01 X17.1216 Y9.9008
G01 X17.0915 Y9.9018
G01 X16.5759 Y9.9256
G01 X16.5160 Y9.9298
G01 X16.4810 Y9.9330
G01 X16.4213 Y9.9400
G01 X15.9101 Y10.0113
G01 X15.8509 Y10.0209
G01 X15.8162 Y10.0274
G01 X15.7575 Y10.0398
G01 X15.2551 Y10.1580
G01 X15.1969 Y10.1731
G01 X15.1631 Y10.1827
G01 X15.1057 Y10.2005
G01 X14.6163 Y10.3645
G01 X14.5598 Y10.3849
G01 X14.5270 Y10.3976
G01 X14.4715 Y10.4206
G01 X13.9993 Y10.6291
G01 X13.9450 Y10.6546
G01 X13.9135 Y10.6703
G01 X13.8603 Y10.6983
G01 X13.4094 Y10.9495
G01 X13.3576 Y10.9799
G01 X13.3277 Y10.9984
G01 X13.2774 Y11.0312
G01 X12.8516 Y11.3229
G01 X12.8028 Y11.3580
G01 X12.7747 Y11.3792
G01 X12.7276 Y11.4165
G01 X12.3306 Y11.7462
G01 X12.2853 Y11.7856
G01 X12.2592 Y11.8094
G01 X12.2158 Y11.8508
G01 X11.8508 Y12.2158
G01 X11.8094 Y12.2592
G01 X11.7856 Y12.2853
G01 X11.7462 Y12.3306
G01 X11.4165 Y12.7276
G01 X11.3792 Y12.7747
G01 X11.3580 Y12.8028
G01 X11.3229 Y12.8516
G01 X11.0312 Y13.2774
G01 X10.9984 Y13.3277
G01 X10.9799 Y13.3576
G01 X10.9495 Y13.4094
G01 X10.6983 Y13.8603
G01 X10.6703 Y13.9135
G01 X10.6546 Y13.9450
G01 X10.6291 Y13.9993
G01 X10.4206 Y14.4715
G01 X10.3976 Y14.5270
G01 X10.3849 Y14.5598
G01 X10.3645 Y14.6163
G01 X10.2005 Y15.1057
G01 X10.1827 Y15.1631
G01 X10.1731 Y15.1969
G01 X10.1580 Y15.2551
G01 X10.0398 Y15.7575
G01 X10.0274 Y15.8162
G01 X10.0209 Y15.8509
G01 X10.0113 Y15.9101
G01 X9.9400 Y16.4213
G01 X9.9330 Y16.4810
G01 X9.9298 Y16.5160
G01 X9.9256 Y16.5759
G01 X9.9018 Y17.0915
G01 X9.9008 Y17.1216
G01 X9.9000 Y17.1692
G01 X9.9000 Y61.2000
G00 X9.9000 Y61.2000
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X9.9000 Y17.1692
G01 X9.9008 Y17.1216
G01 X9.9018 Y17.0915
G01 X9.9256 Y16.5759
G01 X9.9298 Y16.5160
G01 X9.9330 Y16.4810
G01 X9.9400 Y16.4213
G01 X10.0113 Y15.9101
G01 X10.0209 Y15.8509
G01 X10.0274 Y15.8162
G01 X10.0398 Y15.7575
G01 X10.1580 Y15.2551
G01 X10.1731 Y15.1969
G01 X10.1827 Y15.1631
G01 X10.2005 Y15.1057
G01 X10.3645 Y14.6163
G01 X10.3849 Y14.5598
G01 X10.3976 Y14.5270
G01 X10.4206 Y14.4715
G01 X10.6291 Y13.9993
G01 X10.6546 Y13.9450
G01 X10.6703 Y13.9135
G01 X10.6983 Y13.8603
G01 X10.9495 Y13.4094
G01 X10.9799 Y13.3576
G01 X10.9984 Y13.3277
G01 X11.0312 Y13.2774
G01 X11.3229 Y12.8516
G01 X11.3580 Y12.8028
G01 X11.3792 Y12.7747
G01 X11.4165 Y12.7276
G01 X11.7462 Y12.3306
G01 X11.7856 Y12.2853
G01 X11.8094 Y12.2592
G01 X11.8508 Y12.2158
G01 X12.2158 Y11.8508
G01 X12.2592 Y11.8094
G01 X12.2853 Y11.7856
G01 X12.3306 Y11.7462
G01 X12.7276 Y11.4165
G01 X12.7747 Y11.3792
G01 X12.8028 Y11.3580
G01 X12.8516 Y11.3229
G01 X13.2774 Y11.0312
G01 X13.3277 Y10.9984
G01 X13.3576 Y10.9799
G01 X13.4094 Y10.9495
G01 X13.8603 Y10.6983
G01 X13.9135 Y10.6703
G01 X13.9450 Y10.6546
G01 X13.9993 Y10.6291
G01 X14.4715 Y10.4206
G01 X14.5270 Y10.3976
G01 X14.5598 Y10.3849
G01 X14.6163 Y10.3645
G01 X15.1057 Y10.2005
G01 X15.1631 Y10.1827
G01 X15.1969 Y10.1731
G01 X15.2551 Y10.1580
G01 X15.7575 Y10.0398
G01 X15.8162 Y10.0274
G01 X15.8509 Y10.0209
G01 X15.9101 Y10.0113
G01 X16.4213 Y9.9400
G01 X16.4810 Y9.9330
G01 X16.5160 Y9.9298
G01 X16.5759 Y9.9256
G01 X17.0915 Y9.9018
G01 X17.1216 Y9.9008
G01 X17.1692 Y9.9000
G01 X51.2000 Y9.9000
G00 Z2.0000
G00 X61.2000 Y9.9000
G01 F60.00
G01 Z-0.6000
G01 F120.00
G01 X88.9242 Y9.9000
G01 X88.9880 Y9.9016
G01 X89.1150 Y9.9016
G01 X89.1523 Y9.9034
G01 X89.2793 Y9.9159
G01 X89.3163 Y9.9214
G01 X89.4414 Y9.9463
G01 X89.4777 Y9.9554
G01 X89.5998 Y9.9924
G01 X89.6350 Y10.0050
G01 X89.7529 Y10.0538
G01 X89.7867 Y10.0698
G01 X89.8992 Y10.1300
G01 X89.9313 Y10.1492
G01 X90.0374 Y10.2201
G01 X90.0674 Y10.2424
G01 X90.1660 Y10.3233
G01 X90.1938 Y10.3484
G01 X90.2400 Y10.3924
G01 X106.5116 Y26.6640
G01 X141.2000 Y26.6651
G00 X141.2000 Y26.6651
G01 F60.00
G01 Z-1.2000
G01 F120.00
G01 X106.5116 Y26.6640
G01 X90.2400 Y10.3924
G01 X90.1938 Y10.3484
G01 X90.1660 Y10.3233
G01 X90.0674 Y10.2424
G01 X90.0374 Y10.2201
G01 X89.9313 Y10.1492
G01 X89.8992 Y10.1300
G01 X89.7867 Y10.0698
G01 X89.7529 Y10.0538
G01 X89.6350 Y10.0050
G01 X89.5998 Y9.9924
G01 X89.4777 Y9.9554
G01 X89.4414 Y9.9463
G01 X89.3163 Y9.9214
G01 X89.2793 Y9.9159
G01 X89.1523 Y9.9034
G01 X89.1150 Y9.9016
G01 X88.9880 Y9.9016
G01 X88.9242 Y9.9000
G01 X61.2000 Y9.9000
G00 X61.2000 Y9.9000
G01 F60.00
G01 Z-1.8000
G01 F120.00
G01 X88.9242 Y9.9000
G01 X88.9880 Y9.9016
G01 X89.1150 Y9.9016
G01 X89.1523 Y9.9034
G01 X89.2793 Y9.9159
G01 X89.3163 Y9.9214
G01 X89.4414 Y9.9463
G01 X89.4777 Y9.9554
G01 X89.5998 Y9.9924
G01 X89.6350 Y10.0050
G01 X89.7529 Y10.0538
G01 X89.7867 Y10.0698
G01 X89.8992 Y10.1300
G01 X89.9313 Y10.1492
G01 X90.0374 Y10.2201
G01 X90.0674 Y10.2424
G01 X90.1660 Y10.3233
G01 X90.1938 Y10.3484
G01 X90.2400 Y10.3924
G01 X106.5116 Y26.6640
G01 X141.2000 Y26.6651
G00 X141.2000 Y26.6651
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X106.5116 Y26.6640
G01 X90.2400 Y10.3924
G01 X90.1938 Y10.3484
G01 X90.1660 Y10.3233
G01 X90.0674 Y10.2424
G01 X90.0374 Y10.2201
G01 X89.9313 Y10.1492
G01 X89.8992 Y10.1300
G01 X89.7867 Y10.0698
G01 X89.7529 Y10.0538
G01 X89.6350 Y10.0050
G01 X89.5998 Y9.9924
G01 X89.4777 Y9.9554
G01 X89.4414 Y9.9463
G01 X89.3163 Y9.9214
G01 X89.2793 Y9.9159
G01 X89.1523 Y9.9034
G01 X89.1150 Y9.9016
G01 X88.9880 Y9.9016
G01 X88.9242 Y9.9000
G01 X61.2000 Y9.9000
G00 Z2.0000
G00 X9.9000 Y71.2000
G01 F60.00
G01 Z-0.6000
G01 F120.00
G01 X9.9000 Y100.9892
G01 X9.9016 Y101.0530
G01 X9.9034 Y101.0903
G01 X9.9159 Y101.2173
G01 X9.9214 Y101.2543
G01 X9.9234 Y101.2645
G01 X9.9293 Y101.3725
G01 X9.9336 Y101.4296
G01 X9.9373 Y101.4668
G01 X9.9451 Y101.5302
G01 X10.0256 Y102.0726
G01 X10.0365 Y102.1355
G01 X10.0438 Y102.1722
G01 X10.0578 Y102.2344
G01 X10.1910 Y102.7664
G01 X10.2080 Y102.8279
G01 X10.2189 Y102.8637
G01 X10.2389 Y102.9243
G01 X10.4236 Y103.4406
G01 X10.4466 Y103.5001
G01 X10.4609 Y103.5347
G01 X10.4868 Y103.5930
G01 X10.7212 Y104.0887
G01 X10.7499 Y104.1457
G01 X10.7676 Y104.1787
G01 X10.7990 Y104.2342
G01 X11.0809 Y104.7046
G01 X11.1151 Y104.7585
G01 X11.1359 Y104.7896
G01 X11.1726 Y104.8418
G01 X11.4993 Y105.2822
G01 X11.5385 Y105.3325
G01 X11.5623 Y105.3615
G01 X11.6039 Y105.4098
G01 X11.9722 Y105.8161
G01 X12.0162 Y105.8623
G01 X12.0426 Y105.8888
G01 X12.0889 Y105.9328
G01 X12.4952 Y106.3010
G01 X12.5435 Y106.3427
G01 X12.5724 Y106.3664
G01 X12.6227 Y106.4057
G01 X13.0632 Y106.7324
G01 X13.1153 Y106.7691
G01 X13.1465 Y106.7899
G01 X13.2004 Y106.8240
G01 X13.6707 Y107.1060
G01 X13.7262 Y107.1374
G01 X13.7592 Y107.1550
G01 X13.8162 Y107.1837
G01 X14.3120 Y107.4182
G01 X14.3703 Y107.4440
G01 X14.4048 Y107.4584
G01 X14.4644 Y107.4813
G01 X14.9807 Y107.6661
G01 X15.0413 Y107.6861
G01 X15.0771 Y107.6969
G01 X15.1386 Y107.7140
G01 X15.6705 Y107.8472
G01 X15.7328 Y107.8612
G01 X15.7695 Y107.8685
G01 X15.8323 Y107.8794
G01 X16.3748 Y107.9599
G01 X16.4381 Y107.9677
G01 X16.4753 Y107.9713
G01 X16.5390 Y107.9760
G01 X17.0867 Y108.0029
G01 X17.1373 Y108.0046
G01 X17.1693 Y108.0050
G01 X51.2000 Y108.0036
G00 X51.2000 Y108.0036
G01 F60.00
G01 Z-1.2000
G01 F120.00
G01 X17.1693 Y108.0050
G01 X17.1373 Y108.0046
G01 X17.0867 Y108.0029
G01 X16.5390 Y107.9760
G01 X16.4753 Y107.9713
G01 X16.4381 Y107.9677
G01 X16.3748 Y107.9599
G01 X15.8323 Y107.8794
G01 X15.7695 Y107.8685
G01 X15.7328 Y107.8612
G01 X15.6705 Y107.8472
G01 X15.1386 Y107.7140
G01 X15.0771 Y107.6969
G01 X15.0413 Y107.6861
G01 X14.9807 Y107.6661
G01 X14.4644 Y107.4813
G01 X14.4048 Y107.4584
G01 X14.3703 Y107.4440
G01 X14.3120 Y107.4182
G01 X13.8162 Y107.1837
G01 X13.7592 Y107.1550
G01 X13.7262 Y107.1374
G01 X13.6707 Y107.1060
G01 X13.2004 Y106.8240
G01 X13.1465 Y106.7899
G01 X13.1153 Y106.7691
G01 X13.0632 Y106.7324
G01 X12.6227 Y106.4057
G01 X12.5724 Y106.3664
G01 X12.5435 Y106.3427
G01 X12.4952 Y106.3010
G01 X12.0889 Y105.9328
G01 X12.0426 Y105.8888
G01 X12.0162 Y105.8623
G01 X11.9722 Y105.8161
G01 X11.6039 Y105.4098
G01 X11.5623 Y105.3615
G01 X11.5385 Y105.3325
G01 X11.4993 Y105.2822
G01 X11.1726 Y104.8418
G01 X11.1359 Y104.7896
G01 X11.1151 Y104.7585
G01 X11.0809 Y104.7046
G01 X10.7990 Y104.2342
G01 X10.7676 Y104.1787
G01 X10.7499 Y104.1457
G01 X10.7212 Y104.0887
G01 X10.4868 Y103.5930
G01 X10.4609 Y103.5347
G01 X10.4466 Y103.5001
G01 X10.4236 Y103.4406
G01 X10.2389 Y102.9243
G01 X10.2189 Y102.8637
G01 X10.2080 Y102.8279
G01 X10.1910 Y102.7664
G01 X10.0578 Y102.2344
G01 X10.0438 Y102.1722
G01 X10.0365 Y102.1355
G01 X10.0256 Y102.0726
G01 X9.9451 Y101.5302
G01 X9.9373 Y101.4668
G01 X9.9336 Y101.4296
G01 X9.9293 Y101.3725
G01 X9.9234 Y101.2645
G01 X9.9214 Y101.2543
G01 X9.9159 Y101.2173
G01 X9.9034 Y101.0903
G01 X9.9016 Y101.0530
G01 X9.9000 Y100.9892
G01 X9.9000 Y71.2000
G00 X9.9000 Y71.2000
G01 F60.00
G01 Z-1.8000
G01 F120.00
G01 X9.9000 Y100.9892
G01 X9.9016 Y101.0530
G01 X9.9034 Y101.0903
G01 X9.9159 Y101.2173
G01 X9.9214 Y101.2543
G01 X9.9234 Y101.2645
G01 X9.9293 Y101.3725
G01 X9.9336 Y101.4296
G01 X9.9373 Y101.4668
G01 X9.9451 Y101.5302
G01 X10.0256 Y102.0726
G01 X10.0365 Y102.1355
G01 X10.0438 Y102.1722
G01 X10.0578 Y102.2344
G01 X10.1910 Y102.7664
G01 X10.2080 Y102.8279
G01 X10.2189 Y102.8637
G01 X10.2389 Y102.9243
G01 X10.4236 Y103.4406
G01 X10.4466 Y103.5001
G01 X10.4609 Y103.5347
G01 X10.4868 Y103.5930
G01 X10.7212 Y104.0887
G01 X10.7499 Y104.1457
G01 X10.7676 Y104.1787
G01 X10.7990 Y104.2342
G01 X11.0809 Y104.7046
G01 X11.1151 Y104.7585
G01 X11.1359 Y104.7896
G01 X11.1726 Y104.8418
G01 X11.4993 Y105.2822
G01 X11.5385 Y105.3325
G01 X11.5623 Y105.3615
G01 X11.6039 Y105.4098
G01 X11.9722 Y105.8161
G01 X12.0162 Y105.8623
G01 X12.0426 Y105.8888
G01 X12.0889 Y105.9328
G01 X12.4952 Y106.3010
G01 X12.5435 Y106.3427
G01 X12.5724 Y106.3664
G01 X12.6227 Y106.4057
G01 X13.0632 Y106.7324
G01 X13.1153 Y106.7691
G01 X13.1465 Y106.7899
G01 X13.2004 Y106.8240
G01 X13.6707 Y107.1060
G01 X13.7262 Y107.1374
G01 X13.7592 Y107.1550
G01 X13.8162 Y107.1837
G01 X14.3120 Y107.4182
G01 X14.3703 Y107.4440
G01 X14.4048 Y107.4584
G01 X14.4644 Y107.4813
G01 X14.9807 Y107.6661
G01 X15.0413 Y107.6861
G01 X15.0771 Y107.6969
G01 X15.1386 Y107.7140
G01 X15.6705 Y107.8472
G01 X15.7328 Y107.8612
G01 X15.7695 Y107.8685
G01 X15.8323 Y107.8794
G01 X16.3748 Y107.9599
G01 X16.4381 Y107.9677
G01 X16.4753 Y107.9713
G01 X16.5390 Y107.9760
G01 X17.0867 Y108.0029
G01 X17.1373 Y108.0046
G01 X17.1693 Y108.0050
G01 X51.2000 Y108.0036
G00 X51.2000 Y108.0036
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X17.1693 Y108.0050
G01 X17.1373 Y108.0046
G01 X17.0867 Y108.0029
G01 X16.5390 Y107.9760
G01 X16.4753 Y107.9713
G01 X16.4381 Y107.9677
G01 X16.3748 Y107.9599
G01 X15.8323 Y107.8794
G01 X15.7695 Y107.8685
G01 X15.7328 Y107.8612
G01 X15.6705 Y107.8472
G01 X15.1386 Y107.7140
G01 X15.0771 Y107.6969
G01 X15.0413 Y107.6861
G01 X14.9807 Y107.6661
G01 X14.4644 Y107.4813
G01 X14.4048 Y107.4584
G01 X14.3703 Y107.4440
G01 X14.3120 Y107.4182
G01 X13.8162 Y107.1837
G01 X13.7592 Y107.1550
G01 X13.7262 Y107.1374
G01 X13.6707 Y107.1060
G01 X13.2004 Y106.8240
G01 X13.1465 Y106.7899
G01 X13.1153 Y106.7691
G01 X13.0632 Y106.7324
G01 X12.6227 Y106.4057
G01 X12.5724 Y106.3664
G01 X12.5435 Y106.3427
G01 X12.4952 Y106.3010
G01 X12.0889 Y105.9328
G01 X12.0426 Y105.8888
G01 X12.0162 Y105.8623
G01 X11.9722 Y105.8161
G01 X11.6039 Y105.4098
G01 X11.5623 Y105.3615
G01 X11.5385 Y105.3325
G01 X11.4993 Y105.2822
G01 X11.1726 Y104.8418
G01 X11.1359 Y104.7896
G01 X11.1151 Y104.7585
G01 X11.0809 Y104.7046
G01 X10.7990 Y104.2342
G01 X10.7676 Y104.1787
G01 X10.7499 Y104.1457
G01 X10.7212 Y104.0887
G01 X10.4868 Y103.5930
G01 X10.4609 Y103.5347
G01 X10.4466 Y103.5001
G01 X10.4236 Y103.4406
G01 X10.2389 Y102.9243
G01 X10.2189 Y102.8637
G01 X10.2080 Y102.8279
G01 X10.1910 Y102.7664
G01 X10.0578 Y102.2344
G01 X10.0438 Y102.1722
G01 X10.0365 Y102.1355
G01 X10.0256 Y102.0726
G01 X9.9451 Y101.5302
G01 X9.9373 Y101.4668
G01 X9.9336 Y101.4296
G01 X9.9293 Y101.3725
G01 X9.9234 Y101.2645
G01 X9.9214 Y101.2543
G01 X9.9159 Y101.2173
G01 X9.9034 Y101.0903
G01 X9.9016 Y101.0530
G01 X9.9000 Y100.9892
G01 X9.9000 Y71.2000
G00 Z2.0000
G00 X61.2000 Y108.0032
G01 F60.00
G01 Z-0.6000
G01 F120.00
G01 X89.1783 Y108.0020
G01 X89.2419 Y108.0005
G01 X89.3690 Y108.0005
G01 X89.4063 Y107.9986
G01 X89.5333 Y107.9861
G01 X89.5703 Y107.9806
G01 X89.6954 Y107.9558
G01 X89.7317 Y107.9467
G01 X89.8538 Y107.9096
G01 X89.8890 Y107.8970
G01 X90.0069 Y107.8482
G01 X90.0407 Y107.8322
G01 X90.1532 Y107.7721
G01 X90.1853 Y107.7528
G01 X90.2914 Y107.6820
G01 X90.3214 Y107.6597
G01 X90.4200 Y107.5788
G01 X90.4478 Y107.5536
G01 X90.4940 Y107.5096
G01 X106.7655 Y91.2381
G01 X141.2000 Y91.2391
G00 X141.2000 Y91.2391
G01 F60.00
G01 Z-1.2000
G01 F120.00
G01 X106.7655 Y91.2381
G01 X90.4940 Y107.5096
G01 X90.4478 Y107.5536
G01 X90.4200 Y107.5788
G01 X90.3214 Y107.6597
G01 X90.2914 Y107.6820
G01 X90.1853 Y107.7528
G01 X90.1532 Y107.7721
G01 X90.0407 Y107.8322
G01 X90.0069 Y107.8482
G01 X89.8890 Y107.8970
G01 X89.8538 Y107.9096
G01 X89.7317 Y107.9467
G01 X89.6954 Y107.9558
G01 X89.5703 Y107.9806
G01 X89.5333 Y107.9861
G01 X89.4063 Y107.9986
G01 X89.3690 Y108.0005
G01 X89.2419 Y108.0005
G01 X89.1783 Y108.0020
G01 X61.2000 Y108.0032
G00 X61.2000 Y108.0032
G01 F60.00
G01 Z-1.8000
G01 F120.00
G01 X89.1783 Y108.0020
G01 X89.2419 Y108.0005
G01 X89.3690 Y108.0005
G01 X89.4063 Y107.9986
G01 X89.5333 Y107.9861
G01 X89.5703 Y107.9806
G01 X89.6954 Y107.9558
G01 X89.7317 Y107.9467
G01 X89.8538 Y107.9096
G01 X89.8890 Y107.8970
G01 X90.0069 Y107.8482
G01 X90.0407 Y107.8322
G01 X90.1532 Y107.7721
G01 X90.1853 Y107.7528
G01 X90.2914 Y107.6820
G01 X90.3214 Y107.6597
G01 X90.4200 Y107.5788
G01 X90.4478 Y107.5536
G01 X90.4940 Y107.5096
G01 X106.7655 Y91.2381
G01 X141.2000 Y91.2391
G00 X141.2000 Y91.2391
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X106.7655 Y91.2381
G01 X90.4940 Y107.5096
G01 X90.4478 Y107.5536
G01 X90.4200 Y107.5788
G01 X90.3214 Y107.6597
G01 X90.2914 Y107.6820
G01 X90.1853 Y107.7528
G01 X90.1532 Y107.7721
G01 X90.0407 Y107.8322
G01 X90.0069 Y107.8482
G01 X89.8890 Y107.8970
G01 X89.8538 Y107.9096
G01 X89.7317 Y107.9467
G01 X89.6954 Y107.9558
G01 X89.5703 Y107.9806
G01 X89.5333 Y107.9861
G01 X89.4063 Y107.9986
G01 X89.3690 Y108.0005
G01 X89.2419 Y108.0005
G01 X89.1783 Y108.0020
G01 X61.2000 Y108.0032
G00 Z2.0000
G00 X151.2000 Y91.2394
G01 F60.00
G01 Z-0.6000
G01 F120.00
G01 X182.2691 Y91.2404
G01 X182.3198 Y91.2395
G01 X182.3517 Y91.2384
G01 X182.8994 Y91.2114
G01 X182.9630 Y91.2067
G01 X183.0003 Y91.2031
G01 X183.0636 Y91.1953
G01 X183.6060 Y91.1148
G01 X183.6689 Y91.1039
G01 X183.7056 Y91.0966
G01 X183.7678 Y91.0826
G01 X184.2998 Y90.9494
G01 X184.3613 Y90.9324
G01 X184.3971 Y90.9215
G01 X184.4577 Y90.9015
G01 X184.9740 Y90.7167
G01 X185.0335 Y90.6938
G01 X185.0681 Y90.6795
G01 X185.1264 Y90.6536
G01 X185.6222 Y90.4191
G01 X185.6791 Y90.3905
G01 X185.7121 Y90.3728
G01 X185.7677 Y90.3414
G01 X186.2380 Y90.0594
G01 X186.2919 Y90.0253
G01 X186.3230 Y90.0045
G01 X186.3752 Y89.9678
G01 X186.8157 Y89.6411
G01 X186.8660 Y89.6019
G01 X186.8949 Y89.5781
G01 X186.9432 Y89.5364
G01 X187.3495 Y89.1682
G01 X187.3957 Y89.1242
G01 X187.4222 Y89.0977
G01 X187.4662 Y89.0515
G01 X187.8344 Y88.6452
G01 X187.8761 Y88.5969
G01 X187.8999 Y88.5680
G01 X187.9391 Y88.5177
G01 X188.2658 Y88.0772
G01 X188.3025 Y88.0250
G01 X188.3233 Y87.9939
G01 X188.3574 Y87.9400
G01 X188.6394 Y87.4697
G01 X188.6708 Y87.4141
G01 X188.6885 Y87.3811
G01 X188.7171 Y87.3242
G01 X188.9516 Y86.8284
G01 X188.9775 Y86.7701
G01 X188.9918 Y86.7355
G01 X189.0147 Y86.6760
G01 X189.1995 Y86.1597
G01 X189.2195 Y86.0991
G01 X189.2304 Y86.0633
G01 X189.2474 Y86.0018
G01 X189.3806 Y85.4698
G01 X189.3946 Y85.4076
G01 X189.4019 Y85.3709
G01 X189.4128 Y85.3080
G01 X189.4933 Y84.7656
G01 X189.5011 Y84.7023
G01 X189.5047 Y84.6650
G01 X189.5094 Y84.6014
G01 X189.5364 Y84.0537
G01 X189.5380 Y84.0031
G01 X189.5384 Y83.9679
G01 X189.4736 Y71.2000
G00 X189.4736 Y71.2000
G01 F60.00
G01 Z-1.2000
G01 F120.00
G01 X189.5384 Y83.9679
G01 X189.5380 Y84.0031
G01 X189.5364 Y84.0537
G01 X189.5094 Y84.6014
G01 X189.5047 Y84.6650
G01 X189.5011 Y84.7023
G01 X189.4933 Y84.7656
G01 X189.4128 Y85.3080
G01 X189.4019 Y85.3709
G01 X189.3946 Y85.4076
G01 X189.3806 Y85.4698
G01 X189.2474 Y86.0018
G01 X189.2304 Y86.0633
G01 X189.2195 Y86.0991
G01 X189.1995 Y86.1597
G01 X189.0147 Y86.6760
G01 X188.9918 Y86.7355
G01 X188.9775 Y86.7701
G01 X188.9516 Y86.8284
G01 X188.7171 Y87.3242
G01 X188.6885 Y87.3811
G01 X188.6708 Y87.4141
G01 X188.6394 Y87.4697
G01 X188.3574 Y87.9400
G01 X188.3233 Y87.9939
G01 X188.3025 Y88.0250
G01 X188.2658 Y88.0772
G01 X187.9391 Y88.5177
G01 X187.8999 Y88.5680
G01 X187.8761 Y88.5969
G01 X187.8344 Y88.6452
G01 X187.4662 Y89.0515
G01 X187.4222 Y89.0977
G01 X187.3957 Y89.1242
G01 X187.3495 Y89.1682
G01 X186.9432 Y89.5364
G01 X186.8949 Y89.5781
G01 X186.8660 Y89.6019
G01 X186.8157 Y89.6411
G01 X186.3752 Y89.9678
G01 X186.3230 Y90.0045
G01 X186.2919 Y90.0253
G01 X186.2380 Y90.0594
G01 X185.7677 Y90.3414
G01 X185.7121 Y90.3728
G01 X185.6791 Y90.3905
G01 X185.6222 Y90.4191
G01 X185.1264 Y90.6536
G01 X185.0681 Y90.6795
G01 X185.0335 Y90.6938
G01 X184.9740 Y90.7167
G01 X184.4577 Y90.9015
G01 X184.3971 Y90.9215
G01 X184.3613 Y90.9324
G01 X184.2998 Y90.9494
G01 X183.7678 Y91.0826
G01 X183.7056 Y91.0966
G01 X183.6689 Y91.1039
G01 X183.6060 Y91.1148
G01 X183.0636 Y91.1953
G01 X183.0003 Y91.2031
G01 X182.9630 Y91.2067
G01 X182.8994 Y91.2114
G01 X182.3517 Y91.2384
G01 X182.3198 Y91.2395
G01 X182.2691 Y91.2404
G01 X151.2000 Y91.2394
G00 X151.2000 Y91.2394
G01 F60.00
G01 Z-1.8000
G01 F120.00
G01 X182.2691 Y91.2404
G01 X182.3198 Y91.2395
G01 X182.3517 Y91.2384
G01 X182.8994 Y91.2114
G01 X182.9630 Y91.2067
G01 X183.0003 Y91.2031
G01 X183.0636 Y91.1953
G01 X183.6060 Y91.1148
G01 X183.6689 Y91.1039
G01 X183.7056 Y91.0966
G01 X183.7678 Y91.0826
G01 X184.2998 Y90.9494
G01 X184.3613 Y90.9324
G01 X184.3971 Y90.9215
G01 X184.4577 Y90.9015
G01 X184.9740 Y90.7167
G01 X185.0335 Y90.6938
G01 X185.0681 Y90.6795
G01 X185.1264 Y90.6536
G01 X185.6222 Y90.4191
G01 X185.6791 Y90.3905
G01 X185.7121 Y90.3728
G01 X185.7677 Y90.3414
G01 X186.2380 Y90.0594
G01 X186.2919 Y90.0253
G01 X186.3230 Y90.0045
G01 X186.3752 Y89.9678
G01 X186.8157 Y89.6411
G01 X186.8660 Y89.6019
G01 X186.8949 Y89.5781
G01 X186.9432 Y89.5364
G01 X187.3495 Y89.1682
G01 X187.3957 Y89.1242
G01 X187.4222 Y89.0977
G01 X187.4662 Y89.0515
G01 X187.8344 Y88.6452
G01 X187.8761 Y88.5969
G01 X187.8999 Y88.5680
G01 X187.9391 Y88.5177
G01 X188.2658 Y88.0772
G01 X188.3025 Y88.0250
G01 X188.3233 Y87.9939
G01 X188.3574 Y87.9400
G01 X188.6394 Y87.4697
G01 X188.6708 Y87.4141
G01 X188.6885 Y87.3811
G01 X188.7171 Y87.3242
G01 X188.9516 Y86.8284
G01 X188.9775 Y86.7701
G01 X188.9918 Y86.7355
G01 X189.0147 Y86.6760
G01 X189.1995 Y86.1597
G01 X189.2195 Y86.0991
G01 X189.2304 Y86.0633
G01 X189.2474 Y86.0018
G01 X189.3806 Y85.4698
G01 X189.3946 Y85.4076
G01 X189.4019 Y85.3709
G01 X189.4128 Y85.3080
G01 X189.4933 Y84.7656
G01 X189.5011 Y84.7023
G01 X189.5047 Y84.6650
G01 X189.5094 Y84.6014
G01 X189.5364 Y84.0537
G01 X189.5380 Y84.0031
G01 X189.5384 Y83.9679
G01 X189.4736 Y71.2000
G00 X189.4736 Y71.2000
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X189.5384 Y83.9679
G01 X189.5380 Y84.0031
G01 X189.5364 Y84.0537
G01 X189.5094 Y84.6014
G01 X189.5047 Y84.6650
G01 X189.5011 Y84.7023
G01 X189.4933 Y84.7656
G01 X189.4128 Y85.3080
G01 X189.4019 Y85.3709
G01 X189.3946 Y85.4076
G01 X189.3806 Y85.4698
G01 X189.2474 Y86.0018
G01 X189.2304 Y86.0633
G01 X189.2195 Y86.0991
G01 X189.1995 Y86.1597
G01 X189.0147 Y86.6760
G01 X188.9918 Y86.7355
G01 X188.9775 Y86.7701
G01 X188.9516 Y86.8284
G01 X188.7171 Y87.3242
G01 X188.6885 Y87.3811
G01 X188.6708 Y87.4141
G01 X188.6394 Y87.4697
G01 X188.3574 Y87.9400
G01 X188.3233 Y87.9939
G01 X188.3025 Y88.0250
G01 X188.2658 Y88.0772
G01 X187.9391 Y88.5177
G01 X187.8999 Y88.5680
G01 X187.8761 Y88.5969
G01 X187.8344 Y88.6452
G01 X187.4662 Y89.0515
G01 X187.4222 Y89.0977
G01 X187.3957 Y89.1242
G01 X187.3495 Y89.1682
G01 X186.9432 Y89.5364
G01 X186.8949 Y89.5781
G01 X186.8660 Y89.6019
G01 X186.8157 Y89.6411
G01 X186.3752 Y89.9678
G01 X186.3230 Y90.0045
G01 X186.2919 Y90.0253
G01 X186.2380 Y90.0594
G01 X185.7677 Y90.3414
G01 X185.7121 Y90.3728
G01 X185.6791 Y90.3905
G01 X185.6222 Y90.4191
G01 X185.1264 Y90.6536
G01 X185.0681 Y90.6795
G01 X185.0335 Y90.6938
G01 X184.9740 Y90.7167
G01 X184.4577 Y90.9015
G01 X184.3971 Y90.9215
G01 X184.3613 Y90.9324
G01 X184.2998 Y90.9494
G01 X183.7678 Y91.0826
G01 X183.7056 Y91.0966
G01 X183.6689 Y91.1039
G01 X183.6060 Y91.1148
G01 X183.0636 Y91.1953
G01 X183.0003 Y91.2031
G01 X182.9630 Y91.2067
G01 X182.8994 Y91.2114
G01 X182.3517 Y91.2384
G01 X182.3198 Y91.2395
G01 X182.2691 Y91.2404
G01 X151.2000 Y91.2394
G00 Z2.0000
G00 X189.4228 Y61.2000
G01 F60.00
G01 Z-0.6000
G01 F120.00
G01 X189.2844 Y33.9247
G01 X189.2837 Y33.8904
G01 X189.2824 Y33.8530
G01 X189.2554 Y33.3053
G01 X189.2507 Y33.2417
G01 X189.2471 Y33.2045
G01 X189.2393 Y33.1411
G01 X189.1588 Y32.5987
G01 X189.1479 Y32.5358
G01 X189.1406 Y32.4991
G01 X189.1266 Y32.4369
G01 X188.9934 Y31.9049
G01 X188.9764 Y31.8434
G01 X188.9655 Y31.8076
G01 X188.9455 Y31.7470
G01 X188.7607 Y31.2307
G01 X188.7378 Y31.1712
G01 X188.7235 Y31.1366
G01 X188.6976 Y31.0783
G01 X188.4631 Y30.5826
G01 X188.4345 Y30.5256
G01 X188.4168 Y30.4926
G01 X188.3854 Y30.4371
G01 X188.1034 Y29.9667
G01 X188.0693 Y29.9128
G01 X188.0485 Y29.8817
G01 X188.0118 Y29.8295
G01 X187.6851 Y29.3891
G01 X187.6459 Y29.3388
G01 X187.6221 Y29.3098
G01 X187.5804 Y29.2615
G01 X187.2122 Y28.8552
G01 X187.1682 Y28.8090
G01 X187.1417 Y28.7825
G01 X187.0955 Y28.7385
G01 X186.6892 Y28.3703
G01 X186.6409 Y28.3286
G01 X186.6120 Y28.3049
G01 X186.5617 Y28.2656
G01 X186.1212 Y27.9389
G01 X186.0690 Y27.9022
G01 X186.0379 Y27.8814
G01 X185.9840 Y27.8473
G01 X185.5137 Y27.5653
G01 X185.4581 Y27.5339
G01 X185.4251 Y27.5163
G01 X185.3682 Y27.4876
G01 X184.8724 Y27.2531
G01 X184.8141 Y27.2273
G01 X184.7795 Y27.2129
G01 X184.7200 Y27.1900
G01 X184.2037 Y27.0052
G01 X184.1431 Y26.9852
G01 X184.1073 Y26.9744
G01 X184.0458 Y26.9573
G01 X183.5138 Y26.8241
G01 X183.4516 Y26.8101
G01 X183.4149 Y26.8028
G01 X183.3520 Y26.7919
G01 X182.8096 Y26.7114
G01 X182.7463 Y26.7036
G01 X182.7090 Y26.7000
G01 X182.6454 Y26.6953
G01 X182.0977 Y26.6684
G01 X182.0471 Y26.6667
G01 X182.0152 Y26.6663
G01 X151.2000 Y26.6654
G00 X151.2000 Y26.6654
G01 F60.00
G01 Z-1.2000
G01 F120.00
G01 X182.0152 Y26.6663
G01 X182.0471 Y26.6667
G01 X182.0977 Y26.6684
G01 X182.6454 Y26.6953
G01 X182.7090 Y26.7000
G01 X182.7463 Y26.7036
G01 X182.8096 Y26.7114
G01 X183.3520 Y26.7919
G01 X183.4149 Y26.8028
G01 X183.4516 Y26.8101
G01 X183.5138 Y26.8241
G01 X184.0458 Y26.9573
G01 X184.1073 Y26.9744
G01 X184.1431 Y26.9852
G01 X184.2037 Y27.0052
G01 X184.7200 Y27.1900
G01 X184.7795 Y27.2129
G01 X184.8141 Y27.2273
G01 X184.8724 Y27.2531
G01 X185.3682 Y27.4876
G01 X185.4251 Y27.5163
G01 X185.4581 Y27.5339
G01 X185.5137 Y27.5653
G01 X185.9840 Y27.8473
G01 X186.0379 Y27.8814
G01 X186.0690 Y27.9022
G01 X186.1212 Y27.9389
G01 X186.5617 Y28.2656
G01 X186.6120 Y28.3049
G01 X186.6409 Y28.3286
G01 X186.6892 Y28.3703
G01 X187.0955 Y28.7385
G01 X187.1417 Y28.7825
G01 X187.1682 Y28.8090
G01 X187.2122 Y28.8552
G01 X187.5804 Y29.2615
G01 X187.6221 Y29.3098
G01 X187.6459 Y29.3388
G01 X187.6851 Y29.3891
G01 X188.0118 Y29.8295
G01 X188.0485 Y29.8817
G01 X188.0693 Y29.9128
G01 X188.1034 Y29.9667
G01 X188.3854 Y30.4371
G01 X188.4168 Y30.4926
G01 X188.4345 Y30.5256
G01 X188.4631 Y30.5826
G01 X188.6976 Y31.0783
G01 X188.7235 Y31.1366
G01 X188.7378 Y31.1712
G01 X188.7607 Y31.2307
G01 X188.9455 Y31.7470
G01 X188.9655 Y31.8076
G01 X188.9764 Y31.8434
G01 X188.9934 Y31.9049
G01 X189.1266 Y32.4369
G01 X189.1406 Y32.4991
G01 X189.1479 Y32.5358
G01 X189.1588 Y32.5987
G01 X189.2393 Y33.1411
G01 X189.2471 Y33.2045
G01 X189.2507 Y33.2417
G01 X189.2554 Y33.3053
G01 X189.2824 Y33.8530
G01 X189.2837 Y33.8904
G01 X189.2844 Y33.9247
G01 X189.4228 Y61.2000
G00 X189.4228 Y61.2000
G01 F60.00
G01 Z-1.8000
G01 F120.00
G01 X189.2844 Y33.9247
G01 X189.2837 Y33.8904
G01 X189.2824 Y33.8530
G01 X189.2554 Y33.3053
G01 X189.2507 Y33.2417
G01 X189.2471 Y33.2045
G01 X189.2393 Y33.1411
G01 X189.1588 Y32.5987
G01 X189.1479 Y32.5358
G01 X189.1406 Y32.4991
G01 X189.1266 Y32.4369
G01 X188.9934 Y31.9049
G01 X188.9764 Y31.8434
G01 X188.9655 Y31.8076
G01 X188.9455 Y31.7470
G01 X188.7607 Y31.2307
G01 X188.7378 Y31.1712
G01 X188.7235 Y31.1366
G01 X188.6976 Y31.0783
G01 X188.4631 Y30.5826
G01 X188.4345 Y30.5256
G01 X188.4168 Y30.4926
G01 X188.3854 Y30.4371
G01 X188.1034 Y29.9667
G01 X188.0693 Y29.9128
G01 X188.0485 Y29.8817
G01 X188.0118 Y29.8295
G01 X187.6851 Y29.3891
G01 X187.6459 Y29.3388
G01 X187.6221 Y29.3098
G01 X187.5804 Y29.2615
G01 X187.2122 Y28.8552
G01 X187.1682 Y28.8090
G01 X187.1417 Y28.7825
G01 X187.0955 Y28.7385
G01 X186.6892 Y28.3703
G01 X186.6409 Y28.3286
G01 X186.6120 Y28.3049
G01 X186.5617 Y28.2656
G01 X186.1212 Y27.9389
G01 X186.0690 Y27.9022
G01 X186.0379 Y27.8814
G01 X185.9840 Y27.8473
G01 X185.5137 Y27.5653
G01 X185.4581 Y27.5339
G01 X185.4251 Y27.5163
G01 X185.3682 Y27.4876
G01 X184.8724 Y27.2531
G01 X184.8141 Y27.2273
G01 X184.7795 Y27.2129
G01 X184.7200 Y27.1900
G01 X184.2037 Y27.0052
G01 X184.1431 Y26.9852
G01 X184.1073 Y26.9744
G01 X184.0458 Y26.9573
G01 X183.5138 Y26.8241
G01 X183.4516 Y26.8101
G01 X183.4149 Y26.8028
G01 X183.3520 Y26.7919
G01 X182.8096 Y26.7114
G01 X182.7463 Y26.7036
G01 X182.7090 Y26.7000
G01 X182.6454 Y26.6953
G01 X182.0977 Y26.6684
G01 X182.0471 Y26.6667
G01 X182.0152 Y26.6663
G01 X151.2000 Y26.6654
G00 X151.2000 Y26.6654
G01 F60.00
G01 Z-2.0000
G01 F120.00
G01 X182.0152 Y26.6663
G01 X182.0471 Y26.6667
G01 X182.0977 Y26.6684
G01 X182.6454 Y26.6953
G01 X182.7090 Y26.7000
G01 X182.7463 Y26.7036
G01 X182.8096 Y26.7114
G01 X183.3520 Y26.7919
G01 X183.4149 Y26.8028
G01 X183.4516 Y26.8101
G01 X183.5138 Y26.8241
G01 X184.0458 Y26.9573
G01 X184.1073 Y26.9744
G01 X184.1431 Y26.9852
G01 X184.2037 Y27.0052
G01 X184.7200 Y27.1900
G01 X184.7795 Y27.2129
G01 X184.8141 Y27.2273
G01 X184.8724 Y27.2531
G01 X185.3682 Y27.4876
G01 X185.4251 Y27.5163
G01 X185.4581 Y27.5339
G01 X185.5137 Y27.5653
G01 X185.9840 Y27.8473
G01 X186.0379 Y27.8814
G01 X186.0690 Y27.9022
G01 X186.1212 Y27.9389
G01 X186.5617 Y28.2656
G01 X186.6120 Y28.3049
G01 X186.6409 Y28.3286
G01 X186.6892 Y28.3703
G01 X187.0955 Y28.7385
G01 X187.1417 Y28.7825
G01 X187.1682 Y28.8090
G01 X187.2122 Y28.8552
G01 X187.5804 Y29.2615
G01 X187.6221 Y29.3098
G01 X187.6459 Y29.3388
G01 X187.6851 Y29.3891
G01 X188.0118 Y29.8295
G01 X188.0485 Y29.8817
G01 X188.0693 Y29.9128
G01 X188.1034 Y29.9667
G01 X188.3854 Y30.4371
G01 X188.4168 Y30.4926
G01 X188.4345 Y30.5256
G01 X188.4631 Y30.5826
G01 X188.6976 Y31.0783
G01 X188.7235 Y31.1366
G01 X188.7378 Y31.1712
G01 X188.7607 Y31.2307
G01 X188.9455 Y31.7470
G01 X188.9655 Y31.8076
G01 X188.9764 Y31.8434
G01 X188.9934 Y31.9049
G01 X189.1266 Y32.4369
G01 X189.1406 Y32.4991
G01 X189.1479 Y32.5358
G01 X189.1588 Y32.5987
G01 X189.2393 Y33.1411
G01 X189.2471 Y33.2045
G01 X189.2507 Y33.2417
G01 X189.2554 Y33.3053
G01 X189.2824 Y33.8530
G01 X189.2837 Y33.8904
G01 X189.2844 Y33.9247
G01 X189.4228 Y61.2000
G00 Z2.0000
M05
G00 Z2.0000
G00 Z15.00
