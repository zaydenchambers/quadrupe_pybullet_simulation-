# Part measurements (STEP 1)

All values in mm, in each STL's own print-bed frame. `proximal (heuristic)` is the end with the larger cross-section at 15 % / 85 % of the principal axis; it is only reported for elongated parts and is cross-checked against the functional features in README.md.

| part | bbox (print) | PCA extents | volume cm³ | watertight | xsec 15% / 85% | proximal end (heuristic) |
|---|---|---|---|---|---|---|
| CalibrationTool | [np.float64(119.42), np.float64(110.57), np.float64(8.0)] | [np.float64(150.93), np.float64(107.48), np.float64(9.62)] | 21.61 | False |  |  |
| Cover_BottomFront | [np.float64(114.01), np.float64(31.63), np.float64(142.29)] | [np.float64(118.76), np.float64(145.64), np.float64(48.71)] | 32.74 | True |  |  |
| Cover_BottomHatch | [np.float64(112.46), np.float64(107.4), np.float64(185.62)] | [np.float64(212.99), np.float64(108.17), np.float64(22.8)] | 100.74 | True | 334.5 / 322.7 | [np.float64(50.6), np.float64(-0.0), np.float64(3.9)] |
| Cover_BottomRear | [np.float64(114.01), np.float64(44.4), np.float64(146.06)] | [np.float64(137.64), np.float64(157.23), np.float64(58.53)] | 39.55 | True |  |  |
| Cover_Emblem | [np.float64(9.6), np.float64(43.5), np.float64(4.65)] | [np.float64(43.51), np.float64(9.32), np.float64(4.78)] | 1.05 | True | 17.2 / 29.4 | [np.float64(-53.6), np.float64(20.9), np.float64(3.0)] |
| Cover_HeadPanel | [np.float64(115.75), np.float64(97.34), np.float64(89.51)] | [np.float64(119.49), np.float64(91.4), np.float64(94.1)] | 54.3 | True |  |  |
| Cover_HeadPanelSlug | [np.float64(30.19), np.float64(29.74), np.float64(13.29)] | [np.float64(29.79), np.float64(29.58), np.float64(16.46)] | 2.92 | True |  |  |
| Cover_LeftFemur | [np.float64(156.79), np.float64(55.47), np.float64(40.45)] | [np.float64(162.77), np.float64(60.83), np.float64(43.38)] | 33.26 | True | 4.5 / 304.9 | [np.float64(-79.2), np.float64(-17.3), np.float64(19.7)] |
| Cover_RearPanelBtnHolder_6x6LongBtn | [np.float64(46.95), np.float64(18.62), np.float64(8.23)] | [np.float64(48.56), np.float64(23.25), np.float64(8.46)] | 2.15 | True | 27.8 / 51.5 | [np.float64(-49.7), np.float64(2.7), np.float64(-7.5)] |
| Cover_RearPanel_2Piece_Base | [np.float64(61.02), np.float64(99.57), np.float64(6.04)] | [np.float64(99.79), np.float64(61.58), np.float64(7.02)] | 9.56 | True |  |  |
| Cover_RearPanel_2Piece_Shell | [np.float64(74.83), np.float64(116.05), np.float64(70.25)] | [np.float64(117.28), np.float64(83.03), np.float64(79.95)] | 41.24 | True |  |  |
| Cover_RightFemur | [np.float64(156.79), np.float64(55.47), np.float64(40.45)] | [np.float64(162.75), np.float64(60.43), np.float64(43.5)] | 33.24 | True | 107.2 / 304.7 | [np.float64(-79.2), np.float64(17.3), np.float64(19.7)] |
| Cover_TopFront | [np.float64(115.06), np.float64(44.6), np.float64(150.33)] | [np.float64(161.44), np.float64(162.06), np.float64(61.45)] | 44.38 | True |  |  |
| Cover_TopRear | [np.float64(119.1), np.float64(32.09), np.float64(146.0)] | [np.float64(157.06), np.float64(136.18), np.float64(40.44)] | 40.94 | True |  |  |
| Electronics_BuckConveterCase | [np.float64(55.3), np.float64(18.75), np.float64(33.61)] | [np.float64(60.78), np.float64(44.47), np.float64(19.71)] | 8.44 | False |  |  |
| Foot | [np.float64(37.0), np.float64(20.0), np.float64(20.0)] | [np.float64(36.99), np.float64(20.84), np.float64(20.57)] | 3.59 | True |  |  |
| Frame_ChassisTrunk | [np.float64(127.0), np.float64(110.0), np.float64(46.91)] | [np.float64(145.81), np.float64(156.6), np.float64(50.75)] | 89.69 | True |  |  |
| Frame_FrontShoulderInner | [np.float64(110.04), np.float64(70.0), np.float64(48.98)] | [np.float64(110.06), np.float64(73.12), np.float64(53.29)] | 59.65 | True |  |  |
| Frame_FrontShoulderMiddle | [np.float64(110.0), np.float64(52.5), np.float64(70.0)] | [np.float64(110.13), np.float64(72.12), np.float64(57.88)] | 52.02 | True |  |  |
| Frame_FrontShoulderOuter | [np.float64(110.0), np.float64(66.0), np.float64(6.0)] | [np.float64(111.26), np.float64(70.23), np.float64(6.85)] | 22.48 | True |  |  |
| Frame_LeftCoax | [np.float64(37.59), np.float64(46.13), np.float64(57.5)] | [np.float64(61.7), np.float64(58.16), np.float64(47.39)] | 23.88 | True |  |  |
| Frame_LeftFemur | [np.float64(138.54), np.float64(51.09), np.float64(34.66)] | [np.float64(143.65), np.float64(70.0), np.float64(39.25)] | 66.02 | True | 168.0 / 317.3 | [np.float64(-71.6), np.float64(15.3), np.float64(17.3)] |
| Frame_LeftTibia | [np.float64(129.35), np.float64(100.09), np.float64(38.48)] | [np.float64(151.97), np.float64(40.73), np.float64(32.82)] | 30.27 | True | 105.0 / 247.4 | [np.float64(-62.9), np.float64(-42.1), np.float64(17.3)] |
| Frame_RearShoulderInner | [np.float64(110.0), np.float64(70.0), np.float64(48.98)] | [np.float64(110.64), np.float64(72.85), np.float64(50.88)] | 50.25 | True |  |  |
| Frame_RearShoulderMiddle | [np.float64(110.0), np.float64(70.0), np.float64(6.0)] | [np.float64(110.72), np.float64(72.23), np.float64(7.48)] | 21.07 | True |  |  |
| Frame_RearShoulderOuter | [np.float64(110.0), np.float64(70.0), np.float64(52.5)] | [np.float64(79.89), np.float64(111.41), np.float64(64.61)] | 54.91 | True |  |  |
| Frame_RightCoax | [np.float64(37.51), np.float64(46.14), np.float64(57.5)] | [np.float64(60.87), np.float64(57.41), np.float64(49.91)] | 23.93 | True |  |  |
| Frame_RightFemur | [np.float64(138.53), np.float64(51.15), np.float64(34.66)] | [np.float64(143.91), np.float64(68.14), np.float64(39.61)] | 66.01 | True | 166.8 / 313.3 | [np.float64(-71.5), np.float64(-15.1), np.float64(17.6)] |
| Frame_RightTibia | [np.float64(151.05), np.float64(31.29), np.float64(38.49)] | [np.float64(151.16), np.float64(40.3), np.float64(32.27)] | 30.25 | True | 228.9 / 105.1 | [np.float64(75.2), np.float64(0.1), np.float64(16.9)] |
| Mp3ProSpeakerMount | [np.float64(68.05), np.float64(43.0), np.float64(21.6)] | [np.float64(70.85), np.float64(43.18), np.float64(22.55)] | 15.81 | True |  |  |
| Sensor_FrontPIRMount | [np.float64(50.38), np.float64(29.83), np.float64(8.05)] | [np.float64(50.38), np.float64(30.48), np.float64(8.95)] | 3.94 | True |  |  |
| Sensor_MP3SpeakerMount | [np.float64(68.05), np.float64(21.6), np.float64(43.0)] | [np.float64(70.85), np.float64(43.57), np.float64(22.34)] | 16.58 | True |  |  |
| Sensor_PIRCup | [np.float64(19.97), np.float64(20.02), np.float64(13.5)] | [np.float64(20.58), np.float64(24.1), np.float64(22.68)] | 1.14 | True |  |  |
| Sensor_RearPIRMount | [np.float64(64.02), np.float64(24.05), np.float64(6.1)] | [np.float64(64.02), np.float64(24.08), np.float64(6.19)] | 4.22 | True | 35.6 / 35.6 | [np.float64(-44.3), np.float64(-15.6), np.float64(5.4)] |
| Sensor_UltraSonicPlate | [np.float64(57.59), np.float64(58.62), np.float64(12.15)] | [np.float64(64.42), np.float64(67.17), np.float64(12.57)] | 8.7 | False |  |  |
| Stand_Cone | [np.float64(139.67), np.float64(140.0), np.float64(240.0)] | [np.float64(240.43), np.float64(140.0), np.float64(139.81)] | 307.22 | True |  |  |
| Stand_ConeBase | [np.float64(220.0), np.float64(160.0), np.float64(11.0)] | [np.float64(220.0), np.float64(160.0), np.float64(11.0)] | 148.62 | True |  |  |
| Stand_ConeCap | [np.float64(113.29), np.float64(145.4), np.float64(61.65)] | [np.float64(145.5), np.float64(113.52), np.float64(61.91)] | 122.8 | True |  |  |

## Key functional features

- Coax roll pivot peg Ø8.00 at print (X,Z)=[-8.68, 17.5]; pitch horn pattern centre (Y,Z)=[2.31, 28.05]; roll↔pitch perpendicular offset 10.55; servo shaft offset from ear centre 10.75
- Femur knee horn centre (X,Z)=[-56.72, 22.05]; servo ear pairs at [16.22, 12.16] and [64.72, 14.66] (spacing 48.57, tilt 2.96°)
- Tibia knee pin Ø8.00 at (X,Y)=[55.5, -2.86]; tip arc centre [-60.45, 0.58] r=14.99; knee→tip-arc 116.00; pin offset from servo centre 10.24
- Head tab normals differ by 0.14°, rear-panel tab normals by 0.19° (parallel ⇒ one rigid mounting orientation)

## Mirror pairs (ICP after reflecting across print Y)

- Frame_RightCoax → Frame_LeftCoax: mean 0.012 mm, p95 0.013 mm
- Frame_RightFemur → Frame_LeftFemur: mean 0.005 mm, p95 0.011 mm
- Frame_LeftTibia → Frame_RightTibia: mean 0.009 mm, p95 0.026 mm
- Cover_RightFemur → Cover_LeftFemur: mean 0.005 mm, p95 0.007 mm
