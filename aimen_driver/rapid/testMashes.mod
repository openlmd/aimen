MODULE Robpath

    PERS tooldata toolRobpath:=[TRUE,[[351.1,-36.6,86.9],[0.707110,0.000000,-0.707110,0.000000]],[20,[70,30,123.5],[0,0,1,0],1,0,1]];
    PERS wobjdata wobjRobpath:=[FALSE,TRUE,"",[[1655.0,-87.0,932.0],[1.000000,0.000000,0.000000,0.000000]],[[0,0,0],[1,0,0,0]]];

    VAR triggdata laserONRobpath;
    VAR triggdata laserOFFRobpath;

    VAR num layers := 10;
    VAR num n_layer := 0;
    VAR num total_height := 10;
    VAR num layer_height := 0.5;
    VAR bool layer_perp := FALSE;
    VAR num scanning_layer := 1;
    VAR num last_scan := 0;

    CONST speeddata vRobpath:=[8,500,5000,1000];
    CONST speeddata vRobpathT:=[25,500,5000,1000];


    VAR robtarget Trobpath0:=[[0,0,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget Trobpath1:=[[20,0,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget Trobpath2:=[[0,5,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget Trobpath3:=[[20,5,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    VAR robtarget Trobpath4:=[[0,0,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget Trobpath5:=[[0,20,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget Trobpath6:=[[5,0,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget Trobpath7:=[[5,20,0],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    !Punto vaso
    VAR robtarget TrobpathReposo:=[[-100,-100,110],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    !Puntos de escaneo
    VAR robtarget TrobpathEi:=[[0,-50,30],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget TrobpathEf:=[[0,50,30],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];


PROC claddingRobpath()
    last_scan := last_scan + 1;

    MoveL Trobpath0, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
    TriggL Trobpath1, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;
    MoveL Trobpath2, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
    TriggL Trobpath3, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;

    scanning;
ENDPROC

PROC claddingRobpathB()
  last_scan := last_scan + 1;

  MoveL Trobpath4, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
  TriggL Trobpath5, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;
  MoveL Trobpath6, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
  TriggL Trobpath7, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;

  scanning;
ENDPROC

PROC scanning()
    IF (scanning_layer <> 0) AND (scanning_layer = last_scan) THEN
      last_scan := 0;
      MoveL TrobpathReposo, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
      stopPowder;
      TPWrite "Stopping powder feeder";
      Stop \NoRegain;
      MoveL TrobpathEi, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
      TPWrite "Ready for scanning";
      Stop \NoRegain;
      MoveL TrobpathEf, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
      TPWrite "Scanning finished";
      Stop \NoRegain;
      MoveL TrobpathReposo, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
      startPowder;
    ENDIF
ENDPROC

PROC fillHeight()
    n_layer := 0;
    WHILE total_height > (wobjRobpath.oframe.trans.z + layer_height) DO
      TPWrite "Processing layer: ", \Num:=n_layer;
      TPWrite "Tool height: ", \Num:=wobjRobpath.oframe.trans.z;
      IF layer_perp THEN
        claddingRobpath;
        layer_perp := FALSE;
      ELSE
        claddingRobpathB;
        layer_perp := TRUE;
      ENDIF
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z + layer_height;
      !Correccion de altura debido as desviacions entre a real e a teorica
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z - height_compensation;
      n_layer := n_layer + 1;
    ENDWHILE
ENDPROC

PROC fillLayers()
    n_layer := 0;
    WHILE n_layer < layers DO
      TPWrite "Processing layer: ", \Num:=n_layer;
      TPWrite "Tool height: ", \Num:=wobjRobpath.oframe.trans.z;
      IF layer_perp THEN
        claddingRobpath;
        layer_perp := FALSE;
      ELSE
        claddingRobpathB;
        layer_perp := TRUE;
      ENDIF
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z + layer_height;
      !Correccion de altura debido as desviacions entre a real e a teorica
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z - height_compensation;
      n_layer := n_layer + 1;
    ENDWHILE
ENDPROC

PROC fillWStop()
    n_layer := 0;
    WHILE not stop_layer DO
      TPWrite "Processing layer: ", \Num:=n_layer;
      TPWrite "Tool height: ", \Num:=wobjRobpath.oframe.trans.z;
      IF layer_perp THEN
        claddingRobpath;
        layer_perp := FALSE;
      ELSE
        claddingRobpathB;
        layer_perp := TRUE;
      ENDIF
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z + layer_height;
      !Correccion de altura debido as desviacions entre a real e a teorica
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z - height_compensation;
      n_layer := n_layer + 1;
    ENDWHILE
ENDPROC

PROC startPowder()
    Set DoWeldGas;
    SetAO AoGTV_ExternDisk, 35;
    SetAO AoGTV_ExternMassFlow, 26.6;
    Set doGTV_StartExtern;
    WaitTime 15;
ENDPROC

PROC stopPowder()
  Reset doGTV_StartExtern;
  Reset DoWeldGas;
ENDPROC


PROC mainRobpath()
    SetDO Do_FL_RedENC, 1;
    SetDO Do_FL_StandByEnc, 1;
    WaitDI Di_FL_EstadBy, 1;
    WaitDI Di_FL_ErrorLaserApagado, 0;

    SetGO GO_FL_Programa, 0; !set the program for control of laser power - prog 5
    !SetGO GO_FL_PotenciaLaser1, 1000;
    WaitTime 1;

    TriggIO laserONRobpath, 0 \Start \DOp:=Do_FL_RayoLaserEnc, 1;
    TriggIO laserOFFRobpath, 0 \DOp:=Do_FL_RayoLaserEnc, 0;

    startPowder;

    ConfL \Off;

    MoveJ TrobpathReposo, vRobpathT ,z0,toolRobpath\WObj:=wobjRobpath;

    fillHeight;

    MoveJ TrobpathReposo, vRobpathT, z0,toolRobpath\WObj:=wobjRobpath;

    stopPowder;

    SetDO Do_FL_StandByEnc, 0;

ENDPROC

ENDMODULE
