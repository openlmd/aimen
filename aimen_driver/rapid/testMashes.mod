MODULE Robpath

    PERS tooldata toolRobpath:=[TRUE,[[351.1,-36.6,86.9],[0.707110,0.000000,-0.707110,0.000000]],[20,[70,30,123.5],[0,0,1,0],1,0,1]];
    PERS wobjdata wobjRobpath:=[FALSE,TRUE,"",[[1655.0,-87.0,932.0],[1.000000,0.000000,0.000000,0.000000]],[[0,0,0],[1,0,0,0]]];

    VAR triggdata laserONRobpath;
    VAR triggdata laserOFFRobpath;

    VAR num layers := 10;
    VAR num n_layer := 0;
    VAR num total_height := 10;
    VAR num layer_height := 0.7;
    VAR bool layer_perp := FALSE;

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


PROC claddingRobpath()
    MoveL Trobpath0, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
    TriggL Trobpath1, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;
    MoveL Trobpath2, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
    TriggL Trobpath3, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;
ENDPROC

PROC claddingRobpathPerp()
  MoveL Trobpath4, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
  TriggL Trobpath5, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;
  MoveL Trobpath6, vRobpathT, z0, toolRobpath \WObj:=wobjRobpath;
  TriggL Trobpath7, vRobpath, laserONRobpath \T2:=laserOFFRobpath, z0, toolRobpath\WObj:=wobjRobpath;
ENDPROC

PROC fillHeight()
    n_layer := 0;
    WHILE total_height > (wobjRobpath.oframe.trans.z + layer_height) DO
      IF layer_perp THEN
        claddingRobpath;
        layer_perp := FALSE;
      ELSE
        claddingRobpathPerp;
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
      IF layer_perp THEN
        claddingRobpath;
        layer_perp := FALSE;
      ELSE
        claddingRobpathPerp;
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
      IF layer_perp THEN
        claddingRobpath;
        layer_perp := FALSE;
      ELSE
        claddingRobpathPerp;
        layer_perp := TRUE;
      ENDIF
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z + layer_height;
      !Correccion de altura debido as desviacions entre a real e a teorica
      wobjRobpath.oframe.trans.z := wobjRobpath.oframe.trans.z - height_compensation;
      n_layer := n_layer + 1;
    ENDWHILE
ENDPROC

PROC mainRobpath()

    Set TdoExtActiv;
    WaitDi TdiExtActiv, 1;
    Set TdoLaserOn;
    WaitDi TdiLaserOn, 1;
    Set TdoStandBy;
    Set TdoActLaser;
    SetGO TGOPROGRAM_No, 28;
    WaitDi TdiLaserAsig, 1;
    WaitTime 2;
    WaitDi TdiLaserReady, 1;

    TriggIO laserONRobpath, 0 \Start \DOp:=TdoPStartStat, 1;
    TriggIO laserOFFRobpath, 0 \DOp:=TdoPStartStat, 0;

    Set DoWeldGas;
    SetAO AoGTV_ExternDisk, 35;
    SetAO AoGTV_ExternMassFlow, 26.6;
    Set doGTV_StartExtern;
    WaitTime 15;

    ConfL \Off;

    MoveJ [[0.0,0.0,100.0],[1.0,0.0,0.0,0.0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v80,z0,toolRobpath\WObj:=wobjRobpath;

    fillHeight;

    MoveJ [[0.0,0.0,100.0],[1.0,0.0,0.0,0.0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v80,z0,toolRobpath\WObj:=wobjRobpath;

    Reset doGTV_StartExtern;
    Reset DoWeldGas;

    Reset TdoActLaser;
    Reset TdoStandBy;
    Reset TdoLaserOn;
    Reset TdoExtActiv;

ENDPROC

ENDMODULE
