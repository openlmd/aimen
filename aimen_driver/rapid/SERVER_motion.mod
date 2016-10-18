MODULE SERVER_motion


LOCAL CONST zonedata DEFAULT_CORNER_DIST := z10;
LOCAL VAR intnum intr_cancel_motion;
LOCAL VAR robtarget pAct;
LOCAL VAR robtarget pActB;
LOCAL VAR robtarget pActC;
!//Control of the laser
VAR triggdata laserON_fl015;
VAR triggdata laserOFF_fl015;

PROC Initialize()
		TriggIO laserON_fl015, 0\DOp:=Do_FL_RayoLaserEnc, 1;
		TriggIO laserOFF_fl015, 0\DOp:=Do_FL_RayoLaserEnc, 0;
	n_cartesian_command := 1;
	n_cartesian_motion := 1;
	!ActUnit STN1;
	!Find the current external axis values so they don't move when we start
	!jointsTarget := CJointT();
	!externalAxis := jointsTarget.extax;
ENDPROC


PROC main()
    VAR jointtarget target;
    VAR zonedata stop_mode;

    !//Motion configuration
    ConfL \Off;
    SingArea \Wrist;
    moveCompleted:= TRUE;

    !//Initialization of WorkObject, Tool, Speed, Zone and Laser
    Initialize;

    ! Set up interrupt
	IDelete intr_cancel_motion;
	CONNECT intr_cancel_motion WITH new_cancel_motion_handler;
	IPers cancel_motion, intr_cancel_motion;

    WHILE true DO
      pAct := CRobT(\Tool:=currentTool \WObj:=currentWObj);
        !Check for new motion command
      IF n_cartesian_command <> n_cartesian_motion THEN
          TEST command_type{n_cartesian_motion}
            CASE 1: !Cartesian linear move
              moveCompleted := FALSE;
              cartesianTarget{n_cartesian_motion}.extax := pAct.extax;
              MoveL cartesianTarget{n_cartesian_motion}, cartesian_speed{n_cartesian_motion}, currentZone, currentTool \WObj:=currentWobj ;
              moveCompleted := TRUE;

            CASE 10: !Cartesian joint move
              moveCompleted := FALSE;
              cartesianTarget{n_cartesian_motion}.extax := pAct.extax;
              MoveJ cartesianTarget{n_cartesian_motion}, cartesian_speed{n_cartesian_motion}, currentZone, currentTool \WObj:=currentWobj ;
              moveCompleted := TRUE;

            CASE 121: !External axis move
              moveCompleted := FALSE;
              pActB := CRobT(\Tool:=currentTool \WObj:=currentWObj);
              pActB.extax.eax_b := extAxisMove{n_cartesian_motion};
              MOVEJ pActB, cartesian_speed{n_cartesian_motion}, currentZone, currentTool \WObj:=currentWobj;
              !IndAMove STN1, 1\ToAbsNum:=cartesianTarget{n_cartesian_motion}.extax.eax_b, cartesian_speed{n_cartesian_motion}.v_reax;
              !IndReset STN1, 1;
							moveCompleted := TRUE;

						CASE 122: !External axis move
							moveCompleted := FALSE;
							pActC := CRobT(\Tool:=currentTool \WObj:=currentWObj);
							pActC.extax.eax_c := extAxisMove{n_cartesian_motion};
							MOVEJ pActC, cartesian_speed{n_cartesian_motion}, currentZone, currentTool \WObj:=currentWobj;
							!IndAMove STN1, 2\ToAbsNum:=cartesianTarget{n_cartesian_motion}.extax.eax_b, cartesian_speed{n_cartesian_motion}.v_reax;
							!IndReset STN1, 2;
							moveCompleted := TRUE;

            CASE 110: !Trigger linear OFF
              moveCompleted := FALSE;
              cartesianTarget{n_cartesian_motion}.extax := pAct.extax;
              TriggL cartesianTarget{n_cartesian_motion}, cartesian_speed{n_cartesian_motion}, laserOFF_fl015, currentZone, currentTool \WObj:=currentWobj ;
							moveCompleted := TRUE;

            CASE 111: !Trigger linear ON
              moveCompleted := FALSE;
              cartesianTarget{n_cartesian_motion}.extax := pAct.extax;
              TriggL cartesianTarget{n_cartesian_motion}, cartesian_speed{n_cartesian_motion}, laserON_fl015, currentZone, currentTool \WObj:=currentWobj ;
							moveCompleted := TRUE;

						CASE 930: !WaitDI
							IF commandSetDO{n_cartesian_motion} THEN
								WaitDI Di_FL_EstadBy, 1;
							ELSE
								WaitDI Di_FL_EstadBy, 0;
							ENDIF

						CASE 931: !WaitDI
							IF commandSetDO{n_cartesian_motion} THEN
								WaitDI Di_FL_ErrorLaserApagado, 1;
							ELSE
								WaitDI Di_FL_ErrorLaserApagado, 0;
							ENDIF

						CASE 94: !Wait time
							WaitTime numBufferAux{n_cartesian_motion};

						CASE 950: !
							SetGO GO_FL_Programa, numBufferAux{n_cartesian_motion};

						CASE 951: !
							SetGO GO_FL_PotenciaLaser1, numBufferAux{n_cartesian_motion};

						CASE 960: !
							SetAO AoGTV_ExternDisk, numBufferAux{n_cartesian_motion};

						CASE 961: !
							SetAO AoGTV_ExternMassflow, numBufferAux{n_cartesian_motion};

						CASE 970: !Set DO gtv START
							IF commandSetDO{n_cartesian_motion} THEN
								SetDO doGTV_StartExtern, 1;
							ELSE
								SetDO doGTV_StartExtern, 0;
							ENDIF

						CASE 971: !Set DO gtv STOP
							IF commandSetDO{n_cartesian_motion} THEN
								SetDO doGTV_Stop, 1;
							ELSE
								SetDO doGTV_Stop, 0;
							ENDIF

						CASE 972: !Set DO
							IF commandSetDO{n_cartesian_motion} THEN
								SetDO Do_FL_RedENC, 1;
							ELSE
								SetDO Do_FL_RedENC, 0;
							ENDIF

						CASE 973: !Set DO
							IF commandSetDO{n_cartesian_motion} THEN
								SetDO Do_FL_StandByEnc, 1;
							ELSE
								SetDO Do_FL_StandByEnc, 0;
							ENDIF

						CASE 974: !Set DO
							IF commandSetDO{n_cartesian_motion} THEN
								SetDO DoWeldGas, 1;
							ELSE
								SetDO DoWeldGas, 0;
							ENDIF

						CASE 975: !Set DO
							IF commandSetDO{n_cartesian_motion} THEN
								SetDO DoRootGas, 1;
							ELSE
								SetDO DoRootGas, 0;
							ENDIF

            DEFAULT:
				TPWrite "SERVER_motion: Illegal instruction code: ", \Num:=command_type{n_cartesian_motion};
          ENDTEST
          n_cartesian_motion := n_cartesian_motion + 1;
          IF n_cartesian_motion > 49
            n_cartesian_motion := 1;
        ENDIF
        WaitTime 0.1;  ! Throttle loop while waiting for new command
    ENDWHILE
ERROR
	TEST ERRNO
			CASE ERR_NORUNUNIT:
					TPWrite "MOTION: No contact with unit.";
					TRYNEXT;
			DEFAULT:
					ErrWrite \W, "Motion Error", "Error executing motion.  Aborting trajectory.";
					abort_trajectory;
	ENDTEST
ENDPROC

LOCAL PROC abort_trajectory()
    clear_path;
    ExitCycle;  ! restart program
ENDPROC

LOCAL PROC clear_path()
    IF ( NOT (IsStopMoveAct(\FromMoveTask) OR IsStopMoveAct(\FromNonMoveTask)) )
        StopMove;          ! stop any active motions
    StartMove;             ! re-enable motions
ENDPROC

LOCAL TRAP new_cancel_motion_handler
	IF (NOT cancel_motion) RETURN;
	! Reset
	cancel_motion := FALSE;
	n_cartesian_motion := n_cartesian_command;
	SetDO Do_FL_RayoLaserEnc, 0;
	SetDO doGTV_StartExtern, 0;
	SetDO doGTV_Stop, 0;
	SetDO Do_FL_RedENC, 0;
	SetDO Do_FL_StandByEnc, 0;
	abort_trajectory;
	ERROR
		TEST ERRNO
				CASE ERR_NORUNUNIT:
						TPWrite "MOTION: No contact with unit.";
						TRYNEXT;
				DEFAULT:
						ErrWrite \W, "Motion Error", "Error executing motion.  Aborting trajectory.";
						abort_trajectory;
		ENDTEST
ENDTRAP

ENDMODULE
