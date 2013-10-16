/**
 * @file OncillaCmd.h
 * @author Stanislaw Maciag maciag@student.agh.edu.pl
 * @brief Input command for OncillaRobot
 */

#ifndef ONCILLACMD_H
#define	ONCILLACMD_H

/**
 * Command for OncillaRobot object
 */
class OncillaCmd {

	/**
	 * Union which stores command arguments
	 */
	union cmdArgs {
		double doubleArgs[12];
		int16_t int16_tArgs[12];

	};

public:
	/**
	 * Command ID, defined by OncillaCmd::ID.
	 */
	char id;
	/**
	 * Command arguments
	 */
	cmdArgs args;

	/**
	 * Enum type for command ID identification.
	 * Currently implemented commands:
	 * 		- SET_POS - set new target position for robots joints, arguments type is double
	 * 		- RESET_TIMER - reset internal robots timer and clear recorded trajectory
	 */
	enum ID {
		SET_POS, RESET_TIMER

	};

	/**
	 * Constructor for double type arguments.
	 * @param id New command ID
	 * @param doubleArgs Pointer for new command arguments of type double, must be valid
	 */
	OncillaCmd(ID id, double doubleArgs[]) :
			id(id) {

		memcpy(args.doubleArgs, doubleArgs, 12 * sizeof(double));

	}

	/**
	 * Constructor for int16_t type arguments.
	 * @param id New command ID
	 * @param int16_tArgs New command arguments of type int16_t, must be valid
	 */
	OncillaCmd(ID id, int16_t int16_tArgs[]) :
			id(id) {

		memcpy(args.int16_tArgs, int16_tArgs, 12 * sizeof(int16_t));

	}

	/**
	 * Constructor for command without arguments.
	 * @param id New command ID
	 */
	OncillaCmd(ID id) :
			id(id) {

	}

	/**
	 * Default constructor for empty command.
	 */
	OncillaCmd() :
			id(-1) {

	}

};

#endif	/* ONCILLACMD_H */

