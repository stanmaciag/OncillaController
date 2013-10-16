package OncillaClient;

import java.io.*;
import java.net.*;

/**
 * OncillaClient is utility that allows user to remotely control the Oncilla robot with working OncillaServer program.
 * After constructing the OncillaClient object following actions are possible:
 * <ul>
 * <li>connect to the robot (to the OncillaServer)</li>
 * <li>disconnect from the robot</li>
 * <li>check connection status</li>
 * <li>initialize the robot</li>
 * <li>shutdown robot</li>
 * <li>shutdown server</li>
 * <li>set desired position of the joints</li>
 * <li>get current position of the joints</li>
 * <li>get next point of recorded trajectory</li>
 * <li>reset trajectory record</li>
 * </ul>
 * @author Stanislaw Maciag maciag@student.agh.edu.pl
 *
 */
public class OncillaClient {

	/**
	 * Definition of available commands that are sent to the robot. List of commands:
	 * <ul>
	 * <li>CMD_EXIT - shutdown the server</li>
	 * <li>CMD_INIT - initialize the robot</li>
	 * <li>CMD_SHUTDOWN - shutdown the robot</li>
	 * <li>CMD_SET_POS - set new positions of the joints</li>
	 * <li>CMD_GET_POS - get current position of the joints</li>
	 * <li>CMD_PING - test connection</li>
	 * <li>CMD_GET_NEXT_POINT - get next point of the recorded trajectory</li>
	 * <li>CMD_RESET - reset trajectory record</li>
	 * </ul>
	 * Byte values, which are used in communication, are assigned to enum values.
	 */
	public enum Commands {
		CMD_EXIT((byte) 0x00), CMD_INIT((byte) 0x01), CMD_SHUTDOWN((byte) 0x02), CMD_SET_POS(
				(byte) 0x04), CMD_GET_POS((byte) 0x05), CMD_PING((byte) 0x06), CMD_GET_NEXT_POINT(
				(byte) 0x09), CMD_RESET((byte) 0x10);

		private byte byteVal;

		Commands(byte byteVal) {
			this.byteVal = byteVal;
		}

		public byte getByteVal() {
			return byteVal;
		}

	};

	/**
	 * Communication buffer
	 */
	private char[] buf = new char[1500];
	/**
	 * Client socket
	 */
	private Socket clientSocket;
	/**
	 * Output data stream
	 */
	private OutputStream outputStream;
	// private Queue<String> messageQueue;
	/**
	 * Connection state
	 */
	private boolean isConnected;
	// private Thread listener;
	/**
	 * Input data stream
	 */
	private InputStream inputStream;
	/**
	 * Reader for the input data stream
	 */
	private BufferedReader inputReader;

	/**
	 * Default constructor
	 */
	public OncillaClient() {
		isConnected = false;

	}

	/**
	 * Try connecting to the server
	 * @param host IP address of the server
	 * @param port Port number for the service
	 * @return True if the connection is established, false otherwise
	 */
	public boolean connect(String host, int port) {
		try {
			clientSocket = new Socket(host, port);
		} catch (UnknownHostException e) {
			return false;
		} catch (IOException e) {
			return false;
		}
		try {
			outputStream = clientSocket.getOutputStream();
		} catch (IOException e) {
			return false;
		}
		try {
			inputStream = clientSocket.getInputStream();
			inputReader = new BufferedReader(new InputStreamReader(inputStream));
		} catch (IOException e) {
			return false;
		}
		try {
			clientSocket.setSoTimeout(50);
		} catch (SocketException e) {

			return false;
		}

		isConnected = true;

		return true;
	}

	/**
	 * End connection to the server
	 * @throws IOException
	 * @throws InterruptedException
	 */
	public void disconnect() throws IOException, InterruptedException {
		isConnected = false;
		clientSocket.close();
		outputStream.close();
		inputStream.close();

	}

	/**
	 * Send data to the server
	 * @param cmd Outgoing data
	 */
	private void send(byte[] cmd) {

		try {
			outputStream.write(cmd);
		} catch (IOException e) {
			System.exit(0);
		}

	}

	/**
	 * Connection state getter
	 * @return Connection state
	 */
	public synchronized boolean isConnected() {
		return isConnected;
	}

	/**
	 * Initialize the robot by sending CMD_INIT command to the server. Should be called before calling:
	 * <ul>
	 * <li>shutdownRobot()</li>
	 * <li>setPosRad()</li>
	 * <li>setPosDeg()</li>
	 * <li>getPosRad()</li>
	 * <li>getPosDeg()</li>
	 * <li>getNextTrajectPointRad()</li>
	 * <li>readTraject()</li>
	 * <li>resetTrajectRec()</li>
	 * </ul>
	 */
	public void init() {

		byte curCmd[] = { Commands.CMD_INIT.getByteVal() };
		send(curCmd);

		try {
			clientSocket.setSoTimeout(60000);
			inputReader.read(buf);
			clientSocket.setSoTimeout(50);

		} catch (IOException e) {

			e.printStackTrace();
		}
	}

	/**
	 * Shutdowns the server by sending CMD_EXIT command to the server
	 */
	public void shutdownServer() {

		byte curCmd[] = { Commands.CMD_EXIT.getByteVal() };

		send(curCmd);

	}

	/**
	 * Shutdowns the server by sending CMD_SHUTDOWN command to the server
	 */
	public void shutdownRobot() {

		byte curCmd[] = { Commands.CMD_SHUTDOWN.getByteVal() };

		send(curCmd);

	}

	/**
	 * Sets the target position of the joints by sending CMD_SET_POS command with arguments
	 * @param pos Target positions of the joints. Elements description:
	 * <ul>
	 * <li>pos[0] - left fore hip joint angle position in radians</li>
	 * <li>pos[1] - left fore knee joint position described by bend coefficient</li>
	 * <li>pos[2] - right fore hip joint angle position in radians</li>
	 * <li>pos[3] - right fore knee joint position described by bend coefficient</li>
	 * <li>pos[4] - left hind hip joint angle position in radians</li>
	 * <li>pos[5] - left hind knee joint position described by bend coefficient</li>
	 * <li>pos[6] - right hind hip joint angle position in radians</li>
	 * <li>pos[7] - right hind knee joint position described by bend coefficient</li>
	 * <li>pos[8] - left fore servo position</li>
	 * <li>pos[9] - right fore servo position</li>
	 * <li>pos[10] - left hind servo position</li>
	 * <li>pos[11] - right hind servo position</li>
	 * </ul>
	 * @return False on error, true otherwise
	 */
	public boolean setPosRad(float[] pos) {

		if (pos.length != 12)
			return false;

		String curCmdStr = new String();

		for (int i = 0; i < 12; i++) {

			curCmdStr += " ";
			curCmdStr += Float.toString(pos[i]);

		}

		if (curCmdStr.length() % 2 != 0)
			curCmdStr += '\0';

		try {
			byte[] curCmdArgs;
			curCmdArgs = curCmdStr.getBytes("ASCII");
			byte[] curCmd = new byte[curCmdArgs.length + 1];
			System.arraycopy(curCmdArgs, 0, curCmd, 1, curCmdArgs.length);
			curCmd[0] = Commands.CMD_SET_POS.getByteVal();
			send(curCmd);
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
			return false;
		}

		return true;

	}

	/**
	 * Sets the target position of the joints by sending CMD_SET_POS command with arguments
	 * @param pos Target positions of the joints. Elements description:
	 * <ul>
	 * <li>pos[0] - left fore hip joint angle position in degrees</li>
	 * <li>pos[1] - left fore knee joint position described by bend coefficient</li>
	 * <li>pos[2] - right fore hip joint angle position in degrees</li>
	 * <li>pos[3] - right fore knee joint position described by bend coefficient</li>
	 * <li>pos[4] - left hind hip joint angle position in degrees</li>
	 * <li>pos[5] - left hind knee joint position described by bend coefficient</li>
	 * <li>pos[6] - right hind hip joint angle position in degrees</li>
	 * <li>pos[7] - right hind knee joint position described by bend coefficient</li>
	 * <li>pos[8] - left fore servo position</li>
	 * <li>pos[9] - right fore servo position</li>
	 * <li>pos[10] - left hind servo position</li>
	 * <li>pos[11] - right hind servo position</li>
	 * </ul> 
	 * @return False on error, true otherwise
	 */
	public boolean setPosDeg(float[] pos) {

		if (pos.length != 12)
			return false;

		for (int i = 0; i < 8; i += 2)
			pos[i] = (float) (pos[i] * java.lang.Math.PI / 180.0);

		return setPosRad(pos);

	}

	/**
	 * Gets the last positions of the joints
	 * @return Array of joints positions, elements identical like in setPosRad()
	 */
	public float[] getPosRad() {

		byte curCmd[] = { Commands.CMD_GET_POS.getByteVal() };

		send(curCmd);

		try {

			inputReader.read(buf);
			String response = new String(buf);
			String[] posStr = response.split(" ");
			float[] pos = new float[posStr.length];

			for (int i = 0; i < posStr.length; i++) {

				pos[i] = Float.parseFloat(posStr[i]);

			}

			return pos;

		} catch (IOException e) {

			// e.printStackTrace();
		}
		return null;
	}

	/**
	 * Gets the last positions of the joints
	 * @return Array of joints positions, elements identical like in setPosDeg()
	 */
	public float[] getPosDeg() {

		float[] curPos = getPosRad();

		for (int i = 0; i < 8; i += 2)
			curPos[i] = (float) (curPos[i] * 180.0 / java.lang.Math.PI);

		return curPos;
	}

	/**
	 * Gets the next recorded point of the joints trajectory
	 * @return Trajectory point, elements description:
	 * <ul>
	 * <li>0 - time in ms</li>
	 * <li>1 - left fore hip joint angle position in radians</li>
	 * <li>2 - left fore knee joint position described by bend coefficient</li>
	 * <li>3 - right fore hip joint angle position in radians</li>
	 * <li>4 - right fore knee joint position described by bend coefficient</li>
	 * <li>5 - left hind hip joint angle position in radians</li>
	 * <li>6 - left hind knee joint position described by bend coefficient</li>
	 * <li>7 - right hind hip joint angle position in radians</li>
	 * <li>8 - right hind knee joint position described by bend coefficient</li>
	 * <li>9 - left fore servo position</li>
	 * <li>10 - right fore servo position</li>
	 * <li>11 - left hind servo position</li>
	 * <li>12 - right hind servo position</li>
	 * </ul>
	 */
	public float[] getNextTrajectPointRad() {

		byte curCmd[] = { Commands.CMD_GET_NEXT_POINT.getByteVal() };

		send(curCmd);

		try {

			inputReader.read(buf);
			String response = new String(buf);
			String[] posStr = response.split(" ");
			float[] pos = new float[posStr.length];
			if (posStr.length != 13)
				return null;
			for (int i = 0; i < posStr.length; i++) {

				pos[i] = Float.parseFloat(posStr[i]);

			}

			return pos;

		} catch (IOException e) {

			// e.printStackTrace();
		}
		return null;
	}

	/**
	 * Reads all recorded trajectory
	 * @param size Expected number of samples
	 * @return Array sized size x 13 - rows are single trajectory points 
	 */
	public float[][] readTraject(int size) {

		float[][] newTraject = new float[size][13];

		for (int i = 0; i < size; i++) {

			float[] newPoint = getNextTrajectPointRad();

			if (newPoint != null)
				newTraject[i] = newPoint;
			else
				i--;
		}

		return newTraject;

	}

	/**
	 * Resets trajectory record, sets time to 0
	 */
	public void resetTrajectRec() {

		byte curCmd[] = { Commands.CMD_RESET.getByteVal() };

		send(curCmd);

	}

	/**
	 * Tests connection
	 * @return Latency in ms
	 */
	public long ping() {

		long startTime = System.currentTimeMillis();

		byte curCmd[] = { Commands.CMD_PING.getByteVal() };

		send(curCmd);
		try {

			inputReader.read(buf);

		} catch (IOException e) {

			e.printStackTrace();
		}

		long stopTime = System.currentTimeMillis();
		return stopTime - startTime;
	}

}
