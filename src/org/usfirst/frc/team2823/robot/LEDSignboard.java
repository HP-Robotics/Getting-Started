package org.usfirst.frc.team2823.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class LEDSignboard {
	public static final String UP_ARROW = "up.ppm";
	public static final String DOWN_ARROW = "down.ppm";
	
	private static final LEDSignboard singleton;
	static {
		LEDSignboard value = null;
		try {
			value = new LEDSignboard();
		} catch (IOException e) {
			System.err.println("Unable to connect to PI");
			e.printStackTrace();
		}
		singleton = value;
	}

	private final InetAddress pi;
	private final DatagramSocket serverSocket;

	private LEDSignboard() throws IOException {
		pi = InetAddress.getByName("raspberry-pi.local");
		serverSocket = new DatagramSocket(9876);
	}

	public static void sendTextMessage(String message) {
		rawCommand("text " + message);
	}
	
	public static void sendFile(String filename) {
		rawCommand("file " + filename);
	}
	
	public static void rawCommand(String command) {
		if (singleton != null) {
			try {
				singleton.udp(command);
			} catch (IOException e) {
				System.err.println("Error sending message to pi");
				e.printStackTrace();
			}
		} else {
			System.err.println("WARNING: No raspberry pi connection");
		}
	}

	private void udp(String command) throws IOException {
		byte[] sendData = command.getBytes();
		DatagramPacket sendPacket = new DatagramPacket(sendData,
				sendData.length, pi, 5201);
		serverSocket.send(sendPacket);
		System.out.println(command);
	}
}
