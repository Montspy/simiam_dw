package edu.gatech.gritslab;

/*
 * Install RXTX java library
 */

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.FileDescriptor;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

// Copyright (C) 2014, Georgia Tech Research Corporation
// see the LICENSE file included with this software

public class GritsBotConnector {
	private InputStream inputStream_;
	private OutputStream outputStream_;
	private SerialPort serialPort_;
	private int robotID_;
	
	private BufferedReader inReader_;

	public GritsBotConnector(String portName, int robotID) {
		robotID_ = robotID;
		
		try {
			CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
			
			if ( portIdentifier.isCurrentlyOwned() )
			{
				System.out.println("Error: Port is currently in use");
			}
			else
			{
				CommPort commPort = portIdentifier.open(this.getClass().getName(),2000);
			
				if ( commPort instanceof SerialPort )
				{
					serialPort_ = (SerialPort) commPort;
					serialPort_.setSerialPortParams(115200,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
					serialPort_.enableReceiveTimeout(500);	// 5ms RX timeout
					
					inputStream_ = serialPort_.getInputStream();
					outputStream_ = serialPort_.getOutputStream();
					
					inReader_ = new BufferedReader(new InputStreamReader(inputStream_));
				}
				else
				{
					System.out.println("Error: Only serial ports are handled by this example.");
				}
			} 
		}
		catch (Exception e) {
				System.out.println("Error: Cannot open serial port.");
		}
	}

	public void close() throws Exception {
		outputStream_.close();
		inputStream_.close();
		serialPort_.close();
	}
	
	public boolean initialize() {
		return true;
	}
	
	public String readAll() throws IOException {
		byte[] recv_buffer = new byte[512];
		inputStream_.read(recv_buffer);
		String s = new String(recv_buffer);
		return s;
	}
	
	public void reset() {
		
	}
	
	public void setMotorRPS(float right_motor_rps, float left_motor_rps) {
		String message = "2,2,0," + left_motor_rps + "," + right_motor_rps + "\n";
		sendMessage(message);
	}
	
	public double[] getMotorRPS() {
		String reply = sendMessageWaitForReply("0,58,0\n");
		return parseReply(reply);
	}
	
	public double[] getIREncodedValues() {
		double IR012[];
		double IR345[];
		
		String reply = sendMessageWaitForReply("0,53,0\n");
		IR012 = parseReply(reply);
		if(IR012 == null)	return null;
		
		reply = waitForReply();
		IR345 = parseReply(reply);
		if(IR345 == null)	return null;
		
		double IRValues[] = 	{IR012[0], IR012[1], IR012[2], 
								 IR345[0], IR345[1], IR345[2]};
		return IRValues;
	}
	
	public double[] parseReply(String reply) {	// '2, 11, 4, 3.14, 6.28\n'double
		if ((reply != null) && (reply.length() > 0)) {
			String content = reply.substring(0, reply.length()-1); // remove \n
			content = content.trim();
			String[] tokens = content.split(",");
			int sizeMsg = 0;
			
			try {
				sizeMsg = Integer.parseInt(tokens[0].trim());
			} catch (NumberFormatException e) {
				System.err.println("Error: unable to decode data.");
				return null;
			}
			if(sizeMsg > 0) {
				double[] parsed_reply = new double[sizeMsg];
				
				for(int i = 0 ; i < sizeMsg ; i++) {
					try {
						parsed_reply[i] = Double.parseDouble(tokens[i+3].trim());
					} catch (NumberFormatException e) {
						System.err.println("Error: unable to decode data.");
						parsed_reply[i] = Double.NaN;
					}
				}
			
				return parsed_reply;
			}
		}
		return null;
	}
	
	public void sendMessage(String message) {			
		byte[] send_buffer = message.getBytes();
		
		try {
			this.outputStream_.write(send_buffer);
		}
		catch(IOException e) {
			System.err.println("Error: Failed to send packet.");
		}
	}
	
	public String sendMessageWaitForReply(String message) {
		sendMessage(message);
		
		return waitForReply();
	}
	
	public String waitForReply() {
		String reply = null;
		
		try
		{
			reply = inReader_.readLine();
		}
		catch ( IOException e ) {
			System.err.println("Error: Failed to read packet.");
		}
		
		return reply;
	}
}
