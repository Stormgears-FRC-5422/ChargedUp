package frc.robot.subsystems.stormnet;

import frc.utils.configfile.StormProp;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.util.Map;

// This same pattern could someday be used to implement an I2C listener or something else entirely
// that's why I didn't subclass EthernetVoice. It is really solving a slightly different problem.
public class UDPListener extends StormNetVoice implements Runnable{
    private Map<String, Integer> m_commandMap;
    private byte[] m_receiveBuffer = new byte[0];
    private boolean m_stopNow = false;
    private final Object m_lock = 0;

    public UDPListener() {
    }

    @Override
    public String getDeviceString() {
        return "UDP listener";
    }

    public void setCommandLocations(Map<String, Integer> commandMap, int size) {
        m_commandMap = commandMap;
        synchronized (m_lock) {
            m_receiveBuffer = new byte[size];
        }
    }

    @Override
    protected boolean transaction_internal(byte[] dataToSend, int sendSize, byte[] dataReceived, int receiveSize) {
        // This listener is just managing bytes that are coming in on the wire.
        // The downstream side already knows what it is sending, and this side must
        // agree. I can think of more complex patterns where this is negotiated at startup
        // but not right now.
        //
        // All commands start with a unique character, so map that character to its position
        // in the receiveBuffer. So <"P", 0>;<"F", 1> would make sense since PING returns 1 bytes
        //
        // No bytes are actually sent, but the command name is trapped to find the offset
        // from which to pull the relevant bytes

//        System.out.println(new String(new byte[]{dataToSend[0]}));// wow really?
//        System.out.println(m_commandMap.toString());
        int offset = m_commandMap.get( new String(new byte[]{dataToSend[0]}) );  //todo error handling
        synchronized (m_lock) {
            System.arraycopy(m_receiveBuffer, offset, dataReceived,0, receiveSize);
        }
//        System.out.println(Arrays.toString(dataReceived));
        return StormNetSensor.STORMNET_SUCCESS;
    }

    // This only happens once, so we know a lot about its synchrony with other
    // parts of the class
    public void run() {
        DatagramSocket socket;
        m_stopNow = false;

        System.out.println("Starting listener thread...");
        byte[] localBuffer = new byte[m_receiveBuffer.length];
        int udpListenerPort = StormProp.getInt("udpListenerPort", -1);

        try {
            // Listening only - lets use a different port
//            socket = new DatagramSocket(StormProp.getInt("udpListenerPort",-1));
            socket = new DatagramSocket(null); // Create an unbound socket
            socket.setReuseAddress(true); // Enable address reuse
            socket.bind(new InetSocketAddress(udpListenerPort)); // Bind the socket to the specified port

            DatagramPacket packet = new DatagramPacket(localBuffer, localBuffer.length);
            while (!m_stopNow) {
                socket.receive(packet);
                //for debugging
                //System.out.print("local buffer: ");
                //System.out.println(Arrays.toString(localBuffer));
                synchronized (m_lock) {
                    System.arraycopy(packet.getData(), 0, m_receiveBuffer, 0, packet.getLength());
                }
            }
            socket.close(); // so we don't run into ourselves later
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void stop() {
        m_stopNow = true;
    }
}

