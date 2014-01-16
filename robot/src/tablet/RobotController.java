package tablet;

import comm.*;
import devices.sensors.Gyroscope;
import jssc.SerialPort;
import jssc.SerialPortException;

public class RobotController {
    
    public static void main(String[] args) throws SerialPortException {
        
        MapleComm comm = new MapleComm(MapleIO.SerialPortType.LINUX);
        comm.registerDevice(new Gyroscope(11, 10));
        comm.initialize();
        comm.transmit();
        comm.updateSensorData();
//        SerialPort serialPort;
//        try {
//            serialPort = new SerialPort("/dev/tty.usbmodemfa131");
//            serialPort.openPort();
//            serialPort.setParams(115200, 8, 1, 0);
//            
//            byte[] bytes;
//            
//            byte[] packet = buildPacket((byte) 'I', new byte[] {(byte) 'G'}, (byte) 0xff);
//            serialPort.writeBytes(packet);
//            while(true) {
//                bytes = serialPort.readBytes();
//                if(bytes != null) {
//                    for(byte elt : bytes) {
//                        System.out.print(elt + " ");
//                    }
//                    System.out.print('\n');
//                }
//            }
//        }
//        catch (SerialPortException ex){
//            System.out.println(ex);
//        }
    }
    
//    private static byte[] buildPacket(byte first, byte[] message, byte last) {
//        int len = message.length;
//        byte[] packet = new byte[len + 2];
//        packet[0] = first;
//        System.arraycopy(message, 0, packet, 1, len);
//        packet[len + 1] = last;
//        return packet;
//    }

}
