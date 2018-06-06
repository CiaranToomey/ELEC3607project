
// A Java program for a Client
import java.net.*;
import java.io.*;

public class Client
{
    // initialize socket and input output streams
    private Socket socket            = null;
    private DataInputStream  input   = null;
    private OutputStream out     = null;

    // constructor to put ip address and port
    public Client(String address, int port)
    {

        // establish a connection
        try
        {
            socket = new Socket(address, port);
            System.out.println("Connected");

            // takes input from terminal
            input  = new DataInputStream(System.in);

            // sends output to the socket
            out    = new DataOutputStream(socket.getOutputStream());
        }
        catch(UnknownHostException u)
        {
            System.out.println(u);
        }
        catch(IOException i)
        {
            System.out.println(i);
        }

//         File file = new File("file.txt");
//         BufferedReader reader = null;

//         try {
//             reader = new BufferedReader(new FileReader(file));
//             String text = null;

//             while ((text = reader.readLine()) != null) {
//                 // System.out.println(text);
//                 try {
//                 byte[] b = text.getBytes();
//                 out.write(b);
//                 } catch(IOException i) {
//                 System.out.println(i);
//                 }
//             }
//         } catch (FileNotFoundException e) {
//             e.printStackTrace();
//         } catch (IOException e) {
//             e.printStackTrace();
//         } finally {
//             try {
//                 if (reader != null) {
//                     reader.close();
//                 }
//             } catch (IOException e) {
//             }
//         }


        // string to read message from input
        String line = "";

        // keep reading until "Over" is input
        while (!line.equals("Over"))
        {
            try
            {
                // interface
                // 'm' for transfer of map data
                // 's' for speed
                // 


                line = input.readLine();
                byte[] b = line.getBytes();
                out.write(b);
            }
            catch(IOException i)
            {
                System.out.println(i);
            }
        }

        // close the connection
        try
        {
            input.close();
            out.close();
            socket.close();
        }
        catch(IOException i)
        {
            System.out.println(i);
        }
    }

    public static void main(String args[])
    {
        Client client = new Client("172.20.10.3", 5000);
    }
}
