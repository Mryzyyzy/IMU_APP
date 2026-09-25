package com.example.imu_app.communication;

import java.io.IOException;
import java.io.InputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.List;

public class TcpClient {
    public interface Listener {
        void onConnected();

        void onLine(String line);

        void onDisconnected(String reason);

        void onError(String message);
    }

    private final Listener listener;
    private final LineFrameBuffer frameBuffer = new LineFrameBuffer();
    private volatile boolean running = false;
    private Socket socket;
    private Thread worker;

    public TcpClient(Listener listener) {
        this.listener = listener;
    }

    public synchronized void connect(String host, int port) {
        disconnect();
        running = true;
        frameBuffer.clear();
        worker = new Thread(() -> runClient(host, port), "imu-tcp-client");
        worker.start();
    }

    public synchronized void disconnect() {
        running = false;
        if (socket != null) {
            try {
                socket.close();
            } catch (IOException ignored) {
                // Closing a socket can race with the read loop; the loop reports final state.
            }
            socket = null;
        }
    }

    public boolean isRunning() {
        return running;
    }

    private void runClient(String host, int port) {
        try (Socket newSocket = new Socket()) {
            socket = newSocket;
            newSocket.connect(new InetSocketAddress(host, port), 5000);
            newSocket.setSoTimeout(1000);
            listener.onConnected();

            InputStream input = newSocket.getInputStream();
            byte[] chunk = new byte[2048];
            while (running && !newSocket.isClosed()) {
                try {
                    int read = input.read(chunk);
                    if (read < 0) {
                        break;
                    }
                    List<String> frames = frameBuffer.append(chunk, read);
                    for (String frame : frames) {
                        listener.onLine(frame);
                    }
                } catch (SocketTimeoutException ignored) {
                    // Keep the read loop responsive to disconnect().
                }
            }
            if (running) {
                listener.onDisconnected("连接已断开");
            }
        } catch (IOException error) {
            if (running) {
                listener.onError(error.getMessage() == null ? "TCP 连接失败" : error.getMessage());
            }
        } finally {
            running = false;
            socket = null;
            frameBuffer.clear();
        }
    }
}
