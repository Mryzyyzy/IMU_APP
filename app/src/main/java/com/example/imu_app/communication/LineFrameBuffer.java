package com.example.imu_app.communication;

import java.io.ByteArrayOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

public class LineFrameBuffer {
    private final ByteArrayOutputStream buffer = new ByteArrayOutputStream();

    public synchronized List<String> append(byte[] data, int length) {
        List<String> frames = new ArrayList<>();
        for (int i = 0; i < length; i++) {
            byte value = data[i];
            if (value == '\n') {
                addFrame(frames);
            } else {
                buffer.write(value);
            }
        }
        return frames;
    }

    public synchronized void clear() {
        buffer.reset();
    }

    private void addFrame(List<String> frames) {
        String frame = new String(buffer.toByteArray(), StandardCharsets.US_ASCII).trim();
        buffer.reset();
        if (!frame.isEmpty()) {
            frames.add(frame);
        }
    }
}
