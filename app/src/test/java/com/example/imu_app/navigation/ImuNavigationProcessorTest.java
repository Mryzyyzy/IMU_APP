package com.example.imu_app.navigation;

import com.example.imu_app.coordinate.CoordinateConverter;
import com.example.imu_app.model.ImuSample;
import com.example.imu_app.model.TrackPoint;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class ImuNavigationProcessorTest {
    @Test
    public void stationarySamplesAlignAndRemainNearOrigin() {
        ImuNavigationProcessor processor = new ImuNavigationProcessor();
        processor.reset(30.659462, 104.065735, 482.0, ImuNavigationProcessor.V1_MATLAB_PORT);

        TrackPoint point = null;
        for (int i = 0; i < 360; i++) {
            TrackPoint next = processor.process(new ImuSample(
                    1_000L + i * 10L,
                    0.0,
                    0.0,
                    9.80665,
                    0.0,
                    0.0,
                    0.0,
                    null,
                    null,
                    null,
                    null,
                    i
            ));
            if (i < 299) {
                assertNull(next);
            }
            if (next != null) {
                point = next;
            }
        }

        assertNotNull(point);
        assertTrue(Math.abs(point.east) < 0.05);
        assertTrue(Math.abs(point.north) < 0.05);
        assertTrue(point.speed < 0.05);
        assertTrue(point.quality >= 0.8);
    }

    @Test
    public void wgs84EnuRoundTripStaysStableForLocalOffsets() {
        double refLat = 30.659462;
        double refLon = 104.065735;
        double refAlt = 482.0;
        double[] wgs84 = CoordinateConverter.enuToWgs84(12.5, -8.25, 1.75, refLat, refLon, refAlt);
        double[] enu = CoordinateConverter.wgs84ToEnu(wgs84[0], wgs84[1], wgs84[2], refLat, refLon, refAlt);

        assertEquals(12.5, enu[0], 1e-6);
        assertEquals(-8.25, enu[1], 1e-6);
        assertEquals(1.75, enu[2], 1e-9);
    }
}
