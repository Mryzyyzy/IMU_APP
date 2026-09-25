package com.example.imu_app.navigation;

import com.example.imu_app.model.ImuSample;
import com.example.imu_app.model.TrackPoint;

import java.util.Collections;
import java.util.List;

public class ImuNavigationProcessor {
    public static final String V1_MATLAB_PORT = "v1_matlab_port";
    public static final String V2_PDR_TURN_SNAP = "v2_pdr_turn_snap";

    private String algorithmVersion = V1_MATLAB_PORT;
    private double referenceLatitude = 30.659462;
    private double referenceLongitude = 104.065735;
    private double referenceAltitude = 482.0;

    private final V1MatlabPortAlgorithm v1 = new V1MatlabPortAlgorithm();
    private final V2PdrTurnSnapAlgorithm v2 = new V2PdrTurnSnapAlgorithm();

    public void reset(double referenceLatitude, double referenceLongitude, double referenceAltitude, String algorithmVersion) {
        this.referenceLatitude = referenceLatitude;
        this.referenceLongitude = referenceLongitude;
        this.referenceAltitude = referenceAltitude;
        this.algorithmVersion = normalizeAlgorithmVersion(algorithmVersion);
        v1.reset(referenceLatitude, referenceLongitude, referenceAltitude);
        v2.reset(referenceLatitude, referenceLongitude, referenceAltitude);
    }

    public TrackPoint process(ImuSample sample) {
        if (V2_PDR_TURN_SNAP.equals(algorithmVersion)) {
            return v2.process(sample);
        }
        return v1.process(sample);
    }

    public List<TrackPoint> processBatch(List<ImuSample> samples) {
        if (samples.isEmpty()) {
            return Collections.emptyList();
        }
        reset(referenceLatitude, referenceLongitude, referenceAltitude, algorithmVersion);
        if (V2_PDR_TURN_SNAP.equals(algorithmVersion)) {
            return v2.processBatch(samples);
        }
        return v1.processBatch(samples);
    }

    public static String normalizeAlgorithmVersion(String value) {
        if (V2_PDR_TURN_SNAP.equals(value)) {
            return V2_PDR_TURN_SNAP;
        }
        return V1_MATLAB_PORT;
    }
}
