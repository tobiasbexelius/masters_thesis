package com.apm.tobias.automaticpackagemeasuring;

import android.hardware.Camera;
import android.util.Log;

import java.util.List;

public class CameraUtils {
    private static final String TAG = "CameraUtils";

    public static void selectOptimalPictureSize(Camera.Parameters params) {
        if (params == null) {
            return;
        }

        Camera.Size best = null;
        List<Camera.Size> supportedPictureSizes = params.getSupportedPictureSizes();
        if (supportedPictureSizes == null) {
            return;
        }

        for (Camera.Size size : supportedPictureSizes) {
            if (isSizeBetter(size, best)) {
                best = size;
            }
        }

        if (best != null) {
            Log.d(TAG, String.format("Setting picture size to %dx%d", best.width, best.height));
            params.setPictureSize(best.width, best.height);
        }
    }

    public static Camera.Size getOptimalPreviewSize(Camera.Parameters params, int w, int h) {
        List<Camera.Size> sizes = params.getSupportedPreviewSizes();
        if (sizes == null) {
            return null;
        }

        final double ASPECT_TOLERANCE = 0.2;
        double targetRatio = (double) w / h;
        Camera.Size optimalSize = null;
        for (Camera.Size size : sizes) {
            if(size.height == 720 && size.width==1280)
                return size;

            double ratio = (double) size.width / size.height;
            if (Math.abs(ratio - targetRatio) > ASPECT_TOLERANCE) {
                continue;
            }
            if (isSizeBetter(size, optimalSize)) {
                optimalSize = size;
            }
        }
        // Cannot find the one match the aspect ratio, ignore the requirement

        if (optimalSize == null) {
            for (Camera.Size size : sizes) {
                if (isSizeBetter(size, optimalSize)) {
                    optimalSize = size;
                }
            }
        }

        return optimalSize;
    }

    private static boolean isSizeBetter(Camera.Size contender, Camera.Size best) {
        return best == null || contender.height * contender.width > best.height * best.width;
    }

    public static void setHighestFPSRange(Camera.Parameters params) {
        if (params == null) {
            return;
        }

        List<int[]> ranges = params.getSupportedPreviewFpsRange();
        if (ranges == null) {
            return;
        }

        int bestRange = Integer.MAX_VALUE;
        int bestMax = 0;
        for (int[] range : ranges) {
            int r = range[1] - range[0];
            if (range[1] > bestMax || range[1] == bestMax && r > bestRange) {
                bestMax = range[1];
                bestRange = r;
            }
        }

        Log.d(TAG, String.format("Setting FPS range to %d-%d", bestMax - bestRange, bestMax));
        params.setPreviewFpsRange(bestMax - bestRange, bestMax);
    }

    public static void selectFocusMode(Camera.Parameters params, String requestedMode) {
        List<String> supportedFocusModes = params.getSupportedFocusModes();
        if (supportedFocusModes == null) {
            return;
        }

        for (String mode : supportedFocusModes) {
            if (mode.equals(requestedMode)) {
                params.setFocusMode(mode);
                return;
            }
        }
    }
}
