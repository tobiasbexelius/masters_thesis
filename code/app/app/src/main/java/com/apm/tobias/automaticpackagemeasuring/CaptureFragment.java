package com.apm.tobias.automaticpackagemeasuring;

import android.app.Activity;
import android.app.Fragment;
import android.graphics.Rect;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CompoundButton;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.apm.tobias.automaticpackagemeasuring.core.Edge;
import com.apm.tobias.automaticpackagemeasuring.core.PackageMeasurer;
import com.apm.tobias.automaticpackagemeasuring.core.ReferenceObject;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class CaptureFragment extends Fragment implements Camera.PictureCallback, Camera.PreviewCallback {
    private static final String TAG = "CaptureFragment";

    private final PackageMeasurer mPackageMeasurer = new PackageMeasurer();
    private final Handler mHandler = new Handler();
    private CameraThread mCameraThread;
    private Camera mCamera;
    private SurfaceView mSurfaceView;
    private Camera.Size mPreviewSize;
    private SurfaceHolder mSurfaceViewHolder;
    private View mView;

    private OverlayView mReferenceObjectOverlay;
    private TextView mFPSView;
    private TextView mProcessingTimeView;
    private long mLastFrame = 0;
    private ToggleButton mTorchButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        mView = inflater.inflate(R.layout.fragment_capture, container, false);

        if (mView == null) {
            return null;
        }

        mSurfaceView = (SurfaceView) mView.findViewById(R.id.preview);
        if (mSurfaceView != null) {
            mSurfaceViewHolder = mSurfaceView.getHolder();
        }

        mReferenceObjectOverlay = (OverlayView) mView.findViewById(R.id.reference_object_overlay);
        mFPSView = (TextView) mView.findViewById(R.id.fps);
        mProcessingTimeView = (TextView) mView.findViewById(R.id.processing_Time);

        mTorchButton = (ToggleButton) mView.findViewById(R.id.action_torch);
        mTorchButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(final CompoundButton buttonView, final boolean isChecked) {
                toggleTorch(isChecked);
            }
        });
        mTorchButton.setChecked(false);

        return mView;
    }

    @Override
    public void onResume() {
        super.onResume();
        mCameraThread = new CameraThread();
        mCameraThread.start();
    }

    @Override
    public void onPause() {
        super.onPause();

        if (mCameraThread == null) {
            return;
        }

        mCameraThread.shutdown();
        mCameraThread = null;
    }

    private void cameraCreated(Camera camera) {
        mCamera = camera;
        setupCamera();

        try {
            byte[] previewBuffer = new byte[(int) (mPreviewSize.width * mPreviewSize.height * 1.5)];
            mCamera.setPreviewDisplay(mSurfaceViewHolder);
            mCamera.addCallbackBuffer(previewBuffer);
            mCamera.setPreviewCallbackWithBuffer(this);
            mCamera.startPreview();
        } catch (IOException e) {
            Log.e(TAG, "Failed to start the preview", e);
        }
    }

    private void setupCamera() {
        Camera.Parameters params = mCamera.getParameters();
        // We invert the width and height since we rotate the preview
        final Camera.Size size = mPreviewSize = CameraUtils.getOptimalPreviewSize(params, mView.getHeight(), mView.getWidth());

        mHandler.post(new Runnable() {
            @Override
            public void run() {
                RelativeLayout.LayoutParams lp = (RelativeLayout.LayoutParams) mSurfaceView.getLayoutParams();
                if (lp != null) {
                    lp.width = mView.getWidth();
                    lp.height = (int) (lp.width * (float) size.width / size.height);

                    if (lp.height > mView.getHeight()) {
                        lp.height = mView.getHeight();
                        lp.width = (int) (lp.height * (float) size.height / size.width);
                    }

                    mSurfaceView.setLayoutParams(lp);
                    Log.d(TAG, String.format("Setting surface size to %dx%d", lp.width, lp.height));
                }
            }
        });

        Log.d(TAG, String.format("Setting preview size to %dx%d", size.width, size.height));
        params.setRotation(90);
        params.setPreviewSize(size.width, size.height);
        params.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
        CameraUtils.setHighestFPSRange(params);
        CameraUtils.selectOptimalPictureSize(params);

        CameraUtils.selectFocusMode(params, Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);

        mCamera.setDisplayOrientation(90);
        mCamera.setParameters(params);
    }

    @Override
    public void onPreviewFrame(final byte[] data, Camera camera) {
        long start = System.currentTimeMillis();
        mPackageMeasurer.analyzeVideoFrame(data, mPreviewSize.width, mPreviewSize.height);
        long now = System.currentTimeMillis();
        final long processingTime = now - start;

        final long sinceLastFrame;
        if (mLastFrame != 0) {
            sinceLastFrame = now - mLastFrame;
        } else {
            sinceLastFrame = 1000;
        }
        mLastFrame = now;

        mHandler.post(new Runnable() {
            @Override
            public void run() {
                if (!isResumed()) {
                    return;
                }

                mFPSView.setText(getString(R.string.fps, 1000.0f / sinceLastFrame));
                mProcessingTimeView.setText(getString(R.string.processing_time, processingTime));

                drawReferenceObject();
            }
        });

        if (mCamera != null) {
            mCamera.addCallbackBuffer(data);
        }
    }

    private void drawReferenceObject() {
        ReferenceObject referenceObject = mPackageMeasurer.getReferenceObject();
        List<Edge> edges = referenceObject.getEdges();

        if(edges.isEmpty())
            mReferenceObjectOverlay.clear();
        else
            mReferenceObjectOverlay.updateEdges(edges);
        
        mReferenceObjectOverlay.invalidate();
    }

    @Override
    public void onPictureTaken(byte[] data, Camera camera) {
    }

    private void toggleTorch(final boolean isChecked) {
        if (mCamera != null) {
            Camera.Parameters params = mCamera.getParameters();
            if (isChecked) {
                params.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
            } else {
                params.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
            }
            mCamera.setParameters(params);
        }
    }

    private void cameraFailedToOpen() {
        mHandler.post(new Runnable() {
            @Override
            public void run() {
                Activity activity = getActivity();
                if (activity != null) {
                    Toast.makeText(getActivity(), R.string.camera_failed_to_open, Toast.LENGTH_LONG).show();
                    getActivity().finish();
                }
            }
        });
    }

    private class CameraThread extends Thread {
        public Handler mHandler;
        private boolean mShutdown = false;
        private Camera mCamera;

        @Override
        public void run() {
            Looper.prepare();
            if (mShutdown) {
                return;
            }
            synchronized (this) {
                mHandler = new Handler();
            }
            createCamera();
            Looper.loop();
        }

        public void shutdown() {
            mShutdown = true;
            synchronized (this) {
                if (mHandler != null) {
                    mHandler.post(new Runnable() {
                        @Override
                        public void run() {
                            if (mCamera != null) {
                                mCamera.stopPreview();
                                mCamera.release();
                                mCamera = null;
                                Looper.myLooper().quitSafely();
                            }
                        }
                    });
                }
            }
        }

        private void createCamera() {
            try {
                mCamera = Camera.open();
                cameraCreated(mCamera);

            } catch (Exception e) {
                e.printStackTrace();
                cameraFailedToOpen();
            }
        }
    }

}