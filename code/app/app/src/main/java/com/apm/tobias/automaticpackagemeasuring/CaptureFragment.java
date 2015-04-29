package com.apm.tobias.automaticpackagemeasuring;

import android.app.Activity;
import android.app.Fragment;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CompoundButton;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

public class CaptureFragment extends Fragment implements Camera.PictureCallback, Camera.PreviewCallback, Handler.Callback {

    private static final String TAG = "CaptureFragment";
    private static final int MSG_CREATE_CAMERA = 0x01;
    private static final int MSG_DESTROY_CAMERA = 0x02;

    private final PackageMeasurer mPackageMeasurer = new PackageMeasurer();
    private final Handler mHandler = new Handler();
    private final HandlerThread mCameraThread = new HandlerThread("CameraHandler");
    private Handler mCameraHandler;
    private Camera mCamera;
    private SurfaceView mSurfaceView;
    private Camera.Size mPreviewSize;
    private SurfaceHolder mSurfaceViewHolder;
    private View mView;

    private OverlayView mOverlayView;
    private TextView mFPSView;
    private TextView mProcessingTimeView;
    private long mLastFrame = 0;
    private ToggleButton mTorchButton;
    private int mRotation;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mCameraThread.start();
        mCameraHandler = new Handler(mCameraThread.getLooper(), this);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        mView = inflater.inflate(R.layout.fragment_capture, container, false);

        if (mView == null) {
            return null;
        }

        mSurfaceView = (SurfaceView) mView.findViewById(R.id.preview);
        if (mSurfaceView != null) {
            SurfaceHolder surfaceViewHolder = mSurfaceView.getHolder();
            surfaceViewHolder.addCallback(new SurfaceHolder.Callback() {
                @Override
                public void surfaceCreated(SurfaceHolder holder) {
                    mSurfaceViewHolder = holder;
                    if (isResumed()) {
                        mCameraHandler.sendEmptyMessage(MSG_CREATE_CAMERA);
                    }
                }

                @Override
                public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

                }

                @Override
                public void surfaceDestroyed(SurfaceHolder holder) {
                    mSurfaceViewHolder = null;
                }
            });
        }

        mOverlayView = (OverlayView) mView.findViewById(R.id.reference_object_overlay);
        mFPSView = (TextView) mView.findViewById(R.id.fps);
        mProcessingTimeView = (TextView) mView.findViewById(R.id.processing_Time);

        mTorchButton = (ToggleButton) mView.findViewById(R.id.action_torch);
        mTorchButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(final CompoundButton buttonView, final boolean isChecked) {
                setTorchEnabled(isChecked);
            }
        });
        mTorchButton.setChecked(false);

        return mView;
    }

    @Override
    public void onResume() {
        super.onResume();
        if (mSurfaceViewHolder != null) {
            mCameraHandler.sendEmptyMessage(MSG_CREATE_CAMERA);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        mCameraHandler.sendEmptyMessage(MSG_DESTROY_CAMERA);
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        mSurfaceViewHolder = null;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mCameraThread.quit();
    }

    private void cameraCreated(Camera camera) {
        mCamera = camera;
        setupCamera();

        Log.d(TAG, "Camera created");
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
        Log.d(TAG, "CAMERA SIZE= " + size.width + ", " + size.height);
        Log.d(TAG, "VIEW SIZE= " + mView.getWidth() + ", " + mView.getHeight());
        mHandler.post(new Runnable() {
            @Override
            public void run() {
                ViewGroup.LayoutParams lp = mSurfaceView.getLayoutParams();
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
        Camera.CameraInfo info = new Camera.CameraInfo();
        mCamera.getCameraInfo(0, info);

        mOverlayView.setRotation(info.orientation);
        mRotation = info.orientation;
        mOverlayView.setPreviewSize(size.height, size.width);
        mCamera.setDisplayOrientation(info.orientation);


        mCamera.setParameters(params);
    }

    @Override
    public void onPreviewFrame(final byte[] data, Camera camera) {
        YuvImage image = new YuvImage(data, mCamera.getParameters().getPreviewFormat(), mPreviewSize.width, mPreviewSize.height, null);
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        image.compressToJpeg(new Rect(0, 0, mPreviewSize.width, mPreviewSize.height), 90, byteArrayOutputStream);
        byte[] bytes = byteArrayOutputStream.toByteArray();
        final Bitmap bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.length);

        long start = System.currentTimeMillis();

        mPackageMeasurer.analyzeVideoFrame(data, mPreviewSize.width, mPreviewSize.height, mRotation);
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

                mOverlayView.setReferenceObject(mPackageMeasurer.getReferenceObject());
                mOverlayView.setPackage(mPackageMeasurer.getPackage());
                mOverlayView.setDimensions(mPackageMeasurer.getDimensions());
                mOverlayView.setMeasuredEdges(mPackageMeasurer.getMeasuredEdges());
                mOverlayView.invalidate();
            }
        });

        if (mCamera != null) {
            mCamera.addCallbackBuffer(data);
        }
    }

    @Override
    public void onPictureTaken(byte[] data, Camera camera) {
    }

    private void setTorchEnabled(final boolean isChecked) {
        mCameraHandler.post(new Runnable() {
            @Override
            public void run() {
                if (mCamera == null) {
                    return;
                }
                Camera.Parameters params = mCamera.getParameters();
                if (isChecked) {
                    params.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
                } else {
                    params.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
                }
                mCamera.setParameters(params);
            }
        });
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

    @Override
    public boolean handleMessage(Message msg) {
        switch (msg.what) {
            case MSG_CREATE_CAMERA:
                createCamera();
                return true;

            case MSG_DESTROY_CAMERA:
                destroyCamera();
                return true;

            default:
                return false;
        }
    }

    private void createCamera() {
        if (mCamera != null) {
            return;
        }
        try {
            mCamera = Camera.open();
            cameraCreated(mCamera);

        } catch (Exception e) {
            e.printStackTrace();
            cameraFailedToOpen();
        }
    }

    private void destroyCamera() {
        if (mCamera != null) {
            mCamera.stopPreview();
            mCamera.release();
            mCamera = null;
        }
    }
}