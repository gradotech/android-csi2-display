package org.freedesktop.gstreamer.tutorials.tutorial_3

import android.annotation.SuppressLint
import android.app.Activity
import android.graphics.SurfaceTexture
import android.os.Bundle
import android.view.MotionEvent
import android.view.Surface
import android.view.TextureView
import android.view.View
import android.widget.Toast
import com.andyha.gstream.tutorial.shared.GStreamerTextureView
import org.freedesktop.gstreamer.GStreamer.init

class Tutorial3 : Activity(), TextureView.SurfaceTextureListener {
    private external fun nativeInit()
    private external fun nativeFinalize()
    private external fun nativePlay()
    private external fun nativePause()
    private external fun nativeSurfaceInit(surface: Any)
    private external fun nativeSurfaceFinalize()
    private external fun nativeSetPin(value: Boolean)
    private external fun nativeSetupUART(value: Boolean): Boolean
    private external fun nativeWriteCoordinates(x: Int, y: Int, slot: Int, press: Int): Boolean

    private val nativeCustomData: Long = 0
    private var isOnFocus: Boolean = true;

    private lateinit var gstreamerTextureView: GStreamerTextureView

    companion object {
        @JvmStatic
        private external fun nativeClassInit(): Boolean

        init {
            System.loadLibrary("gstreamer_android")
            System.loadLibrary("tutorial-3")
            nativeClassInit()
        }

        private const val targetWidth = 1920f
        const val targetHeight = 1200f
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        try {
            init(this)
        } catch (e: Exception) {
            Toast.makeText(this, e.message, Toast.LENGTH_LONG).show()
            finish()
            return
        }

        window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_FULLSCREEN
                        or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY)

        actionBar?.hide()
        setContentView(R.layout.main)

        gstreamerTextureView = findViewById(R.id.gstTextureView)
        gstreamerTextureView.surfaceTextureListener = this

        nativeInit()
        nativeSetupUART(true)

        gstreamerTextureView.setOnTouchListener { _, event ->
            val viewWidth = gstreamerTextureView.width.toFloat()
            val viewHeight = gstreamerTextureView.height.toFloat()

            val pointerCount = event.pointerCount

            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN, MotionEvent.ACTION_POINTER_DOWN -> {
                    for (i in 0 until pointerCount) {
                        val pointerId = event.getPointerId(i)
                        val x = event.getX(i)
                        val y = event.getY(i)

                        val mappedX = (x / viewWidth) * Companion.targetWidth
                        val mappedY = (y / viewHeight) * Companion.targetHeight

                        nativeWriteCoordinates(mappedX.toInt(), mappedY.toInt(), pointerId, 1)
                    }

                    if (isOnFocus) {
                        nativeSetPin(true)
                    }
                    true
                }

                MotionEvent.ACTION_MOVE -> {
                    for (i in 0 until pointerCount) {
                        val pointerId = event.getPointerId(i)
                        val x = event.getX(i)
                        val y = event.getY(i)

                        val mappedX = (x / viewWidth) * Companion.targetWidth
                        val mappedY = (y / viewHeight) * Companion.targetHeight

                        if (isOnFocus) {
                            nativeWriteCoordinates(mappedX.toInt(), mappedY.toInt(), pointerId, 1)
                        }
                    }
                    true
                }

                MotionEvent.ACTION_UP, MotionEvent.ACTION_POINTER_UP, MotionEvent.ACTION_CANCEL -> {
                    for (i in 0 until pointerCount) {
                        val pointerId = event.getPointerId(i)

                        nativeWriteCoordinates(0, 0, pointerId, 0)
                    }

                    if (isOnFocus) {
                        nativeSetPin(false)
                    }
                    true
                }

                else -> false
            }
        }
    }

    override fun onDestroy() {
        nativeSetupUART(false)
        nativeFinalize()
        super.onDestroy()
    }

    override fun onResume() {
        super.onResume()
        isOnFocus = true;
    }

    override fun onPause() {
        super.onPause()
        isOnFocus = false;
    }

    private fun onGStreamerInitialized() {
        nativePlay()
    }

    override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
        nativeSurfaceInit(Surface(surface))
    }

    override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
        nativeSurfaceFinalize()
        return true
    }

    override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int)
    { }

    override fun onSurfaceTextureUpdated(surface: SurfaceTexture)
    { }
}
