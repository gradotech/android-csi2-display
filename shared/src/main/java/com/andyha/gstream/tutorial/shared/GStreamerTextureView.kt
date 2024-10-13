package com.andyha.gstream.tutorial.shared

import android.content.Context
import android.graphics.SurfaceTexture
import android.util.AttributeSet
import android.view.TextureView

class GStreamerTextureView : TextureView, TextureView.SurfaceTextureListener {

    var mediaWidth = 320
    var mediaHeight = 240
    private var surfaceAvailable = false
    private var surface: SurfaceTexture? = null

    constructor(context: Context?, attrs: AttributeSet?, defStyle: Int) : super(context!!, attrs, defStyle)
    constructor(context: Context?, attrs: AttributeSet?) : super(context!!, attrs)
    constructor(context: Context?) : super(context!!)

    init {
        surfaceTextureListener = this
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        var width = 0
        var height = 0
        val wmode = MeasureSpec.getMode(widthMeasureSpec)
        val hmode = MeasureSpec.getMode(heightMeasureSpec)
        val wsize = MeasureSpec.getSize(widthMeasureSpec)
        val hsize = MeasureSpec.getSize(heightMeasureSpec)

        when (wmode) {
            MeasureSpec.AT_MOST -> {
                width = if (hmode == MeasureSpec.EXACTLY) {
                    (hsize * mediaWidth / mediaHeight).coerceAtMost(wsize)
                } else {
                    wsize
                }
            }
            MeasureSpec.EXACTLY -> width = wsize
            MeasureSpec.UNSPECIFIED -> width = mediaWidth
        }
        when (hmode) {
            MeasureSpec.AT_MOST -> {
                height = if (wmode == MeasureSpec.EXACTLY) {
                    (wsize * mediaHeight / mediaWidth).coerceAtMost(hsize)
                } else {
                    hsize
                }
            }
            MeasureSpec.EXACTLY -> height = hsize
            MeasureSpec.UNSPECIFIED -> height = mediaHeight
        }

        if (hmode == MeasureSpec.AT_MOST && wmode == MeasureSpec.AT_MOST) {
            val correctHeight = width * mediaHeight / mediaWidth
            val correctWidth = height * mediaWidth / mediaHeight
            if (correctHeight < height) height = correctHeight else width = correctWidth
        }

        width = suggestedMinimumWidth.coerceAtLeast(width)
        height = suggestedMinimumHeight.coerceAtLeast(height)
        setMeasuredDimension(width, height)
    }

    override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
        this.surface = surface
        surfaceAvailable = true
        // Notify the main activity or GStreamer pipeline that the surface is available
    }

    override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) {
        // Handle surface size changes if necessary
    }

    override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
        surfaceAvailable = false
        this.surface = null
        // Notify the main activity or GStreamer pipeline that the surface is destroyed
        return true
    }

    override fun onSurfaceTextureUpdated(surface: SurfaceTexture) {
        // Handle surface texture updates if necessary
    }

    fun getSurface(): SurfaceTexture? {
        return if (surfaceAvailable) surface else null
    }
}
