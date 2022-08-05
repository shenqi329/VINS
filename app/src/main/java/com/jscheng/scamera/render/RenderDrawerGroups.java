package com.jscheng.scamera.render;

import android.content.Context;
import android.opengl.EGLContext;
import android.opengl.GLES30;
import android.util.Log;

import com.example.vins.MagicPenJNI;
import com.example.vins.obj.Ball;
import com.jscheng.scamera.util.GlesUtil;

import static com.jscheng.scamera.util.LogUtil.TAG;

import java.nio.IntBuffer;

/**
 * Created By Chengjunsen on 2018/8/31
 * 统一管理所有的RenderDrawer 和 FBO
 */
public class RenderDrawerGroups {
    private int mInputTexture;
    private int mFrameBuffer;
    private int mDepthStencilRender;
    private OriginalRenderDrawer mOriginalDrawer;
    private WaterMarkRenderDrawer mWaterMarkDrawer;
    private DisplayRenderDrawer mDisplayDrawer;
    private RecordRenderDrawer mRecordDrawer;
    private Ball ball;

    public RenderDrawerGroups(Context context) {
        this.mOriginalDrawer = new OriginalRenderDrawer();
        this.mWaterMarkDrawer = new WaterMarkRenderDrawer(context);
        this.mDisplayDrawer = new DisplayRenderDrawer();
        this.mRecordDrawer = new RecordRenderDrawer(context);
        this.ball = new Ball(context);
        this.mFrameBuffer = 0;
        this.mInputTexture = 0;
    }

    public void setInputTexture(int texture) {
        this.mInputTexture = texture;
    }

    public void bindFrameBuffer(int textureId) {
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, mFrameBuffer);
        GLES30.glFramebufferTexture2D(GLES30.GL_FRAMEBUFFER, GLES30.GL_COLOR_ATTACHMENT0, GLES30.GL_TEXTURE_2D, textureId, 0);

        GLES30.glFramebufferRenderbuffer(GLES30.GL_FRAMEBUFFER, GLES30.GL_DEPTH_STENCIL_ATTACHMENT, GLES30.GL_RENDERBUFFER, mDepthStencilRender);

        if (GLES30.glCheckFramebufferStatus(GLES30.GL_FRAMEBUFFER) != GLES30.GL_FRAMEBUFFER_COMPLETE) {
            Log.e("bindFrameBuffer","ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
        }
    }

    public void unBindFrameBuffer() {
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0);
    }

    public void deleteFrameBuffer() {
        GLES30.glDeleteFramebuffers(1, new int[]{mFrameBuffer}, 0);
        GLES30.glDeleteTextures(1, new int[]{mInputTexture}, 0);
    }

    public void create() {
        this.mOriginalDrawer.create();
        this.mWaterMarkDrawer.create();
        this.mDisplayDrawer.create();
        this.mRecordDrawer.create();
        this.ball.create();
        MagicPenJNI.init();
    }

    public int createDepthStencilRenderBuffer(int width, int height) {
        IntBuffer rbo = IntBuffer.allocate(1);
        GLES30.glGenRenderbuffers(1, rbo);
        GLES30.glBindRenderbuffer(GLES30.GL_RENDERBUFFER, rbo.get(0));
        GLES30.glRenderbufferStorage(GLES30.GL_RENDERBUFFER, GLES30.GL_DEPTH24_STENCIL8, width, height);
        return  rbo.get(0);
    }

    public void surfaceChangedSize(int width, int height) {
        mFrameBuffer = GlesUtil.createFrameBuffer();
        mOriginalDrawer.surfaceChangedSize(width, height);
        mWaterMarkDrawer.surfaceChangedSize(width, height);
        mDisplayDrawer.surfaceChangedSize(width, height);
        mRecordDrawer.surfaceChangedSize(width, height);

        this.mOriginalDrawer.setInputTextureId(mInputTexture);
        int textureId = this.mOriginalDrawer.getOutputTextureId();
        mWaterMarkDrawer.setInputTextureId(textureId);
        mDisplayDrawer.setInputTextureId(textureId);
        mRecordDrawer.setInputTextureId(textureId);

        mDepthStencilRender = createDepthStencilRenderBuffer(width, height);
    }

    public void drawRender(BaseRenderDrawer drawer, boolean useFrameBuffer, long timestamp, float[] transformMatrix) {
        if (useFrameBuffer) {
            bindFrameBuffer(drawer.getOutputTextureId());
        }
        drawer.draw(timestamp, transformMatrix);
        if (useFrameBuffer) {
            unBindFrameBuffer();
        }
    }

    public void draw(long timestamp, float[] transformMatrix) {
        if (mInputTexture == 0 || mFrameBuffer == 0) {
            Log.e(TAG, "draw: mInputTexture or mFramebuffer or list is zero");
            return;
        }
        drawRender(mOriginalDrawer, true, timestamp, transformMatrix);
        // 绘制顺序会控制着 水印绘制哪一层

        // 画球
        bindFrameBuffer(this.mOriginalDrawer.getOutputTextureId());
        //ball.draw();
        double seconds = timestamp / 1000000000.0;
        MagicPenJNI.draw(seconds);
        unBindFrameBuffer();

        //drawRender(mWaterMarkDrawer, true, timestamp, transformMatrix);
        drawRender(mDisplayDrawer, false,  timestamp, transformMatrix);
    }

    public void startRecord() {
        mRecordDrawer.startRecord();
    }

    public void stopRecord() {
        mRecordDrawer.stopRecord();
    }
}
