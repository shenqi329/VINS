package com.example.vins;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;

public class MagicPenDrawLineView extends View {

    private Point[] _points;

    public MagicPenDrawLineView(Context context) {
        super(context);
    }

    public MagicPenDrawLineView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
    }

    public MagicPenDrawLineView(Context context, @Nullable AttributeSet attrs,
                          int defStyleAttr) {
        super(context, attrs, defStyleAttr);
    }

    void  SetLinePoints(Point[] points) {
        _points = points;
        invalidate();
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        //drawCenterLineX(canvas);
        //drawCenterLineY(canvas);
        drawPoints(canvas);
    }

    private void drawPoints(Canvas canvas) {
        if (null == _points) {
            return;
        }
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.BLUE);
        paint.setStrokeWidth(2);

        int[] color = new int[4];
        color[0] = Color.RED;
        color[1] = Color.GREEN;
        color[2] = Color.BLUE;
        color[3] = Color.BLACK;

        for (int i = 0 ; i < _points.length; ++i) {
            paint.setColor(color[i]);
            if (i == _points.length - 1) {
                canvas.drawLine(_points[i].x, _points[i].y, _points[0].x, _points[0].y, paint);
            } else {
                canvas.drawLine(_points[i].x, _points[i].y, _points[i + 1].x, _points[i + 1].y, paint);
            }
        }
    }

    /**
     * 绘制x轴的中间竖线
     */
    private void drawCenterLineX(Canvas canvas){
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.BLUE);
        paint.setStrokeWidth(2);
        canvas.drawLine(getWidth()/2, 0, getWidth()/2, getHeight(), paint);
    }

    /**
     * 绘制y轴的中间横线
     */
    private void drawCenterLineY(Canvas canvas){
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.BLUE);
        paint.setStrokeWidth(2);
        canvas.drawLine(0, getHeight()/2, getWidth(), getHeight()/2, paint);
    }
}
