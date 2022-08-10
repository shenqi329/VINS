//
// Created by DELL on 2022/8/4.
//

#ifndef VINS_MAGICPENRENDER_H
#define VINS_MAGICPENRENDER_H

#ifdef ANDROID
#include <GLES3/gl3.h>
#include <GLES3/gl3ext.h>
#include <GLES3/gl3platform.h>
#else
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#endif

#include <glm/glm.hpp>

#include "MagicPen3DModel.h"

class MagicPenRender {
public:
    ~MagicPenRender();

    void Init();

    void SetTextureEdgeImage(cv::Mat texture_edge_image_rgba);

	void Draw(MagicPen3DModel *model, double timeStampSec, float cameraRotateX, float CameraRotateY);
private:
    void Draw(MagicPen3DLimbModel *model, double timeStampSec, float cameraRotateX, float CameraRotateY);

    void InitProgram();
    void InitBuffer();

    void GenTextures();

    GLuint _programObject;

    GLuint _VBO, _VAO, _EBO;

    GLuint _VBO_edge, _VAO_edge, _EBO_edge;

    GLuint  _texture_edge;

    cv::Mat _texture_edge_image_rgba;

	bool _standInit = false;
	float _init_rotate_x;
	float _init_time;
	glm::mat4 _stand_model;
};


#endif //VINS_MAGICPENRENDER_H
