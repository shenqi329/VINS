//
// Created by DELL on 2022/8/4.
//

#ifndef VINS_MAGICPENRENDER_H
#define VINS_MAGICPENRENDER_H

#include <GLES3/gl3.h>
#include <GLES3/gl3ext.h>
#include <GLES3/gl3platform.h>

#include "MagicPen3DModel.h"

class MagicPenRender {
public:
    ~MagicPenRender();

    void Init();

    void Draw(MagicPen3DModel *model, float timeStampSec);
private:

    void InitProgram();
    void InitBuffer();

    void GenTextures();

    GLuint _programObject;

    GLuint _VBO, _VAO, _EBO;

    GLuint _VBO_edge, _VAO_edge, _EBO_edge;

    GLuint  _texture;
};


#endif //VINS_MAGICPENRENDER_H
