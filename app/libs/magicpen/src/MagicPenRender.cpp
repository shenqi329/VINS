//
// Created by DELL on 2022/8/4.
//

#ifdef ANDROID
#include <android/log.h>
#define LOG_TAG "MagicPenRender"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#define LOGE(...)
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "MagicPenRender.h"

GLuint LoadShader ( GLenum type, const char *shaderSrc )
{
    GLuint shader;
    GLint compiled;

    // Create the shader object
    shader = glCreateShader ( type );

    if ( shader == 0 )
    {
        return 0;
    }

    // Load the shader source
    glShaderSource ( shader, 1, &shaderSrc, NULL );

    // Compile the shader
    glCompileShader ( shader );

    // Check the compile status
    glGetShaderiv ( shader, GL_COMPILE_STATUS, &compiled );

    if ( !compiled )
    {
        GLint infoLen = 0;

        glGetShaderiv ( shader, GL_INFO_LOG_LENGTH, &infoLen );

        if ( infoLen > 1 )
        {
            char *infoLog = (char *)malloc ( sizeof ( char ) * infoLen );

            glGetShaderInfoLog ( shader, infoLen, NULL, infoLog );
            LOGE("Error compiling shader:[%s]", infoLog );

            free ( infoLog );
        }

        glDeleteShader ( shader );
        return 0;
    }

    return shader;

}

MagicPenRender::~MagicPenRender() {
    glDeleteVertexArrays(1, &_VAO);
    glDeleteBuffers(1, &_VBO);
    glDeleteBuffers(1, &_EBO);

    glDeleteVertexArrays(1, &_VAO_edge);
    glDeleteBuffers(1, &_VBO_edge);
    glDeleteBuffers(1, &_EBO_edge);
}

void MagicPenRender::Init() {
    InitProgram();
    InitBuffer();
}

void MagicPenRender::GenTextures() {

}

void MagicPenRender::InitProgram() {
    char vShaderStr[] =
            "#version 300 es                                            \n"
            "layout (location = 0) in vec3 aPos;                        \n"
            "layout (location = 1) in vec3 aColor;                      \n"
            "layout (location = 2) in vec2 aTexCoord;                   \n"
            "out vec3 ourColor;                                         \n"
            "out vec2 TexCoord;                                         \n"
            "uniform mat4 model;                                        \n"
            "uniform mat4 view;                                         \n"
            "uniform mat4 projection;                                   \n"
            "void main()                                                \n"
            "{                                                          \n"
            "gl_Position = projection * view *model * vec4(aPos, 1.0);  \n"
            "ourColor = aColor;                                         \n"
            "TexCoord = vec2(aTexCoord.x, aTexCoord.y);                 \n"
            "}                                                          \n";

    char fShaderStr[] =
            "#version 300 es                            \n"
            "out vec4 FragColor;                        \n"
            "in vec3 ourColor;                          \n"
            "in vec2 TexCoord;                          \n"
            "uniform sampler2D texture1;                \n"
            "void main()                                \n"
            "{                                          \n"
            "FragColor = texture(texture1, TexCoord);   \n"
            "}                                          \n";

    GLuint vertexShader;
    GLuint fragmentShader;
    GLuint programObject;
    GLint linked;

    vertexShader = LoadShader (GL_VERTEX_SHADER, vShaderStr);
    fragmentShader = LoadShader (GL_FRAGMENT_SHADER, fShaderStr);

    programObject = glCreateProgram();

    if ( programObject == 0 ) {
        LOGE("glCreateProgram Fail");
        return;
    }

    glAttachShader ( programObject, vertexShader );
    glAttachShader ( programObject, fragmentShader );

    // Link the program
    glLinkProgram ( programObject );

    // Check the link status
    glGetProgramiv ( programObject, GL_LINK_STATUS, &linked );

    if ( !linked )
    {
        GLint infoLen = 0;

        glGetProgramiv ( programObject, GL_INFO_LOG_LENGTH, &infoLen );

        if ( infoLen > 1 )
        {
            char *infoLog = (char *)malloc ( sizeof ( char ) * infoLen );

            glGetProgramInfoLog ( programObject, infoLen, NULL, infoLog );
            LOGE("Error linking program:[%s]", infoLog );

            free ( infoLog );
        }

        glDeleteProgram ( programObject );
        return;
    }

    // Store the program object
    _programObject = programObject;
}

void MagicPenRender::InitBuffer() {

    glUseProgram(_programObject);

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)320 / (float)360, 0.1f, 100.0f);
    glUniformMatrix4fv(glGetUniformLocation(_programObject, "projection"), 1, GL_FALSE, &projection[0][0]);

    glGenVertexArrays(1, &_VAO);
    glGenBuffers(1, &_VBO);
    glGenBuffers(1, &_EBO);

    glGenVertexArrays(1, &_VAO_edge);
    glGenBuffers(1, &_VBO_edge);
    glGenBuffers(1, &_EBO_edge);

}

void MagicPenRender::Draw(MagicPen3DModel *pModel, float timeStampSec) {

    if (!pModel->_image_rgba.data) {
        return;
    }

    // load and create a texture
    // -------------------------
    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture); // all upcoming GL_TEXTURE_2D operations now have effect on this texture object
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps

    if (pModel->_image_rgba.data) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, pModel->_image_rgba.cols, pModel->_image_rgba.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, pModel->_image_rgba.data);
        glGenerateMipmap(GL_TEXTURE_2D);
    } else {
        LOGE("Failed to load texture");
    }

    glBindVertexArray(_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, _VBO);
    glBufferData(GL_ARRAY_BUFFER, pModel->_vertices_front_size, pModel->_vertices_front, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, pModel->_indices_front_size, pModel->_indices_front, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(_VAO_edge);

    glBindBuffer(GL_ARRAY_BUFFER, _VBO_edge);
    glBufferData(GL_ARRAY_BUFFER, pModel->_vertices_side_size, pModel->_vertices_side, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO_edge);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, pModel->_indices_side_size, pModel->_indices_side, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // render
    // ------
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    //glViewport(0, 0, 320, 360);

    // bind Texture
    glBindTexture(GL_TEXTURE_2D, texture);

    // render container
    glUseProgram(_programObject);

    // camera/view transformation
    glm::mat4 view = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
    float radius = 3.0f;

#if 1
    float camX   = sin(0) * radius;
    float camZ   = cos(0) * radius;
#else
    float camX   = sin(0) * radius;
    float camZ   = cos(0) * radius;
#endif
#ifdef ANDROID
    glm::vec3 up = glm::vec3(0.0f, -1.0f, 0.0f);
#else
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
#endif
    view = glm::lookAt(glm::vec3(camX, 0.0f, camZ), glm::vec3(0.0f, 0.0f, 0.0f), up);
    glUniformMatrix4fv(glGetUniformLocation(_programObject, "view"), 1, GL_FALSE, &view[0][0]);

    glm::mat4 model;

    glBindVertexArray(_VAO);
    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f));
    //model = glm::rotate(model, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
    glUniformMatrix4fv(glGetUniformLocation(_programObject, "model"), 1, GL_FALSE, &model[0][0]);
    glDrawElements(GL_TRIANGLES, pModel->_indices_front_size / sizeof(int), GL_UNSIGNED_INT, 0);

    // bind Texture
    //glBindTexture(GL_TEXTURE_2D, texture_edge);

    glBindVertexArray(_VAO_edge);
    model = glm::mat4(1.0f);
    //model = glm::rotate(model, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
    glUniformMatrix4fv(glGetUniformLocation(_programObject, "model"), 1, GL_FALSE, &model[0][0]);
    glDrawElements(GL_TRIANGLES, pModel->_indices_side_size / sizeof(int), GL_UNSIGNED_INT, 0);

    glDeleteTextures(1, &texture);
}