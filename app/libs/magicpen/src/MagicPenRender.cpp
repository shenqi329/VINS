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

#include <stb_image.h>
#include <learnopengl/filesystem.h>

#endif

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
	GenTextures();
}

void MagicPenRender::GenTextures() {
    int width_edge, height_edge, nrChannels_edge;
    unsigned char *data_edge = nullptr;
#ifdef _WIN32
	// The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
	data_edge = stbi_load(FileSystem::getPath("resources/textures/edge.png").c_str(), &width_edge, &height_edge, &nrChannels_edge, 0);
#elif defined(ANDROID)
    data_edge = _texture_edge_image_rgba.data;
    width_edge = _texture_edge_image_rgba.cols;
    height_edge = _texture_edge_image_rgba.rows;
#endif
    glGenTextures(1, &_texture_edge);
    glBindTexture(GL_TEXTURE_2D, _texture_edge); // all upcoming GL_TEXTURE_2D operations now have effect on this texture object
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    if (data_edge) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_edge, height_edge, 0, GL_RGBA, GL_UNSIGNED_BYTE, data_edge);
        glGenerateMipmap(GL_TEXTURE_2D);
#ifdef _WIN32
        stbi_image_free(data_edge);
#endif
    }
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

void MagicPenRender::SetTextureEdgeImage(cv::Mat texture_edge_image_rgba) {
    _texture_edge_image_rgba = texture_edge_image_rgba;
}

void MagicPenRender::Draw(MagicPen3DModel *model, double timeStampSec, float cameraRotateX, float CameraRotateY) {
	for (size_t i = 0; i < model->_3dModels.size(); i++) {
		Draw(model->_3dModels[i],  timeStampSec, cameraRotateX, CameraRotateY);
	}
}

void MagicPenRender::Draw(MagicPen3DLimbModel *pModel, double timeStampSec, float cameraRotateX, float CameraRotateY) {

    if (!pModel->_image_rgba.data) {
        return;
    }
	if (!_standInit) {
		_standInit = true;
		_init_rotate_x = 90 + cameraRotateX;
		_init_time = (float)timeStampSec;
		_stand_model = glm::mat4(1.0f);
	} else {
		if (timeStampSec - _init_time < 1.f ) {
			_stand_model = glm::mat4(1.0f);
			float angle = - ((90 + cameraRotateX) * (timeStampSec - _init_time) / 1.f);
			_stand_model = glm::rotate(_stand_model, glm::radians(angle), glm::vec3(1.0f, 0.0f, 0.0f));
		} else {
            _stand_model = glm::mat4(1.0f);
            float angle = - (90 + cameraRotateX);
            _stand_model = glm::rotate(_stand_model, glm::radians(angle), glm::vec3(1.0f, 0.0f, 0.0f));
        }
	}

    float scale = 1.0f;//curMinAreaRect.size.width / beginMinAreaRect.size.width;

    glEnable(GL_DEPTH_TEST);

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
    glClearColor(1.0f, 1.0f, 1.0f, .0f);

    glClear(GL_DEPTH_BUFFER_BIT);
    // bind Texture
    glBindTexture(GL_TEXTURE_2D, texture);

    // render container
    glUseProgram(_programObject);

    // camera/view transformation
    glm::mat4 view = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
    float angle = 0;//timeStampSec * 20;

#ifdef ANDROID
    glm::vec3 up = glm::vec3(0.0f, -1.0f, 0.0f);
#else
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
#endif
    view = glm::lookAt(glm::vec3(0.0f, 0.0f, -3.0f), glm::vec3(0.0f, 0.0f, 0.0f), up);

    glUniformMatrix4fv(glGetUniformLocation(_programObject, "view"), 1, GL_FALSE, &view[0][0]);

    glm::mat4 model;

    glBindVertexArray(_VAO);
    model = glm::mat4(1.0f);
    model = glm::rotate(model, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model,glm::vec3(scale, scale, scale));
	model = _stand_model * model;
    //model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f));
    glUniformMatrix4fv(glGetUniformLocation(_programObject, "model"), 1, GL_FALSE, &model[0][0]);
    glDrawElements(GL_TRIANGLES, pModel->_indices_front_size / sizeof(int), GL_UNSIGNED_INT, 0);

	glBindVertexArray(_VAO);
	model = glm::mat4(1.0f);
    model = glm::rotate(model, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model,glm::vec3(scale, scale, scale));
	model = _stand_model * model;
	model = glm::translate(model, glm::vec3(0.0f, 0.0f, -0.10001f));
	glUniformMatrix4fv(glGetUniformLocation(_programObject, "model"), 1, GL_FALSE, &model[0][0]);
    glDrawElements(GL_TRIANGLES, pModel->_indices_front_size / sizeof(int), GL_UNSIGNED_INT, 0);

    // bind Texture
    glBindTexture(GL_TEXTURE_2D, _texture_edge);

    glBindVertexArray(_VAO_edge);
    model = glm::mat4(1.0f);
    model = glm::rotate(model, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model,glm::vec3(scale, scale, scale));
	model = _stand_model * model;
    glUniformMatrix4fv(glGetUniformLocation(_programObject, "model"), 1, GL_FALSE, &model[0][0]);
    glDrawElements(GL_TRIANGLES, pModel->_indices_side_size / sizeof(int), GL_UNSIGNED_INT, 0);

    glDeleteTextures(1, &texture);

    glDisable(GL_DEPTH_TEST);
}