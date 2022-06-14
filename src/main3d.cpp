
// This example is heavily based on the tutorial at https://open.gl

#include <vector>
#include <cmath>
#include <string>
// OpenGL Helpers to reduce the clutter
#include "Helpers.h"
#include <fstream>
#include <iostream>
// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mesh.h"

// Timer
#include <chrono>
using namespace Eigen;
using namespace std;
MatrixXf projection(4, 4);
MatrixXf camera(4, 4);

VertexArrayObject VAO;

// This parameter controls the search radius around each vertex
double radius = 0.2;
bool usePaperMethod = true;
int stepCount = 16;
int export_count = 0;
int target_triangle_count = -1; // negative means this option is not used

struct Object
{
    Mesh mesh;
    Transform<float, 3, Affine> transform;
    VertexBufferObject VBO; // vertex buffer object (vertices only)
    VertexBufferObject NBO; // normal buffer object (normal)
    Vector3d color;
};

// Create a list of object
std::vector<Object> Objects;

float Zoom = 1;
Vector2d pan(0, 0);

float camera_angle = 0;
float camera_elevation = 0;
float camera_distance = -0.1;

enum Mode {
    NONE,
    UNITCUBE,
    BUMPYCUBE,
    BUNNY,
    TRANSLATION,
    DELETION,
    COLOR_CHANGE,
    WIREFRAME,
    FLATSHADING,
    PHONGSHADING,
    ORTHOGRAPHIC,
    PERSPECTIVE
};

Mode mode = UNITCUBE;
Mode lighting_mode = FLATSHADING;
Mode projection_mode = ORTHOGRAPHIC;

const float canvas_size = 3;


void setProjection(float fovY, float aspectRatio, float N, float F, float r = -canvas_size, float l = canvas_size, float t = canvas_size, float b = -canvas_size)
{
    projection = MatrixXf::Identity(4, 4);
    if (projection_mode == PERSPECTIVE) {

        projection <<
            2 * N / (r - l), 0, (r + l) / (r - l), 0,
            0, 2 * N / (t - b), (t + b) / (t - b), 0,
            0, 0, (F + N) / (N - F), (2 * N * F) / (N - F),
            0, 0, -1, 0;
    }
    else {
        projection <<
            2 / (r - l), 0, 0, 0,
            0, 2 / (t - b), 0, 0,
            0, 0, -2 / (F - N), 0,
            -(r + l) / (r - l), -(t + b) / (t - b), -(F + N) / (F - N), 1;
    }
}

void makeCameraMatrix() {
    Vector3d camera_position;
    camera_position.z() = cos(camera_angle) * camera_distance;
    camera_position.x() = sin(camera_angle) * camera_distance;
    camera_position.y() = camera_elevation;

    Vector3d world_origin = Vector3d(0, 0, 0);

    Vector3d up_vector = Vector3d(0, 1, 0);
    Vector3d forward_vector = -(world_origin - camera_position).normalized();
    if (world_origin == camera_position) {
        forward_vector = Vector3d(0, 0, 1);
    }
    Vector3d right_vector = up_vector.cross(forward_vector);
    up_vector = forward_vector.cross(right_vector);

    /*
    camera << right_vector.x(), right_vector.y(), right_vector.z(), -right_vector.dot(camera_position),
      up_vector.x(), up_vector.y(), up_vector.z(), -up_vector.dot(camera_position),
      forward_vector.x(), forward_vector.y(), forward_vector.z(), -forward_vector.dot(camera_position),
      0, 0, 0, 1;
    */
    camera << right_vector.x(), up_vector.x(), forward_vector.x(), -right_vector.dot(camera_position),
        right_vector.y(), up_vector.y(), forward_vector.y(), -up_vector.dot(camera_position),
        right_vector.z(), up_vector.z(), forward_vector.z(), -forward_vector.dot(camera_position),
        0, 0, 0, 1;
}

Vector2d getMousePosition(GLFWwindow* window) {
    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    float aspectRatio = float(height) / float(width);

    // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Convert screen position to world coordinates
    double xworld = ((xpos / double(width)) * 2) - 1;
    double yworld = (((height - 1 - ypos) / double(height)) * 2) - 1; // NOTE: y axis is flipped in glfw


    // new_position = position * zoom + pan
    // Let's invert the function (step by step)
    // new_position = position * zoom + pan
    // new_position - pan = position * zoom 
    // (new_position - pan) / zoom = position
    // So I you can see, you have to first substract pan and then divide by zoom
    // Before you were cancelling the zoom and the pan

    // Cancel the translation byy applying the inverse (not a division. It's substraction)
    xworld -= pan.x();
    yworld -= pan.y();

    // Cancel the zoom by applying the inverse
    xworld /= Zoom;
    yworld /= Zoom;

    return Vector2d(xworld, yworld);
}

void updateNBO(Object& O) {
    MatrixXf normal_matrix(3, O.mesh.normals.size() * 3);
    normal_matrix.resize(3, O.mesh.normals.size() * 3);

    for (int i = 0; i < O.mesh.normals.size(); ++i) {
        //generateNormlas(O.triangles);
        for (int j = 0; j < 3; ++j)
        {
            normal_matrix(0, i * 3 + j) = O.mesh.normals[i].x();
            normal_matrix(1, i * 3 + j) = O.mesh.normals[i].y();
            normal_matrix(2, i * 3 + j) = O.mesh.normals[i].z();
        }
    }
    O.NBO.update(normal_matrix);
}

void updateAllNBOs() {
    for (int i = 0; i < Objects.size(); ++i)
    {
        updateNBO(Objects[i]);
    }
}

void FillVBOObject(Object& O)
{
    MatrixXf triangle_matrix(3, O.mesh.triangles.size() * 3);
    triangle_matrix.resize(3, O.mesh.triangles.size() * 3);
    for (int i = 0; i < O.mesh.triangles.size(); ++i) {
        Triangle t = getTriangle(O.mesh, i);
        triangle_matrix(0, i * 3 + 0) = O.mesh.vertices[t.a].point.x();
        triangle_matrix(1, i * 3 + 0) = O.mesh.vertices[t.a].point.y();
        triangle_matrix(2, i * 3 + 0) = O.mesh.vertices[t.a].point.z();

        triangle_matrix(0, i * 3 + 1) = O.mesh.vertices[t.b].point.x();
        triangle_matrix(1, i * 3 + 1) = O.mesh.vertices[t.b].point.y();
        triangle_matrix(2, i * 3 + 1) = O.mesh.vertices[t.b].point.z();

        triangle_matrix(0, i * 3 + 2) = O.mesh.vertices[t.c].point.x();
        triangle_matrix(1, i * 3 + 2) = O.mesh.vertices[t.c].point.y();
        triangle_matrix(2, i * 3 + 2) = O.mesh.vertices[t.c].point.z();
    }
    O.VBO.update(triangle_matrix);

    updateNBO(O);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    // Get the position of the mouse in the window
    Vector2d mouse_pos = getMousePosition(window);
    // Update the position of the first vertex if the left button is pressed
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
        }
        if (action == GLFW_RELEASE) {
        }
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    if (action == GLFW_PRESS)
    {
        switch (key)
        {

        case GLFW_KEY_KP_3: case GLFW_KEY_3:
        {
            Object bunny;
            bunny.mesh = loadOff(DATA_FOLDER "/bunny.off", radius);
            bunny.transform = bunny.transform.Identity();
            bunny.color = Vector3d(1, 1, 1);
            bunny.VBO.init();
            bunny.NBO.init();
            FillVBOObject(bunny);
            Objects.push_back(bunny);
            if (target_triangle_count >= 0) {
                Mesh simplified = simplifyToTarget(bunny.mesh, target_triangle_count);
                std::cout << "Bunny mesh created\n";
                std::cout << "Original Bunny mesh triangle count " << getTriangleCount(bunny.mesh) << "\n";
                std::cout << "Target triangle count " << target_triangle_count << "\n";
                std::cout << "Simplified Bunny mesh triangle count " << getTriangleCount(simplified) << "\n";
                string file_name = "simplified_bunny_obj" + to_string(export_count) + ".obj";
                Export(simplified, file_name);
            }
        }
        break;
        case GLFW_KEY_KP_2: case GLFW_KEY_2:
        {
            Object bumpy_cube;
            bumpy_cube.mesh = loadOff(DATA_FOLDER "/bumpy_cube.off", radius);
            bumpy_cube.transform = bumpy_cube.transform.Identity();
            bumpy_cube.color = Vector3d(1, 1, 1);
            bumpy_cube.VBO.init();
            bumpy_cube.NBO.init();
            FillVBOObject(bumpy_cube);
            Objects.push_back(bumpy_cube);
            if (target_triangle_count >= 0) {
                Mesh simplified = simplifyToTarget(bumpy_cube.mesh, target_triangle_count);
                std::cout << "bumpy_cube mesh created\n";
                std::cout << "Original bumpy_cube mesh triangle count " << getTriangleCount(bumpy_cube.mesh) << "\n";
                std::cout << "Target triangle count " << target_triangle_count << "\n";
                std::cout << "Simplified bumpy_cube mesh triangle count " << getTriangleCount(simplified) << "\n";
                string file_name = "simplified_bumpy_cube_obj" + to_string(export_count) + ".obj";
                Export(simplified, file_name);
            }
        }
        break;
        case  GLFW_KEY_SPACE:
        {
            for (auto& obj : Objects) {
                simplify(obj.mesh, stepCount, usePaperMethod);
                FillVBOObject(obj);
            }
            break;
        }
        case  GLFW_KEY_W:
        {
            if (lighting_mode == WIREFRAME) {
                lighting_mode = FLATSHADING;
            }
            else {
                lighting_mode = WIREFRAME;
            }

            break;
        }
        case GLFW_KEY_E:
        {
            for (int i = 0; i < Objects.size(); ++i) {
                string file_name = "shot_obj" + to_string(i) + "_" + to_string(export_count) + ".obj";
                Export(Objects[i].mesh, file_name);
            }
            export_count++;
        }
        case  GLFW_KEY_ESCAPE:
        {
            mode = NONE;
            break;
        }
        default:
            break;
        }
    }

    const float camera_angle_step = 0.1;
    switch (key) {
    case GLFW_KEY_RIGHT:
        camera_angle += camera_angle_step;
        makeCameraMatrix();
        break;
    case GLFW_KEY_LEFT:
        camera_angle -= camera_angle_step;
        makeCameraMatrix();
        break;
    default:
        break;
    }
}

int main(int argc, char** argv)
{
    if (argc == 2) {
        // If a parameter is passed to the program, it's used as target
        target_triangle_count = std::stoi(argv[1]);
        cout << "The target triangle count is " << target_triangle_count << "\n";
    }

    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

#ifndef __APPLE__
    glewExperimental = true;
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        /* Problem: glewInit failed, something is seriously wrong. */
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }
    glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
    fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
#endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    // Initialize the VAO
    // A Vertex Array Object (or VAO) is an object that describes how the vertex
    // attributes are stored in a Vertex Buffer Object (or VBO). This means that
    // the VAO is not the actual object storing the vertex data,
    // but the descriptor of the vertex data.
    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    /*
    Object cube;
    cube.triangles = makeCube();
    cube.transform = cube.transform.Identity();
    cube.transform.scale(0.5);
    */


    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
        "#version 150 core\n"
        "in vec3 position;"
        "in vec3 normal;"
        "out vec3 vs_normal;"
        "uniform float zoom;" // scale
        "uniform vec2 pan;" // translation
        "uniform mat4 model_view_proj;" // combined model, view and projection matrices
        "uniform mat4 model_view;" // combined model, view and projection matrices
        "void main()"
        "{"
        "    vs_normal = mat3(model_view) * normal;"
        "    vs_normal = normalize(vs_normal);"
        "    gl_Position = model_view_proj * vec4(position, 1.0);"
        "    gl_Position = vec4(gl_Position.xy * zoom + pan, gl_Position.zw);"
        "}";

    const GLchar* fragment_shader =
        "#version 150 core\n"
        "uniform vec4 color;" // the alpha of zero means light is off and 1 means lighting is on
        "out vec4 outColor;"
        "in vec3 vs_normal;"
        "void main()"
        "{"
        "    vec3 n = normalize(vs_normal);"
        "    if (color.a == 0) "
        "      outColor = vec4(0.5);"
        "    else "
        "      outColor = vec4(color.rgb * abs(n.z), 1.0);"
        "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader, fragment_shader, "outColor");
    program.bind();

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    makeCameraMatrix();

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window))
    {
        glEnable(GL_DEPTH_TEST);

        // Update viewport
        int vw, vh;
        glfwGetFramebufferSize(window, &vw, &vh);
        glViewport(0, 0, vw, vh);

        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        // Bind your program
        program.bind();

        // Set the uniform value depending on the time difference
        auto t_now = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();

        // Clear the framebuffer
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // TODO an options to switch between wireframe and shaded triangle
        if (lighting_mode == WIREFRAME) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
        else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        glUniform1f(program.uniform("zoom"), Zoom); // a scale of 1 means no zoom
        // TODO pass the translation to the vertex shader
        glUniform2f(program.uniform("pan"), pan.x(), pan.y()); // a zero translation means no pan

        setProjection(45, float(vw) / vw, 0.1, 100);
        MatrixXf view_proj = projection * camera;

        // Draw
        // TODO put this in a loop for all objects
        for (int i = 0; i < Objects.size(); ++i)
        {
            MatrixXf model_view_proj = view_proj * Objects[i].transform.matrix();
            MatrixXf model_view = camera * Objects[i].transform.matrix();
            glUniformMatrix4fv(program.uniform("model_view_proj"), 1, false, model_view_proj.data());
            glUniformMatrix4fv(program.uniform("model_view"), 1, false, model_view.data());
            program.bindVertexAttribArray("normal", Objects[i].NBO);
            program.bindVertexAttribArray("position", Objects[i].VBO);
            glUniform4f(program.uniform("color"), Objects[i].color.x(), Objects[i].color.y(), Objects[i].color.z(), 1);
            glDrawArrays(GL_TRIANGLES, 0, Objects[i].mesh.triangles.size() * 3);
        }

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    // Deallocate opengl memory
    program.free();
    VAO.free();
    for (int i = 0; i < Objects.size(); ++i) {
        Objects[i].VBO.free();
        Objects[i].NBO.free();
    }

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}
