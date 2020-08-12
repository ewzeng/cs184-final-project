/***********************************************************************
 * This file is a heavily adapted mix of CS184 Project 4 code as well as
 * code from learnopengl.com. The code from learnopengl.com is under the
 * CC BY 4.0 license published by Creative Commons. A link to the license
 * can be found here: https://creativecommons.org/licenses/by-nc/4.0/legalcode
 *************************************************************************/

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <CGL/vector3D.h>
#include <nanogui/nanogui.h>
#include <iostream>

#include "camera.h"
#include "shader_s.h"
#include "fluid.h"
#include "collision/plane.h"

using namespace nanogui;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
Matrix4f getProjectionMatrix();
Matrix4f getViewMatrix();

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// simulation variables
bool is_paused = true;
int frames_per_sec = 15; // CHANGE LATER - JUST TEMP HERE TO SLOW DOWN THE ANIMATION
int simulation_steps = 2;

#define NUM_PARTICLES 1000

// scene variables
CGL::Camera camera;
Fluid fluid;
FluidParameters fp;
vector<Plane*> objects;
vector<Vector3D> external_accelerations;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Fluid Sim", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    
    // set up so window is notified during keypress
    glfwSetKeyCallback(window, key_callback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }


    // build and compile our shader program
    // location differs in linux and windows
    // ------------------------------------
    Shader ourShader("../../../shaders/particle.vert", "../../../shaders/particle.frag"); 

    // initalize fluid and simulation variables
    // later we can support reading in parameters from json files
    // ----------------------------------------------------------
    fluid = Fluid(NUM_PARTICLES);
    float vertices[NUM_PARTICLES * 3];
    for (int i = 0; i < NUM_PARTICLES; i++) {
        vertices[i * 3] = fluid.particles[i].position.x;
        vertices[i * 3 + 1] = fluid.particles[i].position.y;
        vertices[i * 3 + 2] = fluid.particles[i].position.z;
    }

    fp = FluidParameters(1);
    external_accelerations.emplace_back(0, -9.8, 0);

    // set up some collision objects
    Plane bottom = Plane(Vector3D(0, -0.2, 0), Vector3D(0, 1, 0), 0.3);
    objects.push_back(&bottom);
    Plane back = Plane(Vector3D(0, 0, -1), Vector3D(0, 0, 1), 0.3);
    objects.push_back(&back);
    Plane right = Plane(Vector3D(0.2, 0, 0), Vector3D(-1, 0, 0), 0.3);
    objects.push_back(&right);
    Plane left = Plane(Vector3D(-0.2, 0, 0), Vector3D(1, 0, 0), 0.3);
    objects.push_back(&left);
    Plane front = Plane(Vector3D(0, 0, 0.1), Vector3D(0, 0, -1), 0.3);
    objects.push_back(&front);

    // set a cover for testing
    Plane top = Plane(Vector3D(0, 0.55, 0), Vector3D(0, -1, 0), 0.5);
    objects.push_back(&top);

    // set up OpenGL and configure OpenGL buffer objects with data
    // ------------------------------------------------------------
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // bind the Vertex Array Object first
    // as only one VAO is used in this code, we don't need to worry about unbinding
    glBindVertexArray(VAO);

    // bind, setup vertex buffer, and fill with data
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // enable drawing points
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(10);

    // enable depth, Z-buffer
    glEnable(GL_DEPTH_TEST);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO 
    // as the vertex attribute's bound vertex buffer object so afterwards we can safely
    // unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // activate shader
    // ---------------
    ourShader.use();

    // set up camera and perspective
    // -----------------------------
    CGL::Collada::CameraInfo camera_info;
    camera_info.hFov = 80;
    camera_info.vFov = 80;
    camera_info.nClip = 0.01;
    camera_info.fClip = 10000;
    CGL::Vector3D target(0., 0., 0.);
    
    // direction of camera from target (i.e. target -> camera direction)
    CGL::Vector3D c_dir(0., 0., 1);
    c_dir = c_dir.unit();

    camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), 1.0, 0.2, 20.);
    camera.configure(camera_info, SCR_WIDTH, SCR_HEIGHT);

    // calculate and input the perspective transformations into the shader
    Matrix4f view = getViewMatrix();
    Matrix4f projection = getProjectionMatrix();
    Matrix4f viewProjection = projection * view;
    ourShader.setMat4("u_view_projection", viewProjection);

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear buffers

        if (!is_paused) {
            // update positions of vertices
            for (int i = 0; i < simulation_steps; i++) {
                fluid.simulate(frames_per_sec, simulation_steps, &fp, external_accelerations, &objects);
            }  
            for (int i = 0; i < NUM_PARTICLES; i++) {
                vertices[i * 3] = fluid.particles[i].position.x;
                vertices[i * 3 + 1] = fluid.particles[i].position.y;
                vertices[i * 3 + 2] = fluid.particles[i].position.z;
            }

            // update the buffer with the new positions
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);
        }

        // draw
        glDrawArrays(GL_POINTS, 0, NUM_PARTICLES);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process input
// -------------
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS) {
        switch (key) {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, true);
            break;
        case GLFW_KEY_R:
            fluid.reset();
            break;
        case GLFW_KEY_P:
            is_paused = !is_paused;
            break;
        }
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// OpenGL 3.1 deprecated the fixed pipeline, so we lose a lot of useful OpenGL
// functions that have to be recreated here.
// ----------------------------------------------------------------------------

Matrix4f getProjectionMatrix() {
    Matrix4f perspective;
    perspective.setZero();

    double cam_near = camera.near_clip();
    double cam_far = camera.far_clip();

    double theta = camera.v_fov() * PI / 360;
    double range = cam_far - cam_near;
    double invtan = 1. / tanf(theta);

    perspective(0, 0) = invtan / camera.aspect_ratio();
    perspective(1, 1) = invtan;
    perspective(2, 2) = -(cam_near + cam_far) / range;
    perspective(3, 2) = -1;
    perspective(2, 3) = -2 * cam_near * cam_far / range;
    perspective(3, 3) = 0;

    return perspective;
}

Matrix4f getViewMatrix() {
    Matrix4f lookAt;
    Matrix3f R;

    lookAt.setZero();

    CGL::Vector3D c_pos = camera.position();
    CGL::Vector3D c_udir = camera.up_dir();
    CGL::Vector3D c_target = camera.view_point();

    Vector3f eye(c_pos.x, c_pos.y, c_pos.z);
    Vector3f up(c_udir.x, c_udir.y, c_udir.z);
    Vector3f target(c_target.x, c_target.y, c_target.z);

    R.col(2) = (eye - target).normalized();
    R.col(0) = up.cross(R.col(2)).normalized();
    R.col(1) = R.col(2).cross(R.col(0));

    lookAt.topLeftCorner<3, 3>() = R.transpose();
    lookAt.topRightCorner<3, 1>() = -R.transpose() * eye;
    lookAt(3, 3) = 1.0f;

    return lookAt;
}