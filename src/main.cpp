#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "shader_s.h"
#include "fluid.h"
#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// global variables
bool is_paused = true;
Fluid fluid;


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

    // initalize fluid (somewhat hardcoded-in right now)
    // later we can support reading in parameters from json files
    fluid = Fluid(5, 5, 5);
    float vertices[375];
    for (int i = 0; i < 125; i++) {
        vertices[i * 3] = fluid.particles[i].position.x;
        vertices[i * 3 + 1] = fluid.particles[i].position.y;
        vertices[i * 3 + 2] = fluid.particles[i].position.z;
    }

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
    glPointSize(20);

    // enable depth, Z-buffer
    glEnable(GL_DEPTH_TEST);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO 
    // as the vertex attribute's bound vertex buffer object so afterwards we can safely
    // unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // activate shader
    // ---------------
    ourShader.use();



    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear depth buffer

        if (!is_paused) {
            // update positions of vertices
            // also somewhat hardcoded in
            vector<Vector3D> tmp;
            fluid.simulate(0.0, 0.0, NULL, tmp, NULL);
            for (int i = 0; i < 125; i++) {
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
        glDrawArrays(GL_POINTS, 0, 125);

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