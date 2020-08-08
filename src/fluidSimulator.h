#ifndef CGL_FLUID_SIMULATOR_H
#define CGL_FLUID_SIMULATOR_H

#include <nanogui/nanogui.h>
#include <memory>

#include "camera.h"
#include "fluid.h"
#include "particle.h"
#include "collision/collisionObject.h"

using namespace nanogui;

struct UserShader;

class FluidSimulator {
public:
  FluidSimulator(std::string project_root, Screen *screen);
  ~FluidSimulator();

  void init();

  void loadFluid(Fluid *fluid);
  void loadFluidParameters(FluidParameters *fp);
  void loadCollisionObjects(vector<CollisionObject *> *objects);
  virtual bool isAlive();
  virtual void drawContents();

  // Screen events

  virtual bool cursorPosCallbackEvent(double x, double y);
  virtual bool mouseButtonCallbackEvent(int button, int action, int modifiers);
  virtual bool keyCallbackEvent(int key, int scancode, int action, int mods);
  virtual bool dropCallbackEvent(int count, const char **filenames);
  virtual bool scrollCallbackEvent(double x, double y);
  virtual bool resizeCallbackEvent(int width, int height);

private:
  virtual void initGUI(Screen *screen);
  void drawParticles(GLShader &shader);
  
  void load_shaders();
  
  // File management
  
  std::string m_project_root;

  // Camera methods

  virtual void resetCamera();
  virtual Matrix4f getProjectionMatrix();
  virtual Matrix4f getViewMatrix();

  // Default simulation values

  int frames_per_sec = 90;
  int simulation_steps = 30;

  CGL::Vector3D gravity = CGL::Vector3D(0, -9.8, 0);
  nanogui::Color color = nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f);

  Fluid *fluid;
  FluidParameters *fp;
  vector<CollisionObject *> *collision_objects;

  // OpenGL attributes

  int active_shader_idx = 0;

  vector<UserShader> shaders;
  vector<std::string> shaders_combobox_names;

  // Camera attributes

  CGL::Camera camera;
  CGL::Camera canonicalCamera;

  double view_distance;
  double canonical_view_distance;
  double min_view_distance;
  double max_view_distance;

  double scroll_rate;

  // Screen methods

  Screen *screen;
  void mouseLeftDragged(double x, double y);
  void mouseRightDragged(double x, double y);
  void mouseMoved(double x, double y);

  // Mouse flags

  bool left_down = false;
  bool right_down = false;
  bool middle_down = false;

  // Keyboard flags

  bool ctrl_down = false;

  // Simulation flags

  bool is_paused = true;

  // Screen attributes

  int mouse_x;
  int mouse_y;

  int screen_w;
  int screen_h;

  bool is_alive = true;

  Vector2i default_window_size = Vector2i(1024, 800);
};

struct UserShader {
  UserShader(std::string display_name, std::shared_ptr<GLShader> nanogui_shader)
  : display_name(display_name)
  , nanogui_shader(nanogui_shader) {
  }
  
  std::shared_ptr<GLShader> nanogui_shader;
  std::string display_name;
  
};

#endif // CGL_FLUID_SIM_H
