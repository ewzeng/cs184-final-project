#include <cmath>
#include <glad/glad.h>

#include <CGL/vector3D.h>
#include <nanogui/nanogui.h>

#include "fluidSimulator.h"

#include "camera.h"
#include "fluid.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "misc/camera_info.h"
#include "misc/file_utils.h"
// Needed to generate stb_image binaries. Should only define in exactly one source file importing stb_image.h.
#define STB_IMAGE_IMPLEMENTATION
#include "misc/stb_image.h"

using namespace nanogui;
using namespace std;

void FluidSimulator::load_shaders() {
  std::set<std::string> shader_folder_contents;
  bool success = FileUtils::list_files_in_directory(m_project_root + "/shaders", shader_folder_contents);
  if (!success) {
    std::cout << "Error: Could not find the shaders folder!" << std::endl;
  }
  
  std::string std_vert_shader = m_project_root + "/shaders/Default.vert";
  
  for (const std::string& shader_fname : shader_folder_contents) {
    std::string file_extension;
    std::string shader_name;
    
    FileUtils::split_filename(shader_fname, shader_name, file_extension);
    
    if (file_extension != "frag") {
      std::cout << "Skipping non-shader file: " << shader_fname << std::endl;
      continue;
    }
    
    std::cout << "Found shader file: " << shader_fname << std::endl;
    
    // Check if there is a proper .vert shader or not for it
    std::string vert_shader = std_vert_shader;
    std::string associated_vert_shader_path = m_project_root + "/shaders/" + shader_name + ".vert";
    if (FileUtils::file_exists(associated_vert_shader_path)) {
      vert_shader = associated_vert_shader_path;
    }
    
    std::shared_ptr<GLShader> nanogui_shader = make_shared<GLShader>();
    nanogui_shader->initFromFiles(shader_name, vert_shader,
                                  m_project_root + "/shaders/" + shader_fname);
    
    UserShader user_shader(shader_name, nanogui_shader, hint);
    
    shaders.push_back(user_shader);
    shaders_combobox_names.push_back(shader_name);
  }
  
  // Assuming that it's there, use "Particle" by default
  for (size_t i = 0; i < shaders_combobox_names.size(); ++i) {
    if (shaders_combobox_names[i] == "Particle") {
      active_shader_idx = i;
      break;
    }
  }
}

FluidSimulator::FluidSimulator(std::string project_root, Screen *screen)
: m_project_root(project_root) {
  this->screen = screen;
  
  this->load_shaders();

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
}

FluidSimulator::~FluidSimulator() {
  for (auto shader : shaders) {
    shader.nanogui_shader->free();
  }

  if (fluid) delete fluid;
  if (cp) delete cp;
  if (collision_objects) delete collision_objects;
}

void FluidSimulator::loadFluid(Fluid *fluid) { this->fluid = fluid; }

void FluidSimulator::loadFluidParameters(FluidParameters *fp) { this->fp = fp; }

void FluidSimulator::loadCollisionObjects(vector<CollisionObject *> *objects) { this->collision_objects = objects; }

/**
 * Initializes the fluid simulation and spawns a new thread to separate
 * rendering from simulation.
 */
void FluidSimulator::init() {

  // Initialize GUI
  screen->setSize(default_window_size);
  initGUI(screen);

  // Initialize camera

  CGL::Collada::CameraInfo camera_info;
  camera_info.hFov = 50;
  camera_info.vFov = 35;
  camera_info.nClip = 0.01;
  camera_info.fClip = 10000;

  // Try to intelligently figure out the camera target

  Vector3D avg_pm_position(0, 0, 0);

  for (auto &pm : fluid->particles) {
    avg_pm_position += pm.particles / fluid->particles.size();
  }

  CGL::Vector3D target(avg_pm_position.x, avg_pm_position.y / 2,
                       avg_pm_position.z);
  CGL::Vector3D c_dir(0., 0., 0.);
  canonical_view_distance = max(fluid->width, fluid->height) * 0.9;
  scroll_rate = canonical_view_distance / 10;

  view_distance = canonical_view_distance * 2;
  min_view_distance = canonical_view_distance / 10.0;
  max_view_distance = canonical_view_distance * 20.0;

  // canonicalCamera is a copy used for view resets

  camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
               min_view_distance, max_view_distance);
  canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                        view_distance, min_view_distance, max_view_distance);

  screen_w = default_window_size(0);
  screen_h = default_window_size(1);

  camera.configure(camera_info, screen_w, screen_h);
  canonicalCamera.configure(camera_info, screen_w, screen_h);
}

bool FluidSimulator::isAlive() { return is_alive; }

void FluidSimulator::drawContents() {
  glEnable(GL_DEPTH_TEST);

  if (!is_paused) {
    vector<Vector3D> external_accelerations = {gravity};

    for (int i = 0; i < simulation_steps; i++) {
      fluid->simulate(frames_per_sec, simulation_steps, cp, external_accelerations, collision_objects);
    }
  }

  // Bind the active shader

  const UserShader& active_shader = shaders[active_shader_idx];

  GLShader &shader = *active_shader.nanogui_shader;
  shader.bind();

  // Prepare the camera projection matrix

  Matrix4f model;
  model.setIdentity();

  Matrix4f view = getViewMatrix();
  Matrix4f projection = getProjectionMatrix();

  Matrix4f viewProjection = projection * view;

  shader.setUniform("u_model", model);
  shader.setUniform("u_view_projection", viewProjection);
  shader.setUniform("u_color", color, false); // idk what this does LOL
  drawParticles(shader);

  //for (CollisionObject *co : *collision_objects) {
  //  co->render(shader);
  //}
}

void FluidSimulator::drawParticles(GLShader &shader) {

    // HOW TO DRAW PARTICLES WITH A SHADER? drawArray + attributes???? yayeet lets try that

  //shader.setUniform("u_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f), false);
  shader.uploadAttrib("in_position", positions, false);
  // Commented out: the wireframe shader does not have this attribute
  //shader.uploadAttrib("in_normal", normals);

  shader.drawArray(GL_LINES, 0, num_springs * 2);
}

// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// OpenGL 3.1 deprecated the fixed pipeline, so we lose a lot of useful OpenGL
// functions that have to be recreated here.
// ----------------------------------------------------------------------------

void FluidSimulator::resetCamera() { camera.copy_placement(canonicalCamera); }

Matrix4f FluidSimulator::getProjectionMatrix() {
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

Matrix4f FluidSimulator::getViewMatrix() {
  Matrix4f lookAt;
  Matrix3f R;

  lookAt.setZero();

  // Convert CGL vectors to Eigen vectors
  // TODO: Find a better way to do this!

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

// ----------------------------------------------------------------------------
// EVENT HANDLING
// ----------------------------------------------------------------------------

bool FluidSimulator::cursorPosCallbackEvent(double x, double y) {
  if (left_down && !middle_down && !right_down) {
    if (ctrl_down) {
      mouseRightDragged(x, y);
    } else {
      mouseLeftDragged(x, y);
    }
  } else if (!left_down && !middle_down && right_down) {
    mouseRightDragged(x, y);
  } else if (!left_down && !middle_down && !right_down) {
    mouseMoved(x, y);
  }

  mouse_x = x;
  mouse_y = y;

  return true;
}

bool FluidSimulator::mouseButtonCallbackEvent(int button, int action,
                                              int modifiers) {
  switch (action) {
  case GLFW_PRESS:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = true;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = true;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = true;
      break;
    }
    return true;

  case GLFW_RELEASE:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = false;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = false;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = false;
      break;
    }
    return true;
  }

  return false;
}

void FluidSimulator::mouseMoved(double x, double y) { y = screen_h - y; }

void FluidSimulator::mouseLeftDragged(double x, double y) {
  float dx = x - mouse_x;
  float dy = y - mouse_y;

  camera.rotate_by(-dy * (PI / screen_h), -dx * (PI / screen_w));
}

void FluidSimulator::mouseRightDragged(double x, double y) {
  camera.move_by(mouse_x - x, y - mouse_y, canonical_view_distance);
}

bool FluidSimulator::keyCallbackEvent(int key, int scancode, int action,
                                      int mods) {
  ctrl_down = (bool)(mods & GLFW_MOD_CONTROL);

  if (action == GLFW_PRESS) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
      is_alive = false;
      break;
    case 'r':
    case 'R':
      fluid->reset();
      break;
    case ' ':
      resetCamera();
      break;
    case 'p':
    case 'P':
      is_paused = !is_paused;
      break;
    case 'n':
    case 'N':
      if (is_paused) {
        is_paused = false;
        drawContents();
        is_paused = true;
      }
      break;
    }
  }

  return true;
}

bool FluidSimulator::dropCallbackEvent(int count, const char **filenames) {
  return true;
}

bool FluidSimulator::scrollCallbackEvent(double x, double y) {
  camera.move_forward(y * scroll_rate);
  return true;
}

bool FluidSimulator::resizeCallbackEvent(int width, int height) {
  screen_w = width;
  screen_h = height;

  camera.set_screen_size(screen_w, screen_h);
  return true;
}

void FluidSimulator::initGUI(Screen *screen) {
  Window *window;
  
  window = new Window(screen, "Simulation");
  window->setPosition(Vector2i(default_window_size(0) - 245, 15));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  // Spring types

  new Label(window, "Spring types", "sans-bold");

  {
    Button *b = new Button(window, "structural");
    b->setFlags(Button::ToggleButton);
    b->setPushed(cp->enable_structural_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { cp->enable_structural_constraints = state; });

    b = new Button(window, "shearing");
    b->setFlags(Button::ToggleButton);
    b->setPushed(cp->enable_shearing_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { cp->enable_shearing_constraints = state; });

    b = new Button(window, "bending");
    b->setFlags(Button::ToggleButton);
    b->setPushed(cp->enable_bending_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { cp->enable_bending_constraints = state; });
  }

  // Mass-spring parameters

  new Label(window, "Parameters", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "density :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(cp->density / 10);
    fb->setUnits("g/cm^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { cp->density = (double)(value * 10); });

    new Label(panel, "ks :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(cp->ks);
    fb->setUnits("N/m");
    fb->setSpinnable(true);
    fb->setMinValue(0);
    fb->setCallback([this](float value) { cp->ks = value; });
  }

  // Simulation constants

  new Label(window, "Simulation", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "frames/s :", "sans-bold");

    IntBox<int> *fsec = new IntBox<int>(panel);
    fsec->setEditable(true);
    fsec->setFixedSize(Vector2i(100, 20));
    fsec->setFontSize(14);
    fsec->setValue(frames_per_sec);
    fsec->setSpinnable(true);
    fsec->setCallback([this](int value) { frames_per_sec = value; });

    new Label(panel, "steps/frame :", "sans-bold");

    IntBox<int> *num_steps = new IntBox<int>(panel);
    num_steps->setEditable(true);
    num_steps->setFixedSize(Vector2i(100, 20));
    num_steps->setFontSize(14);
    num_steps->setValue(simulation_steps);
    num_steps->setSpinnable(true);
    num_steps->setMinValue(0);
    num_steps->setCallback([this](int value) { simulation_steps = value; });
  }

  // Damping slider and textbox

  new Label(window, "Damping", "sans-bold");

  {
    Widget *panel = new Widget(window);
    panel->setLayout(
        new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *slider = new Slider(panel);
    slider->setValue(cp->damping);
    slider->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string(cp->damping));
    percentage->setUnits("%");
    percentage->setFontSize(14);

    slider->setCallback([percentage](float value) {
      percentage->setValue(std::to_string(value));
    });
    slider->setFinalCallback([&](float value) {
      cp->damping = (double)value;
      // cout << "Final slider value: " << (int)(value * 100) << endl;
    });
  }

  // Gravity

  new Label(window, "Gravity", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "x :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.x);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.x = value; });

    new Label(panel, "y :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.y);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.y = value; });

    new Label(panel, "z :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.z);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.z = value; });
  }
  
  window = new Window(screen, "Appearance");
  window->setPosition(Vector2i(15, 15));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  // Appearance

  {
    
    
    ComboBox *cb = new ComboBox(window, shaders_combobox_names);
    cb->setFontSize(14);
    cb->setCallback(
        [this, screen](int idx) { active_shader_idx = idx; });
    cb->setSelectedIndex(active_shader_idx);
  }

  // Shader Parameters

  new Label(window, "Color", "sans-bold");

  {
    ColorWheel *cw = new ColorWheel(window, color);
    cw->setColor(this->color);
    cw->setCallback(
        [this](const nanogui::Color &color) { this->color = color; });
  }

  new Label(window, "Parameters", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "Normal :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(this->m_normal_scaling);
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { this->m_normal_scaling = value; });

    new Label(panel, "Height :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(this->m_height_scaling);
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { this->m_height_scaling = value; });
  }
}
