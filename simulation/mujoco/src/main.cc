// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "array_safety.h"
#include "glfw_adapter.h"
#include "simulate.h"
#include <mujoco/mujoco.h>

#include "MuJoCoMessageHandler.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign =
    0.1; // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction =
    0.7;                       // fraction of refresh available for simulation
const int kErrorLength = 1024; // load error string length

// model and data
mjModel *m = nullptr;
mjData *d = nullptr;

// control noise variables
mjtNum *ctrlnoise = nullptr;
std::shared_ptr<deepbreak::MuJoCoMessageHandler::ActuatorCmds> actuator_cmds_ptr;

using Seconds = std::chrono::duration<double>;

//---------------------------------------- plugin handling
//-----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *= 2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError()
                  << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char *path = buf.get();
#else
  const char *path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: "
                  << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif

  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char *filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//------------------------------------------- simulation
//-------------------------------------------

mjModel *LoadModel(const char *file, mj::Simulate &sim) {  // 새로운 모델을 로드할때 , 만약 모델이 있을때, 없을때 
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  RCLCPP_INFO(rclcpp::get_logger("MuJoCo"), "load model from: %s\n", filename);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel *mnew = 0;
  if (mju::strlen_arr(filename) > 4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename) +
                        4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError,
                      mj::Simulate::kMaxFilenameLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n') {
        loadError[error_length - 1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.loadError, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n",
                loadError);
    sim.run = 0;
  }

  return mnew;
}

void apply_ctrl(mjModel *m, mjData *d) { // actuator ID 받아오고
  //  communication에 정의된 msg ActuatorCmds임. => 이거 수정해야헸네
  d-> qpos[10] = 1;
  for (size_t k = 0; k < actuator_cmds_ptr->actuators_name.size(); k++) {
    
    int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR,
                                 actuator_cmds_ptr->actuators_name[k].c_str()); // int mn_name2id(const mjModel* m, int type, const char* name);
    if (actuator_id == -1) {
      RCLCPP_INFO(rclcpp::get_logger("MuJoCo"),
                  "not found the name from the received message in mujoco");
      continue;
    }
    int pos_sensor_id =
        mj_name2id(m, mjOBJ_SENSOR,
                   (actuator_cmds_ptr->actuators_name[k] + "_pos").c_str());
    int vel_sensor_id =
        mj_name2id(m, mjOBJ_SENSOR,
                   (actuator_cmds_ptr->actuators_name[k] + "_vel").c_str());
    
    
    

    // PD제어기
    d->ctrl[actuator_id] = actuator_cmds_ptr->kp[k] *
                               (actuator_cmds_ptr->pos[k] -
                                d->sensordata[m->sensor_adr[pos_sensor_id]]) +
                           actuator_cmds_ptr->kd[k] *
                               (actuator_cmds_ptr->vel[k] -
                                d->sensordata[m->sensor_adr[vel_sensor_id]]) +
                           actuator_cmds_ptr->torque[k];

                      
    d->ctrl[actuator_id] =
        std::min(std::max(-100.0, d->ctrl[actuator_id]), 100.0);
  }
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate &sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (!rclcpp::ok()) {
      sim.exitrequest.store(true);
    }
    // if (sim.droploadrequest.load()) {
    //   mjModel *mnew = LoadModel(sim.dropfilename, sim);
    //   sim.droploadrequest.store(false);

    //   mjData *dnew = nullptr;
    //   if (mnew)
    //     dnew = mj_makeData(mnew);
    //   if (dnew) {
    //     sim.load(sim.dropfilename, mnew, dnew);

    //     mj_deleteData(d);
    //     mj_deleteModel(m);

    //     m = mnew;
    //     d = dnew;
    //     mj_forward(m, d);

    //     // allocate ctrlnoise
    //     free(ctrlnoise);
    //     ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
    //     mju_zero(ctrlnoise, m->nu);
    //   }
    // }

    if (sim.uiloadrequest.load()) { // ui가 sim에서 load가 되었다면
      sim.uiloadrequest.fetch_sub(1);
       //sim data file -> 실제 simulation 랜더링 파일
      mjModel *mnew = LoadModel(sim.filename, sim); // 실제 시뮬레이션에서 렌더링 하는 모델을 로드함. :
      mjData *dnew = nullptr;
      if (mnew)
        dnew = mj_makeData(mnew); // model이 로드가 잘 됬다면 dnew로 model의 정보를 가져오기
      if (dnew) {  //dnew로 모델 정보 잘 가져왔다면 sim instance에 
        sim.load(sim.filename, mnew, dnew);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
        mju_zero(ctrlnoise, m->nu);
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery
    //  life
    if (sim.run && sim.busywait) {  // thread가 busy일때 시뮬레이션 thread를 잠시 쉬어줌.
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::lock_guard<std::mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {          // 모델이 로드되고 시뮬레이션을 돌리라고 하면
        // running
        if (sim.run) {
          // record cpu time at start of iteration : 마지막 cpu 부팅시간 
          const auto startCPU = mj::Simulate::Clock::now(); // clock이라는 클래스의 static 함수

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim; // 

          // inject noise
          if (sim.ctrlnoisestd) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(-m->opt.timestep /
                                  mju_max(sim.ctrlnoiserate, mjMINVAL));
            mjtNum scale = sim.ctrlnoisestd * mju_sqrt(1 - rate * rate);

            for (int i = 0; i < m->nu; i++) {
              // update noise
              ctrlnoise[i] =
                  rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] += ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.realTimeIndex];

          // misalignment condition: distance from target sim time is bigger
          // than syncmisalign
          bool misaligned = mju_abs(Seconds(elapsedCPU).count() / slowdown -
                                    elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          // 뭔가 느려져서 제어 시간이 안지켜졌을때 .
          if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
              syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim.speedChanged) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speedChanged = false;

            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6 * m->nbody);
            sim.applyposepertubations(0); // move mocap bodies only
            sim.applyforceperturbations();

            apply_ctrl(sim.m, sim.d);

            // run single step, let next iteration deal with timing
            mj_step(m, d);
          }

          // 대부분의 경우가 여기서 돌아감. <running>
          // in-sync: step until ahead of cpu
          else { 
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction / sim.refreshRate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim) * slowdown) <
                       mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU <
                       Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measuredSlowdown =
                    std::chrono::duration<double>(elapsedCPU).count() /
                    elapsedSim;
                measured = true;
              }

              // clear old perturbations, apply new
              mju_zero(d->xfrc_applied, 6 * m->nbody);
              sim.applyposepertubations(0); // move mocap bodies only
              sim.applyforceperturbations();

              apply_ctrl(sim.m, sim.d); // 여기에 제어를 넣으면 됨. <제어량 입력> 

              // call mj_step
              mj_step(m, d);

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }
        }

        // paused
        else {
          // apply pose perturbation
          sim.applyposepertubations(1); // move mocap and dynamic bodies

          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
        }
      }
    } // release std::lock_guard<std::mutex>
  }
}
} // namespace

//-------------------------------------- physics_thread
//--------------------------------------------
// 이게 시뮬레이션 무한으로 돌려주는 부분
void PhysicsThread(mj::Simulate *sim, const char *filename) { 
  // 파일 주소 : msghandler -> simulation -> main문의 시뮬레이션 쓰레드 :: 결국 시뮬레이션에서 받아옴.
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d) {
      sim->load(filename, m, d);
      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
  }

  PhysicsLoop(*sim);

  rclcpp::shutdown();

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main
//--------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running
// under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void
_mj_rosettaError(const char *msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, const char **argv) {
  rclcpp::init(argc, argv);
  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  // simulate object encapsulates the UI
  auto sim =
      std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>());

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), nullptr);
  auto message_handle =
      std::make_shared<deepbreak::MuJoCoMessageHandler>(sim.get());
  actuator_cmds_ptr = message_handle->get_actuator_cmds_ptr(); // muujocomsghandler의 actuator cmd가 담긴 포인터를 가져옴
  auto spin_func = [](std::shared_ptr<deepbreak::MuJoCoMessageHandler> node_ptr) 
  {
    rclcpp::spin(node_ptr);
  };
  // auto spin_thread = std::thread(spin_func, message_handle);// spin function using message_handler thread
  std::thread spin_thread(spin_func, message_handle);// spin function using message_handler thread
  // start simulation UI loop (blocking call)
  sim->renderloop(); 
  spin_thread.join();
  physicsthreadhandle.join();

  return 0;
}