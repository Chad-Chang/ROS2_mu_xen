# Messages
#builtin_interfaces/Time stamp
std_msgs/Header header

bool mode  # simulation:0 or real:1

string[<=30] actuators_name
float32[<=30] pos
float32[<=30] jvel
float32[<=30] jtorque
float32[<=30] jcurrent

float32[<=30] jki
float32[<=30] jkd
float32[<=30] jkp

float32 qdob
float32 qodob
float32 qjdob
