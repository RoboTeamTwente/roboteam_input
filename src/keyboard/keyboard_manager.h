#include "roboteam_msgs/RobotCommand.h"

class KeyboardManager {
public:
  KeyboardManager();
  void handleSDLEvents();

private:
  int currentId = 0;
}
