#include "gui_func.h"

Vector3 GetVector3FromEditText(char *editText)
{
  Vector3 position = {0};
  sscanf(editText, "%f %f %f", &position.x, &position.y, &position.z);
  return position;
}

void GetEditTextFromVector3(Vector3 v, char* editText)
{
  sprintf(editText, "%.3g %.3g %.3g", v.x, v.y, v.z);
}
