#include "utils.h"

void euler_to_quat(float x, float y, float z, double *q)
{
  double c1 = cos((y) / 2.0);
  double c2 = cos((z) / 2.0);
  double c3 = cos((x) / 2.0);

  double s1 = sin((y) / 2.0);
  double s2 = sin((z) / 2.0);
  double s3 = sin((x) / 2.0);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}
