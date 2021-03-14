#include "main.h"

class PurePursuitFollower{
  private:
    double k = 0;
    std::vector<double> velocity;

  public:
    void FollowPath(Path path);
    void generateVelocity();
    void withTurnGain();
    void withMaxVelocity();
    void withGain();
    int closestPoint();
    Vector lookAhead();
};
